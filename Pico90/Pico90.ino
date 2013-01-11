/*
* Eurus 70cm Tracker (434.200Mhz) - James Coxon jacoxon@googlemail.com
 * Long duration, East to West, High Altitude balloon flight - based up on the code of PicoAtlas.
 * Components - Arduino328, UBlox 6 GPS (Falcom FSA-03), RFM22b Radio
 *
 * Continous Transmission of RTTY.
 *
 * Latest code can be found: https://github.com/jamescoxon/Eurus
 *
 * GPS Code from jonsowman and Joey flight computer CUSF
 * https://github.com/cuspaceflight/joey-m/tree/master/firmware
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <util/crc16.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <avr/pgmspace.h>    // needed for PROGMEM
#include "ax25modem.h"
#include "geofence.h"

static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (25)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

// Data to be transmitted
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

#define ONEPPM 1.0e-6
#define DEBUG false

//Variables
int32_t lat = 514981000, lon = -530000, alt = 36000;
uint8_t hour = 0, minute = 0, second = 0, month = 0, day = 0, lock = 0, sats = 0;
int GPSerror = 0, count = 1, n, gpsstatus, navmode = 0, battV = 0;
int elevation = 0, azimuth = 0, aprs_status = 0, aprs_attempts = 0;
long int doppler = 0;

uint8_t buf[60]; //GPS receive buffer
char superbuffer [80]; //Telem string buffer

char comment[3];

//************Other Functions*****************

static int pointinpoly(const int32_t *poly, int points, int32_t x, int32_t y)
{
  int32_t p0, p1, l0, l1;
  int c = 0;

  /* Read the final point */
  p0 = pgm_read_dword(&poly[points * 2 - 2]);
  p1 = pgm_read_dword(&poly[points * 2 - 1]);

  for(; points; points--, poly += 2)
  {
    l0 = p0;
    l1 = p1;
    p0 = pgm_read_dword(&poly[0]);
    p1 = pgm_read_dword(&poly[1]);

    if(y < p1 && y < l1) continue;
    if(y >= p1 && y >= l1) continue;
    if(x < p0 + (l0 - p0) * (y - p1) / (l1 - p1)) continue;

    c = !c;
  }

  return(c);
}

int geofence_location(int32_t lat_poly, int32_t lon_poly)
{
  if(pointinpoly(UKgeofence, 50, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = ' ';
  }

  else if(pointinpoly(Netherlands_geofence, 50, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'P';
  }

  else if(pointinpoly(Belgium_geofence, 60, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'O';
  }

  else if(pointinpoly(France_geofence, 60, lat_poly, lon_poly) == true)
  {
    comment[0] = ' ';
    comment[1] = 'F';
  }

}

void tx_aprs()
{
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  Serial.print(aprs_lat);
  Serial.print(" ");
  aprs_lon = 900000000 + lon / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  Serial.println(aprs_lon);

  int32_t aprs_alt = alt * 32808 / 10000;

  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);

  ax25_frame(
  APRS_CALLSIGN, APRS_SSID,
  "APRS", 0,
  //0, 0, 0, 0,
  "WIDE1", 1, "WIDE2",1,
  //"WIDE2", 1,
  "!/%s%sO   /A=%06ld|%s|%s/M6JCX,%d",
  ax25_base91enc(slat, 4, aprs_lat),
  ax25_base91enc(slng, 4, aprs_lon),
  aprs_alt, stlm, comment, count
    );

  seq++;
}

ISR(TIMER2_OVF_vect)
{
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;

  /* Update the PWM output */
  OCR2A = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */
        //PORTA &= ~TXENABLE;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;

        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2A */
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  /* Make sure radio is not enabled */
  //PORTA &= ~TXENABLE;

  /* Enable pins for output (Port A pin 4 and Port D pin 7) */
  //DDRA |= TXENABLE;
  pinMode(11, OUTPUT);
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  //PORTA |= TXENABLE;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

void send_APRS() {
  ax25_init();

  Serial.print("Sending ");
  digitalWrite(6, HIGH);
  delay(500);
  digitalWrite(4, HIGH);
  delay(1000);
  tx_aprs();
  delay(1000);
  digitalWrite(6, LOW);
  digitalWrite(4, LOW);
  Serial.println("Sent");
}

void setup() {
  Serial.begin(9600);
  pinMode(6, OUTPUT); //enable  
  pinMode(4, OUTPUT); //stepup
  digitalWrite(6, LOW);
  digitalWrite(4, LOW);

  Serial.println("Running...");

}

void loop() {
  count++;

  //  if(count % 2 == 0)
  // {
  //  geofence_location(lat, lon);
  // }

  aprs_status = 1;
  //Transmit APRS data now
  send_APRS();
  aprs_attempts++;

  delay(5000);

}



