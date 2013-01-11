/*
 AVA 70cms Tracker
 
 By Anthony Stirk M0UPU 
 
 October 2012 Version 3
 Subversion 3.24
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Thanks to : 
 http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 Big thanks to Dave Akerman!
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/sleep.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RF22.h>
RF22 rf22;
/* CONFIGURABLE BITS */
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50     // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.331
#define RADIO_POWER  0x04
/*
                       0x02  5db (3mW)
 0x03  8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */

#define RFM22B_PIN 10
#define RFM22B_SDN 3
#define STATUS_LED 4            // PAVA R7 Boards have an LED on PIN4

#define POWERSAVING      // Comment out to turn power saving off

uint8_t buf[60]; 
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int navmode = 0, battv=0, rawbattv=0, GPSerror = 0, lat_int=0,lon_int=0,txnulls=10;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0, battvaverage=0;
int psm_status = 0;
int rfm_temp;
int32_t tslf=0;
int errorstatus=0; /* Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = RFM22B Error Condition Noted, RFM22B Power Cycled
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 */
int test=0;

void setup() {
  pinMode(STATUS_LED, OUTPUT); 
  blinkled(6);
  Serial.begin(9600);
  blinkled(5);
  resetGPS();
  blinkled(4);
  wait(500);
  blinkled(3);
  setupRadio();
  blinkled(2);
  setupGPS();
  blinkled(1);
  initialise_interrupt();

#ifdef POWERSAVING
  ADCSRA = 0;
#endif

}

void loop()
{
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();

  if(lock!=3) // Blink LED to indicate no lock
  {
    digitalWrite(STATUS_LED, HIGH);   
    wait(750);               
    digitalWrite(STATUS_LED, LOW); 
    errorstatus |=(1 << 5);     
  }
  else
  {
    errorstatus &= ~(1 << 5);
  }

  if(alt<1000) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);      
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3);

    }
  }

#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))
  {
    setGPS_PowerSaveMode();
    wait(1000);
    pinMode(STATUS_LED, INPUT); 
    psm_status=1;
    errorstatus &= ~(1 << 4);
  }
#endif

  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt)
    {
      maxalt=alt;
    }

    if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=3) {
      tslf++;
    }
    else
    {
      tslf=0;
      errorstatus &= ~(1 << 0);
      errorstatus &= ~(1 << 1);
    }
    if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
      setupGPS();
      wait(125);
      setGps_MaxPerformanceMode();
      wait(125);
      //    errorstatus=1;
      errorstatus |=(1 << 0);
      psm_status=0;
      errorstatus |=(1 << 4); 
    }
    if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
      errorstatus |=(1 << 0);
      errorstatus |=(1 << 1);
      Serial.flush();
      resetGPS();
      wait(125);
      setupGPS();
    }
    rfm_temp = (rf22.temperatureRead( RF22_TSRANGE_M64_64C,0 ) / 2) - 64;
    if(rf22.spiRead(0x07) != 0x09 )
    {    
      digitalWrite(RFM22B_SDN, HIGH);
      errorstatus |=(1 << 2);
      wait(500);
      setupRadio();
      wait(500);
    }
  }
}    

void setupGPS() {
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  wait(1000);
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  wait(1000);
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  wait(1000);
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  wait(1000);
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  wait(1000);
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  wait(1000);
  setGPS_DynamicModel3();
  wait(1000);
}
void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84                                               };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }


  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                      };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                               };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}

void setGPS_DynamicModel3()
{
  int gps_set_sucess=0;
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76                                               };
  while(!gps_set_sucess)
  {
    sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A                                                                                                  };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
      (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/100;
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
      (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/100;


    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
      (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67                                                                                                };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(hour > 23 || minute > 59 || second > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission

    if(alt>maxalt)
    {
      maxalt=alt;
    }
    lockvariables=1;
    sprintf(txstring, "$$$$$AVA,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,rfm_temp);
    sprintf(txstring, "%s,%i",txstring,errorstatus);
    sprintf(txstring, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
    maxalt=0;
    lockvariables=0;
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=0; // Should be finished
      txj=0;
      count++;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);	
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }

  }
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    rf22.spiWrite(0x73,0x03); // High
  }
  else
  {
    rf22.spiWrite(0x73,0x00); // Low
  }
}

void setupRadio(){
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  wait(1000);
  rf22.init();
  rf22.setTxPower(RADIO_POWER);
  rf22.setFrequency(RADIO_FREQUENCY);
  rf22.setModeTx();
  rf22.setModemConfig(RF22::UnmodulatedCarrier);
  rf22.spiWrite(0x073, 0x03); 

}

void setGPS_Cyclic() {
  // Update Period 10 seconds , do not enter 'inactive for search' state when no fix unchecked
  uint8_t setCyclic[] = { 
    0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 
    0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 
    0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 
    0x00, 0x64, 0x40, 0x01, 0x00, 0xE3, 0x65                                   };
  sendUBX(setCyclic, sizeof(setCyclic)/sizeof(uint8_t));
}

void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92                                                               }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91                                                               }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}
void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5                                     };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void prepare_data() {

  gps_check_lock();
  gps_get_position();
  gps_get_time();
}
void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(STATUS_LED,HIGH);
    wait(100);
    digitalWrite(STATUS_LED,LOW);
    wait(100);
  }    
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}




















