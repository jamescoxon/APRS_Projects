/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \author Francesco Sacchi <batt@develer.com>
 * \author Luca Ottaviano <lottaviano@develer.com>
 * \author Daniele Basile <asterix@develer.com>
 *
 * \brief Arduino APRS radio demo.
 *
 * This example shows how to read and decode APRS radio packets.
 * It uses the following modules:
 * afsk
 * ax25
 * ser
 *
 * You will see how to use a serial port to output messages, init the afsk demodulator and
 * how to parse input messages using ax25 module.
 */

#include <cpu/irq.h>
#include <cfg/debug.h>

#include <net/afsk.h>
#include <net/ax25.h>

#include <drv/ser.h>
#include <drv/timer.h>

#include <stdio.h>
#include <string.h>

static Afsk afsk;
static AX25Ctx ax25;
static Serial ser;

#define ADC_CH 0

// really? you need a prototype for the very next line? mmm k
void KISS_ify(struct AX25Call org, char *kiss_str);


// Ok, got that prototype now. it's safe begin the code

void KISS_ify(struct AX25Call org, char *kiss_str)
{
    // 20101123 Robert Marshall KI4MCW
    // Bit-shift callsigns in KISS header
    
    uint8_t j ;
    char tmp ;
    
    for (j=0 ; j<6 ; j++)
    {
        tmp = org.call[j] ;
        if ( tmp == 0 ) { tmp = 0x20 ; }
        kiss_str[j] = (uint8_t)( tmp * 2 ) ;
    }
    
    kiss_str[6] = ( 0x60 + ( 2 * org.ssid ) ) & 0xFF ;
    kiss_str[7] = 0 ;
}


static void message_callback(struct AX25Msg *msg)
{
    // Original BeRTOS demo code:
    
	//kfile_printf(&ser.fd, "\n\nSRC[%.6s-%d], DST[%.6s-%d]\r\n", msg->src.call, msg->src.ssid, msg->dst.call, msg->dst.ssid);
	//for (int i = 0; i < msg->rpt_cnt; i++)
    //	kfile_printf(&ser.fd, "via: [%.6s-%d]\r\n", msg->rpt_lst[i].call, msg->rpt_lst[i].ssid);
	//kfile_printf(&ser.fd, "DATA: %.*s\r\n", msg->len, msg->info);
    
    
    // 20101123 Robert Marshall KI4MCW
    // KISS output of imcoming data, for use with Xastir, et al.
    
    uint8_t i ;
    char kiss_call[8], kiss_output[200] ;
    
    // DCD light on Digital8 pin (too short to be effective, but it works)
    DDRB |= 0x01 ;
    PORTB |= 0x01 ;
    
    memset( kiss_output, 0, 200 ) ;
    
    // build KISS header (bit-shifted)
    
    KISS_ify( msg->dst , kiss_call ) ;
    strcat( kiss_output , kiss_call ) ;
    
    KISS_ify( msg->src , kiss_call ) ;
    strcat( kiss_output , kiss_call ) ;
    
    for ( i=0 ; i < msg->rpt_cnt ; i++ )
    {
        KISS_ify( msg->rpt_lst[i] , kiss_call ) ;
        // high bit marks that packet has already crossed this digi
        if ( msg->rpt_used[i] ) { kiss_call[6] |= 0x80 ; }
        strcat( kiss_output , kiss_call ) ;
    }
    
    // low bit high on last digi's SSID means end-of-header
    i = strlen( kiss_output ) ;
    kiss_output[i - 1]++ ;
    
    // FCS
    strcat( kiss_output , "\x03\xF0" ) ;
    // rest of packet (not bit-shifted)
    strncat( kiss_output , msg->info , msg->len ) ;
    
    // output to serial
    
    kfile_putc( 0xC0 , &ser.fd ) ;
    kfile_putc( 0x00 , &ser.fd ) ;
    kfile_printf(&ser.fd, "%.*s\xC0" , strlen(kiss_output) , kiss_output );
    
    // turn off DCD
    PORTB &= 0xFE ;
}

static void init(void)
{
	IRQ_ENABLE;
	kdbg_init();
	timer_init();
    
	/*
	 * Init afsk demodulator. We need to implement the macros defined in hw_afsk.h, which
	 * is the hardware abstraction layer.
	 * We do not need transmission for now, so we set transmission DAC channel to 0.
	 */
	afsk_init(&afsk, ADC_CH, 0);
	/*
	 * Here we initialize AX25 context, the channel (KFile) we are going to read messages
	 * from and the callback that will be called on incoming messages.
	 */
	ax25_init(&ax25, &afsk.fd, message_callback);
    
	/* Initialize serial port, we are going to use it to show APRS messages*/
	ser_init(&ser, SER_UART0);
	ser_setbaudrate(&ser, 9600);
    kfile_printf(&ser.fd, "BOOTED\r\n");
}

//static AX25Call path[] = AX25_PATH(AX25_CALL("apzbrt", 0),
//                                   AX25_CALL("nocall", 0),
//                                   AX25_CALL("wide1", 1),
//                                   AX25_CALL("wide2", 2));
//
//#define APRS_MSG    ">Test BeRTOS APRS http://www.bertos.org"

int count = 0;

int main(void)
{
	init();
	ticks_t start = timer_clock();

	while (1)
	{
		/*
		 * This function will look for new messages from the AFSK channel.
		 * It will call the message_callback() function when a new message is received.
		 * If there's nothing to do, this function will call cpu_relax()
		 */
		ax25_poll(&ax25);


		/* Send out message every 60sec */
		if (timer_clock() - start > ms_to_ticks(60000L))
		{
			start = timer_clock();
			kfile_printf(&ser.fd, "TNC %d\r\n", count);
            count++;
		}
	}
	return 0;
}
