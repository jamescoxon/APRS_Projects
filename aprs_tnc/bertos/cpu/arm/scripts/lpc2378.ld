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
 * \author Francesco sacchi <batt@develer.com>
 *
 * \brief Linker Script for NXP LPC2378 processors.
 *
 */


MEMORY
{
	/* last 12k used by the bootloader */
	rom(rx)  : org = 0x00000000, len = 512k - 12k
	/* First 0x200 bytes and last 288 used by bootloader and debugger */
	ram(rwx) : org = 0x40000200, len = 32k - 0x200 - 288
}

/* USB RAM */
__usb_ram_start = 0x7FD00000;
__usb_ram_size  = 8k;
__usb_ram_end   = __usb_ram_start + __usb_ram_size;

/* Ethernet RAM */
__eth_ram_start = 0x7FE00000;
__eth_ram_size  = 16k;
__eth_ram_end   = __eth_ram_start + __eth_ram_size;

INCLUDE "bertos/cpu/arm/scripts/arm7tdmi_rom.ld"
