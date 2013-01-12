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
 * Copyright 2006 Develer S.r.l. (http://www.develer.com/)
 * -->
 *
 * \version $Id: tc520.h 2506 2009-04-15 08:29:07Z duplo $
 *
 * \brief TC520 ADC driver (intercace)
 *
 * \version $Id: tc520.h 2506 2009-04-15 08:29:07Z duplo $
 * \author Francesco Sacchi <batt@develer.com>
 * \author Marco Benelli <marco@develer.com>
 */

#ifndef DRV_TC520_H
#define DRV_TC520_H

#warning FIXME: This drive is obsolete, you should refactor it.

#include <drv/ser.h>
#include <cfg/compiler.h>

typedef uint32_t tc520_data_t;

/* 17 bit max value */
#define TC520_MAX_VALUE 0x1FFFFUL

tc520_data_t tc520_read(void);
void tc520_init(Serial *spi_port);

#endif  /* DRV_TC520_H */
