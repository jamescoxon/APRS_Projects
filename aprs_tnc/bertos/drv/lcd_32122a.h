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
 * Copyright 2003, 2004, 2006 Develer S.r.l. (http://www.develer.com/)
 * Copyright 2001 Bernie Innocenti <bernie@codewiz.org>
 *
 * -->
 *
 * \author Bernie Innocenti <bernie@codewiz.org>
 * \author Stefano Fedrigo <aleph@develer.com>
 *
 * \brief Displaytech 32122A LCD driver
 *
 * $WIZ$ module_name = "lcd_32122a"
 * $WIZ$ module_hw = "bertos/hw/hw_lcd_32122a.h"
 * $WIZ$ module_configuration = "bertos/cfg/cfg_lcd_32122a.h"
 * $WIZ$ module_depends = "timer", "pwm"
 */

#ifndef DRV_LCD_32122A_H
#define DRV_LCD_32122A_H

/* Display bitmap dims */
#define LCD_WIDTH 122
#define LCD_HEIGHT 32

#include <gfx/gfx.h> /* Bitmap */

#warning __FILTER_NEXT_WARNING__
#warning this drive is untested..

void lcd_32122_init(void);
void lcd_32122_setPwm(int duty);
void lcd_32122_blitBitmap(const Bitmap *bm);

#endif /* DRV_LCD_32122A_H */
