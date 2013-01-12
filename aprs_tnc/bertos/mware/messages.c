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
 * Copyright 2003, 2005 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \brief Messages for LCD.
 *
 * \version $Id: messages.c 3215 2010-03-17 11:45:15Z arighi $
 *
 * \author Bernie Innocenti <bernie@codewiz.org>
 * \author Stefano Fedrigo <aleph@develer.com>
 */

#include "messages.h"

/**
 * Array of pointers to localized strings. Should be filled
 * by localization stuff, but not for now.
 */
const char *msg_strings[MSG_COUNT] = {
	0,
	// TODO: add your strings here
};

/* Buffer for catalog file */
/* char msg_buf[MSG_BUFSIZE]; */


/* The following does not work (move string tables into the DMSG/CMSG segments)
 * #pragma memory=dataseg(DMSG)
 * #pragma memory=constseg(CMSG)
 */


/**
 * Untranslated constant strings used more than once are
 * grouped here to save ROM space.
 */
const char str_empty[] = "";

