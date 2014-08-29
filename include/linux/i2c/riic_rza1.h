/*
 * RIIC bus driver
 *
 * Copyright (C) 2011-2013  Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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
 */

#ifndef _RIIC_H_
#define _RIIC_H_

struct riic_platform_data {
	int	clock;		/* i2c clock (kHZ) */
};

struct riic_core_packet {
	unsigned short slave_address;
	void *data;
	int len;
	int buf_idx;
	unsigned char rw;
#define RIIC_CORE_RW_MASTER_TRANSMIT	0
#define RIIC_CORE_RW_MASTER_RECEIVE	1
#define RIIC_CORE_RW_SLAVE_TRANSMIT	2
#define RIIC_CORE_RW_SLAVE_RECEIVE	3

	unsigned done:1;
};

#endif
