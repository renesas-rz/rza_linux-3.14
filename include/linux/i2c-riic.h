/*
 * i2c-riic.h
 * Copyright (c) 2014 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_I2C_RIIC_H
#define _LINUX_I2C_RIIC_H

/**
 * struct riic_i2c_platform_data - Platform data of Renesas I2C Driver
 * @bus_rate:		Frequency of bus clock in Hz
 *
 */
struct riic_platform_data {
	u32	bus_rate;
};

#endif /* _LINUX_I2C_RIIC_H */
