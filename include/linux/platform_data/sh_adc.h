/*
 * SH ADC platform data
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
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
#ifndef __LINUX_PLATFORM_DATA_SH_ADC_H__
#define __LINUX_PLATFORM_DATA_SH_ADC_H__

struct sh_adc_data {
	u8	num_channels;
	u8	mtu2_ch;
};

#endif /* __LINUX_PLATFORM_DATA_SH_ADC_H__ */

