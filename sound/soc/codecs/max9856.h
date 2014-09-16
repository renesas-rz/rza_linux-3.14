/*
 * max9856.h  --  codec driver for max9856
 *
 * Copyright (C) 2011 taskit GmbH
 * Author: Christian Glindkamp <christian.glindkamp@taskit.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _MAX9856_H
#define _MAX9856_H

#define MAX9856_STATUSA			0x00
#define MAX9856_STATUSB			0x01
#define MAX9856_INTERRUPT		0x02
#define MAX9856_CLOCK			0x03
#define MAX9856_DAC_SYS			0x04
#define MAX9856_DAC_INTA		0x05
#define MAX9856_DAC_INTB		0x06
#define MAX9856_ADC_SYS			0x07
#define MAX9856_ADC_INTA		0x08
#define MAX9856_ADC_INTB		0x09
#define MAX9856_ADC_LEVEL		0x0A
#define MAX9856_HPF			0x0B
#define MAX9856_AGC_CONTROL		0x0C
#define MAX9856_AGC_THRESH		0x0D
#define MAX9856_MIX_IN_L		0x0E
#define MAX9856_MIX_IN_R		0x0F
#define MAX9856_MIX_OUT			0x10
#define MAX9856_DIGITAL_GAIN		0x11
#define MAX9856_AUXIN_GAIN		0x12
#define MAX9856_LINEIN1_GAIN		0x13
#define MAX9856_LINEIN2_GAIN		0x14
#define MAX9856_MICL_GAIN		0x15
#define MAX9856_MICR_GAIN		0x16
#define MAX9856_MIC_MODE		0x17
#define MAX9856_HPL_VOL			0x18
#define MAX9856_HPR_VOL			0x19
#define MAX9856_OUT_MODE		0x1A
#define MAX9856_HEADSET			0x1B
#define MAX9856_PM			0x1C

#define MAX9856_CACHEREGNUM 29

#define MAX9856_CLOCK_PSCLK(n)		((n & 0x7) << 4)
#define MAX9856_CLOCK_MAS		(1 << 3)
#define MAX9856_CLOCK_BSEL(n)		((n & 0x7) << 0)

#define MAX9856_DAC_SYS_DWCI		(1 << 7)
#define MAX9856_DAC_SYS_DBCI		(1 << 6)
#define MAX9856_DAC_SYS_DRATE(n)	((n & 0x3) << 4)
#define MAX9856_DAC_SYS_DDLY		(1 << 3)
#define MAX9856_DAC_SYS_PCM		(1 << 2)
#define MAX9856_DAC_SYS_DHF		(1 << 1)
#define MAX9856_DAC_SYS_WS		(1 << 0)

#define MAX9856_ADC_SYS_AWCI		(1 << 7)
#define MAX9856_ADC_SYS_ABCI		(1 << 6)
#define MAX9856_ADC_SYS_APIN_GPI	(0 << 4)
#define MAX9856_ADC_SYS_APIN_WC		(1 << 4)
#define MAX9856_ADC_SYS_APIN_GPO_L	(2 << 4)
#define MAX9856_ADC_SYS_APIN_GPO_H	(3 << 4)
#define MAX9856_ADC_SYS_ADLY		(1 << 3)

#define MAX9856_MIX_OUT_MXOUTL_SHIFT	(4)
#define MAX9856_MIX_OUT_MXOUTR_SHIFT	(0)

#define MAX9856_PM_SHDN_SHIFT		(7)
#define MAX9856_PM_DIGEN_SHIFT		(5)
#define MAX9856_PM_LOUTEN_SHIFT		(4)
#define MAX9856_PM_DALEN_SHIFT		(3)
#define MAX9856_PM_DAREN_SHIFT		(2)
#define MAX9856_PM_ADLEN_SHIFT		(1)
#define MAX9856_PM_ADREN_SHIFT		(0)

#endif
