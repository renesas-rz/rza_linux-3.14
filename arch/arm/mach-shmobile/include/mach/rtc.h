/*
 * Copyright (C) 2013 Renesas Solutions Corp.
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
 */
#ifndef _ASM_RTC_H
#define _ASM_RTC_H

void time_init(void);
extern void (*board_time_init)(void);
extern void (*rtc_sh_get_time)(struct timespec *);
extern int (*rtc_sh_set_time)(const time_t);

/* some dummy definitions */
#define RTC_BATT_BAD 0x100	/* battery bad */
#define RTC_SQWE 0x08		/* enable square-wave output */
#define RTC_DM_BINARY 0x04	/* all time/date values are BCD if clear */
#define RTC_24H 0x02		/* 24 hour mode - else hours bit 7 means pm */
#define RTC_DST_EN 0x01	        /* auto switch DST - works f. USA only */

struct rtc_time;
unsigned int get_rtc_time(struct rtc_time *);
int set_rtc_time(struct rtc_time *);

#define RTC_CAP_4_DIGIT_YEAR	(1 << 0)

struct sh_rtc_platform_info {
	unsigned long capabilities;
};

#define rtc_reg_size            sizeof(u16)

#define RTC_BIT_INVERTED        0x40    /* bug on SH7750, SH7750S */
#define RTC_DEF_CAPABILITIES    RTC_CAP_4_DIGIT_YEAR

#endif /* _ASM_RTC_H */
