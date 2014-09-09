/*
 * RZ/A1 Timer Driver - OSTM
 *
 * Copyright (C) 2014 Renesas Solutions Corp.
 *
 * Based on include/linux/sh_timer.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef _RZA1_OSTM_H_
#define _RZA1_OSTM_H_

struct rza1_ostm_pdata {
	struct {
		char *name;
		unsigned long rating;
	} clksrc;
	struct {
		char *name;
		unsigned long rating;
	} clkevt;
};

#endif /* _RZA1_OSTM_H_ */
