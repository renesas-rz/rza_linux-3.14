/*
 * RSKRZA1 board support
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Magnus Damm
 * Copyright (C) 2014  Chris Brandt
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
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/common.h>
#include <mach/r7s72100.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

static void __init rskrza1_add_standard_devices(void)
{
	r7s72100_clock_init();
	r7s72100_add_dt_devices();
}

static const char * const rskrza1_boards_compat_dt[] __initconst = {
	"renesas,rskrza1",
	NULL,
};

DT_MACHINE_START(RSKRZA1_DT, "rskrza1")
	.init_early	= r7s72100_init_early,
	.init_machine	= rskrza1_add_standard_devices,
	.dt_compat	= rskrza1_boards_compat_dt,
MACHINE_END
