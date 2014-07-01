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
#include <linux/sh_eth.h>
#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/r7s72100.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

/* Ether */
static const struct sh_eth_plat_data ether_pdata __initconst = {
	.phy			= 0x00, /* PD60610 */
	.edmac_endian		= EDMAC_LITTLE_ENDIAN,
	.phy_interface		= PHY_INTERFACE_MODE_MII,
	.no_ether_link		= 1
};

static const struct resource ether_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8203000, 0x800),
	DEFINE_RES_MEM(0xe8204800, 0x200),
	DEFINE_RES_IRQ(gic_iid(359)),
};

static const struct platform_device_info ether_info __initconst = {
	.parent		= &platform_bus,
	.name		= "r7s72100-ether",
	.id		= -1,
	.res		= ether_resources,
	.num_res	= ARRAY_SIZE(ether_resources),
	.data		= &ether_pdata,
	.size_data	= sizeof(ether_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

static void __init rskrza1_add_standard_devices(void)
{
	r7s72100_clock_init();
	r7s72100_add_dt_devices();

	platform_device_register_full(&ether_info);

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
