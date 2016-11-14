#ifndef __ASM_R7S72100_H__
#define __ASM_R7S72100_H__

enum pfc_pin_number {
	P0_0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8,
	P0_9, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15,
	P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8,
	P1_9, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15,
	P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7, P2_8,
	P2_9, P2_10, P2_11, P2_12, P2_13, P2_14, P2_15,
	P3_0, P3_1, P3_2, P3_3, P3_4, P3_5, P3_6, P3_7, P3_8,
	P3_9, P3_10, P3_11, P3_12, P3_13, P3_14, P3_15,
	P4_0, P4_1, P4_2, P4_3, P4_4, P4_5, P4_6, P4_7, P4_8,
	P4_9, P4_10, P4_11, P4_12, P4_13, P4_14, P4_15,
	P5_0, P5_1, P5_2, P5_3, P5_4, P5_5, P5_6, P5_7, P5_8,
	P5_9, P5_10, P5_11, P5_12, P5_13, P5_14, P5_15,
	P6_0, P6_1, P6_2, P6_3, P6_4, P6_5, P6_6, P6_7, P6_8,
	P6_9, P6_10, P6_11, P6_12, P6_13, P6_14, P6_15,
	P7_0, P7_1, P7_2, P7_3, P7_4, P7_5, P7_6, P7_7, P7_8,
	P7_9, P7_10, P7_11, P7_12, P7_13, P7_14, P7_15,
	P8_0, P8_1, P8_2, P8_3, P8_4, P8_5, P8_6, P8_7, P8_8,
	P8_9, P8_10, P8_11, P8_12, P8_13, P8_14, P8_15,
	P9_0, P9_1, P9_2, P9_3, P9_4, P9_5, P9_6, P9_7, P9_8,
	P9_9, P9_10, P9_11, P9_12, P9_13, P9_14, P9_15,
	P10_0, P10_1, P10_2, P10_3, P10_4, P10_5, P10_6, P10_7, P10_8,
	P10_9, P10_10, P10_11, P10_12, P10_13, P10_14, P10_15,
	P11_0, P11_1, P11_2, P11_3, P11_4, P11_5, P11_6, P11_7, P11_8,
	P11_9, P11_10, P11_11, P11_12, P11_13, P11_14, P11_15,
	GPIO_NR,
};

enum pfc_mode {
	PMODE = 0,
	ALT1, ALT2, ALT3, ALT4, ALT5, ALT6, ALT7, ALT8,
	PINMUX_STATE_NUM,
};

enum pfc_direction {
	DIIO_PBDC_DIS = 0,	/* Direct I/O Mode & PBDC Disable */
	DIIO_PBDC_EN,		/* Direct I/O Mode & PBDC Enable */
	SWIO_OUT_PBDCDIS,	/* Software I/O Mode & Output direction PBDC Disable */
	SWIO_OUT_PBDCEN,	/* Software I/O Mode & Output direction PBDC Enable */
	PORT_OUT_HIGH,		/* Port Mode & Output direction & High Level Output Pn = 1 */
	PORT_OUT_LOW,		/* Port Mode & Output direction & Low Level Output Pn = 0 */
	DIR_OUT,
	DIR_IN,			/* Port Mode or Software I/O Mode is Direction IN */
	DIR_LVDS,
};

void r7s72100_add_dt_devices(void);
void r7s72100_clock_init(void);
void r7s72100_init_early(void);
int r7s72100_pinmux_setup(void);
int r7s72100_pfc_pin_assign(enum pfc_pin_number pinnum, enum pfc_mode mode,
			       enum pfc_direction dir);

#endif /* __ASM_R7S72100_H__ */
