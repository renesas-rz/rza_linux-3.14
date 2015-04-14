#ifndef _CAN_PLATFORM_RZA1_CAN_H_
#define _CAN_PLATFORM_RZA1_CAN_H_

#include <linux/types.h>

/* Clock Select Register settings */
#define CLKR_CLKC		0
#define CLKR_CLK_XINCAN		1

struct rz_can_platform_data {
	int clock_select;	/* Clock source select */
	int channel;
};

#endif	/* !_CAN_PLATFORM_RZA1_CAN_H_ */
