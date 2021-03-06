#ifndef _EDT_FT5X06_H
#define _EDT_FT5X06_H

#define EDT_SWAPXY      0x10000000
#define EDT_XREVERSE    0x20000000
#define EDT_YREVERSE    0x40000000
/*
 * Copyright (c) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

struct edt_ft5x06_platform_data {
	int irq_pin;
	int reset_pin;
        int swapxy;  
	/* startup defaults for operational parameters */
	bool use_parameters;
	u8 gain;
	u8 threshold;
	u8 offset;
	u8 report_rate;
};

#endif /* _EDT_FT5X06_H */
