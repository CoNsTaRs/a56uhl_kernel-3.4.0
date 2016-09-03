/* Copyright (c) 2015, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GEN_VKEYS_
#define VKEY_LOGI(fmt, args...) printk(KERN_INFO "[TP] VK: "fmt, ##args)
#define VKEY_LOGE(fmt, args...) printk(KERN_ERR "[TP][ERR] VK: "fmt, ##args)

struct vkeys_button_data {
	int keycode;
	int center_x;
	int center_y;
	const char *range;
};

struct vkeys_platform_data {
	const char *name;
	int num_keys;
	struct vkeys_button_data *button;
};
#endif
