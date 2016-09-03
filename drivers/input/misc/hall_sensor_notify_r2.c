/*
 *  drivers/input/misc/hall_sensor_notify_r2.c
 *
 *  Copyright (C) 2014 HTC Corporation.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/hall_sensor_r2.h>
#include <linux/module.h>

BLOCKING_NOTIFIER_HEAD(hallsensor_notifier_list_r2);
int hallsensor_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hallsensor_notifier_list_r2, nb);
}
EXPORT_SYMBOL(hallsensor_register_notifier);

int hallsensor_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hallsensor_notifier_list_r2, nb);
}
EXPORT_SYMBOL(hallsensor_unregister_notifier);

int hallsensor_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hallsensor_notifier_list_r2, val, v);
}
EXPORT_SYMBOL_GPL(hallsensor_notifier_call_chain);
