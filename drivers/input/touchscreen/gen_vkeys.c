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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/input/gen_vkeys.h>

#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	"0x01"

static struct kobject *vkey_obj;
static char *vkey_buf;

static ssize_t vkey_show(struct kobject  *obj,
		struct kobj_attribute *attr, char *buf)
{
	strlcpy(buf, vkey_buf, MAX_BUF_SIZE);
	return strnlen(buf, MAX_BUF_SIZE);
}

static struct kobj_attribute vkey_obj_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = vkey_show,
};

static struct attribute *vkey_attr[] = {
	&vkey_obj_attr.attr,
	NULL,
};

static struct attribute_group vkey_grp = {
	.attrs = vkey_attr,
};

static int __devinit vkey_parse_dt(struct device *dev,
			struct vkeys_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *pp = NULL;
	struct vkeys_button_data *vkey;
	int rc, parsed_vkeys = 0;

	rc = of_property_read_string(np, "label", &pdata->name);
	if (rc) {
		VKEY_LOGE("Failed to read label\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "vkey-num", &pdata->num_keys);
	if (rc) {
		VKEY_LOGE("Failed to read key number\n");
		return -EINVAL;
	}

	pdata->button = devm_kzalloc(dev,
		sizeof(struct vkeys_button_data) * pdata->num_keys, GFP_KERNEL);
	if (!pdata->button) {
		VKEY_LOGE("Failed to allocate vkey button memory\n");
		return -ENOMEM;
	}

	while ((pp = of_get_next_child(np, pp))) {
		vkey = &pdata->button[parsed_vkeys];
		rc = of_property_read_u32(pp, "keycode", &vkey->keycode);
		if (rc) {
			VKEY_LOGE("%d: Failed to get property: keycode\n", parsed_vkeys);
			return -EINVAL;
			break;
		}
		rc = of_property_read_u32(pp, "center-x", &vkey->center_x);
		if (rc) {
			VKEY_LOGE("%d: Failed to get property: center-x\n", parsed_vkeys);
			return -EINVAL;
			break;
		}
		rc = of_property_read_u32(pp, "center-y", &vkey->center_y);
		if (rc) {
			VKEY_LOGE("%d: Failed to get property: center_y\n", parsed_vkeys);
			return -EINVAL;
			break;
		}
		rc = of_property_read_string(pp, "range", &vkey->range);
		if (rc) {
			VKEY_LOGE("%d: Failed to get property: range\n", parsed_vkeys);
			return -EINVAL;
		}

		parsed_vkeys++;
	}
	return 0;
}

static int __devinit vkeys_probe(struct platform_device *pdev)
{
	struct vkeys_platform_data *pdata;
	int i, c = 0, ret;
	char *name;

	VKEY_LOGI("+++\n");

	vkey_buf = devm_kzalloc(&pdev->dev, MAX_BUF_SIZE, GFP_KERNEL);
	if (!vkey_buf) {
		VKEY_LOGE("Failed to allocate vkey_buf memory\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct vkeys_platform_data), GFP_KERNEL);
		if (!pdata) {
			VKEY_LOGE("Failed to allocate pdata memory\n");
			return -ENOMEM;
		}

		ret = vkey_parse_dt(&pdev->dev, pdata);
		if (ret) {
			VKEY_LOGE("Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = pdev->dev.platform_data;

	if (!pdata || !pdata->name || !pdata->num_keys || !pdata->button) {
		VKEY_LOGE("pdata is invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < pdata->num_keys; i++) {
		c += snprintf(vkey_buf + c, MAX_BUF_SIZE - c,
				"%s:%d:%d:%d:%s:",
				VKEY_VER_CODE,
				(pdata->button + i)->keycode,
				(pdata->button + i)->center_x,
				(pdata->button + i)->center_y,
				(pdata->button + i)->range);
	}

	vkey_buf[c-1] = '\n';
	vkey_buf[c] = '\0';

	name = devm_kzalloc(&pdev->dev, sizeof(*name) * MAX_BUF_SIZE,
					GFP_KERNEL);
	if (!name) {
		VKEY_LOGE("Failed to allocate attr name memory\n");
		return -ENOMEM;
	}

	snprintf(name, MAX_BUF_SIZE,
				"virtualkeys.%s", pdata->name);
	vkey_obj_attr.attr.name = name;

	vkey_obj = kobject_create_and_add("board_properties", NULL);
	if (!vkey_obj) {
		VKEY_LOGE("unable to create kobject\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(vkey_obj, &vkey_grp);
	if (ret) {
		VKEY_LOGE("failed to create attributes\n");
		goto destroy_kobj;
	}

	VKEY_LOGI("---\n");
	return 0;

destroy_kobj:
	kobject_put(vkey_obj);

	return ret;
}

static int __devexit vkeys_remove(struct platform_device *pdev)
{
	sysfs_remove_group(vkey_obj, &vkey_grp);
	kobject_put(vkey_obj);

	return 0;
}

static struct of_device_id vkey_match_table[] = {
	{ .compatible = "qcom,gen-vkeys",},
	{ },
};

static struct platform_driver vkeys_driver = {
	.probe = vkeys_probe,
	.remove = __devexit_p(vkeys_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "gen_vkeys",
		.of_match_table = vkey_match_table,
	},
};

module_platform_driver(vkeys_driver);
MODULE_LICENSE("GPL v2");
