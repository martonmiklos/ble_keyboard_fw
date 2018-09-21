/*
 * Elan I2C/SMBus Touchpad driver
 *
 * Copyright (c) 2013 ELAN Microelectronics Corp.
 *
 * Author: 林政維 (Duson Lin) <dusonlin@emc.com.tw>
 * Author: KT Liao <kt.liao@emc.com.tw>
 * Version: 1.6.3
 *
 * Based on cyapa driver:
 * copyright (c) 2011-2012 Cypress Semiconductor, Inc.
 * copyright (c) 2011-2012 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include "elan_i2c.h"

#define DRIVER_NAME		"elan_i2c"
#define ELAN_VENDOR_ID		0x04f3
#define ETP_MAX_PRESSURE	255
#define ETP_FWIDTH_REDUCE	90
#define ETP_FINGER_WIDTH	15
#define ETP_RETRY_COUNT		3

#define ETP_MAX_FINGERS		5
#define ETP_FINGER_DATA_LEN	5
#define ETP_REPORT_ID		0x5D
#define ETP_REPORT_ID_OFFSET	2
#define ETP_TOUCH_INFO_OFFSET	3
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_HOVER_INFO_OFFSET	30
#define ETP_MAX_REPORT_LEN	34

/* The main device structure */
struct elan_tp_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct regulator	*vcc;

	const struct elan_transport_ops *ops;

	/* for fw update */
	bool			in_fw_update;

	unsigned int		max_x;
	unsigned int		max_y;
	unsigned int		width_x;
	unsigned int		width_y;
	unsigned int		x_res;
	unsigned int		y_res;

    uint8_t			pattern;
    uint16_t			product_id;
    uint8_t			fw_version;
    uint8_t			sm_version;
    uint8_t			iap_version;
    uint16_t			fw_checksum;
	int			pressure_adjustment;
    uint8_t			mode;
    uint16_t			ic_type;
    uint16_t			fw_validpage_count;
    uint16_t			fw_signature_address;

	bool			irq_wake;

    uint8_t			min_baseline;
    uint8_t			max_baseline;
	bool			baseline_ready;
    uint8_t			clickpad;
};

struct i2c_client *to_i2c_client(struct device *dev)
{
    return dev->client;
}

struct elan_tp_data *i2c_get_clientdata(struct i2c_client *client)
{
    return (struct elan_tp_data*)(client->data);
}

static int elan_get_fwinfo(uint16_t ic_type, uint16_t *validpage_count,
               uint16_t *signature_address)
{
	switch (ic_type) {
	case 0x00:
	case 0x06:
	case 0x08:
		*validpage_count = 512;
		break;
	case 0x03:
	case 0x07:
	case 0x09:
	case 0x0A:
	case 0x0B:
	case 0x0C:
		*validpage_count = 768;
		break;
	case 0x0D:
		*validpage_count = 896;
		break;
	case 0x0E:
		*validpage_count = 640;
		break;
	case 0x10:
		*validpage_count = 1024;
		break;
	default:
		/* unknown ic type clear value */
		*validpage_count = 0;
		*signature_address = 0;
		return -ENXIO;
	}

	*signature_address =
		(*validpage_count * ETP_FW_PAGE_SIZE) - ETP_FW_SIGNATURE_SIZE;

	return 0;
}

static int elan_enable_power(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	do {
        error = elan_i2c_power_control(data->client, true);
		if (error >= 0)
			return 0;

		msleep(30);
	} while (--repeat > 0);

    NRF_LOG_ERROR("failed to enable power: %d\n", error);
	return error;
}

static int elan_disable_power(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	do {
        error = elan_i2c_power_control(data->client, false);
		if (!error) {
			if (error) {
                NRF_LOG_ERROR(
					"failed to disable regulator: %d\n",
					error);
				/* Attempt to power the chip back up */
                elan_i2c_power_control(data->client, true);
				break;
			}

			return 0;
		}

		msleep(30);
	} while (--repeat > 0);

    NRF_LOG_ERROR("failed to disable power: %d\n", error);
	return error;
}

static int elan_sleep(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	do {
        error = elan_i2c_sleep_control(data->client, true);
		if (!error)
			return 0;

		msleep(30);
	} while (--repeat > 0);

	return error;
}

static int elan_query_product(struct elan_tp_data *data)
{
	int error;

    error = elan_i2c_get_product_id(data->client, &data->product_id);
	if (error)
		return error;

    error = elan_i2c_get_sm_version(data->client, &data->ic_type,
					  &data->sm_version, &data->clickpad);
	if (error)
		return error;

	return 0;
}

static int elan_check_ASUS_special_fw(struct elan_tp_data *data)
{
	if (data->ic_type == 0x0E) {
		switch (data->product_id) {
		case 0x05 ... 0x07:
		case 0x09:
		case 0x13:
			return true;
		}
	} else if (data->ic_type == 0x08 && data->product_id == 0x26) {
		/* ASUS EeeBook X205TA */
		return true;
	}

	return false;
}

static int __elan_initialize(struct elan_tp_data *data)
{
	struct i2c_client *client = data->client;
	bool woken_up = false;
	int error;

    error = elan_i2c_initialize(client);
	if (error) {
        dev_err("device initialize failed: %d\n", error);
		return error;
	}

	error = elan_query_product(data);
	if (error)
		return error;

	/*
	 * Some ASUS devices were shipped with firmware that requires
	 * touchpads to be woken up first, before attempting to switch
	 * them into absolute reporting mode.
	 */
	if (elan_check_ASUS_special_fw(data)) {
        error = elan_i2c_sleep_control(client, false);
		if (error) {
            dev_err("failed to wake device up: %d\n", error);
			return error;
		}

		msleep(200);
		woken_up = true;
	}

	data->mode |= ETP_ENABLE_ABS;
    error = elan_i2c_set_mode(client, data->mode);
	if (error) {
        dev_err("failed to switch to absolute mode: %d\n", error);
		return error;
	}

	if (!woken_up) {
        error = elan_i2c_sleep_control(client, false);
		if (error) {
            dev_err("failed to wake device up: %d\n", error);
			return error;
		}
	}

	return 0;
}

static int elan_initialize(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	do {
		error = __elan_initialize(data);
		if (!error)
			return 0;

		msleep(30);
	} while (--repeat > 0);

	return error;
}

static int elan_query_device_info(struct elan_tp_data *data)
{
	int error;
    uint16_t ic_type;

    error = elan_i2c_get_version(data->client, false, &data->fw_version);
	if (error)
		return error;

    error = elan_i2c_get_checksum(data->client, false,
					&data->fw_checksum);
	if (error)
		return error;

    error = elan_i2c_get_version(data->client, true, &data->iap_version);
	if (error)
		return error;

    error = elan_i2c_get_pressure_adjustment(data->client,
						   &data->pressure_adjustment);
	if (error)
		return error;

    error = elan_i2c_get_pattern(data->client, &data->pattern);
	if (error)
		return error;

	if (data->pattern == 0x01)
		ic_type = data->ic_type;
	else
		ic_type = data->iap_version;

	error = elan_get_fwinfo(ic_type, &data->fw_validpage_count,
				&data->fw_signature_address);
	if (error)
		dev_warn(&data->client->dev,
			 "unexpected iap version %#04x (ic type: %#04x), firmware update will not work\n",
			 data->iap_version, data->ic_type);

	return 0;
}

static unsigned int elan_convert_resolution(uint8_t val)
{
	/*
	 * (value from firmware) * 10 + 790 = dpi
	 *
	 * We also have to convert dpi to dots/mm (*10/254 to avoid floating
	 * point).
	 */

	return ((int)(char)val * 10 + 790) * 10 / 254;
}

static int elan_query_device_parameters(struct elan_tp_data *data)
{
	unsigned int x_traces, y_traces;
    uint8_t hw_x_res, hw_y_res;
	int error;

    error = elan_i2c_get_max(data->client, &data->max_x, &data->max_y);
	if (error)
		return error;

    error = elan_i2c_get_num_traces(data->client, &x_traces, &y_traces);
	if (error)
		return error;

	data->width_x = data->max_x / x_traces;
	data->width_y = data->max_y / y_traces;

    error = elan_i2c_get_resolution(data->client, &hw_x_res, &hw_y_res);
	if (error)
		return error;

	data->x_res = elan_convert_resolution(hw_x_res);
	data->y_res = elan_convert_resolution(hw_y_res);

	return 0;
}

/*
 **********************************************************
 * IAP firmware updater related routines
 **********************************************************
 */
static int elan_write_fw_block(struct elan_tp_data *data,
                   const uint8_t *page, uint16_t checksum, int idx)
{
	int retry = ETP_RETRY_COUNT;
	int error;

	do {
        error = elan_i2c_write_fw_block(data->client,
						  page, checksum, idx);
		if (!error)
			return 0;

		dev_dbg(&data->client->dev,
			"IAP retrying page %d (error: %d)\n", idx, error);
	} while (--retry > 0);

	return error;
}

static int __elan_update_firmware(struct elan_tp_data *data,
				  const struct firmware *fw)
{
	struct i2c_client *client = data->client;
    struct device *dev = &client->dev;
	int i, j;
	int error;
    uint16_t iap_start_addr;
    uint16_t boot_page_count;
    uint16_t sw_checksum = 0, fw_checksum = 0;

    error = elan_i2c_prepare_fw_update(client);
	if (error)
		return error;

	iap_start_addr = get_unaligned_le16(&fw->data[ETP_IAP_START_ADDR * 2]);

	boot_page_count = (iap_start_addr * 2) / ETP_FW_PAGE_SIZE;
	for (i = boot_page_count; i < data->fw_validpage_count; i++) {
        uint16_t checksum = 0;
        const uint8_t *page = &fw->data[i * ETP_FW_PAGE_SIZE];

		for (j = 0; j < ETP_FW_PAGE_SIZE; j += 2)
			checksum += ((page[j + 1] << 8) | page[j]);

		error = elan_write_fw_block(data, page, checksum, i);
		if (error) {
            dev_err("write page %d fail: %d\n", i, error);
			return error;
		}

		sw_checksum += checksum;
	}
    // Wait WDT reset and power on reset
    msleep(600);

    error = elan_i2c_finish_fw_update(client);
	if (error)
		return error;

    error = elan_i2c_get_checksum(client, true, &fw_checksum);
	if (error)
		return error;

	if (sw_checksum != fw_checksum) {
        dev_err("checksum diff sw=[%04X], fw=[%04X]\n",
			sw_checksum, fw_checksum);
		return -EIO;
	}

	return 0;
}

static int elan_update_firmware(struct elan_tp_data *data,
				const struct firmware *fw)
{
	struct i2c_client *client = data->client;
	int retval;

	dev_dbg(&client->dev, "Starting firmware update....\n");

	data->in_fw_update = true;

	retval = __elan_update_firmware(data, fw);
	if (retval) {
        dev_err("firmware update failed: %d\n", retval);
        elan_i2c_iap_reset(client);
	} else {
        // Reinitialize TP after fw is updated
		elan_initialize(data);
		elan_query_device_info(data);
	}

	data->in_fw_update = false;

	return retval;
}


/*
 *******************************************************************
 * SYSFS attributes
 *******************************************************************
 */
static ssize_t elan_sysfs_read_fw_checksum(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "0x%04x\n", data->fw_checksum);
}

static ssize_t elan_sysfs_read_product_id(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);

	return sprintf(buf, ETP_PRODUCT_ID_FORMAT_STRING "\n",
		       data->product_id);
}

static ssize_t elan_sysfs_read_fw_ver(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d.0\n", data->fw_version);
}

static ssize_t elan_sysfs_read_sm_ver(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d.0\n", data->sm_version);
}

static ssize_t elan_sysfs_read_iap_ver(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d.0\n", data->iap_version);
}

static ssize_t elan_sysfs_update_fw(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{    
    struct elan_tp_data *data = i2c_get_clientdata(to_i2c_client(dev));
	const struct firmware *fw;
	char *fw_name;
	int error;
    const uint8_t *fw_signature;
    static const uint8_t signature[] = {0xAA, 0x55, 0xCC, 0x33, 0xFF, 0xFF};

	if (data->fw_validpage_count == 0)
		return -EINVAL;

	/* Look for a firmware with the product id appended. */
    sprintf(fw_name, ETP_FW_NAME, data->product_id);
	if (!fw_name) {
        dev_err("failed to allocate memory for firmware name\n");
		return -ENOMEM;
	}

	dev_info(dev, "requesting fw '%s'\n", fw_name);
	if (error) {
        dev_err("failed to request firmware: %d\n", error);
		return error;
	}

	/* Firmware file must match signature data */
	fw_signature = &fw->data[data->fw_signature_address];
	if (memcmp(fw_signature, signature, sizeof(signature)) != 0) {
        dev_err("signature mismatch (expected %*ph, got %*ph)\n",
			(int)sizeof(signature), signature,
			(int)sizeof(signature), fw_signature);
		error = -EBADF;
		goto out_release_fw;
	}

	if (error)
		goto out_release_fw;

	error = elan_update_firmware(data, fw);

out_release_fw:
	release_firmware(fw);
	return error ?: count;
}

static ssize_t calibrate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);
	int tries = 20;
	int retval;
	int error;
    uint8_t val[3];

	data->mode |= ETP_ENABLE_CALIBRATE;
    retval = elan_i2c_set_mode(client, data->mode);
	if (retval) {
        dev_err("failed to enable calibration mode: %d\n",
			retval);
		goto out;
	}

    retval = elan_i2c_calibrate(client);
	if (retval) {
        dev_err("failed to start calibration: %d\n",
			retval);
		goto out_disable_calibrate;
	}

	val[0] = 0xff;
	do {
		/* Wait 250ms before checking if calibration has completed. */
		msleep(250);

        retval = elan_i2c_calibrate_result(client, val);
        if (retval) {
            dev_err("failed to check calibration result: %d\n",
				retval);
        } else if (val[0] == 0) {
			break; /* calibration done */
        }
	} while (--tries);

	if (tries == 0) {
        dev_err("failed to calibrate. Timeout.\n");
		retval = -ETIMEDOUT;
	}

out_disable_calibrate:
	data->mode &= ~ETP_ENABLE_CALIBRATE;
    error = elan_i2c_set_mode(data->client, data->mode);
	if (error) {
        dev_err("failed to disable calibration mode: %d\n",
			error);
		if (!retval)
			retval = error;
	}
out:
	return retval ?: count;
}

static ssize_t elan_sysfs_read_mode(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);
	int error;
	enum tp_mode mode;

    error = elan_i2c_iap_get_mode(data->client, &mode);

	if (error)
		return error;

	return sprintf(buf, "%d\n", (int)mode);
}

static ssize_t acquire_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);
	int error;
	int retval;

	data->baseline_ready = false;

	data->mode |= ETP_ENABLE_CALIBRATE;
    retval = elan_i2c_set_mode(data->client, data->mode);
	if (retval) {
        dev_err("Failed to enable calibration mode to get baseline: %d\n",
			retval);
		goto out;
	}

	msleep(250);

    retval = elan_i2c_get_baseline_data(data->client, true,
					      &data->max_baseline);
	if (retval) {
        dev_err("Failed to read max baseline form device: %d\n",
			retval);
		goto out_disable_calibrate;
	}

    retval = elan_i2c_get_baseline_data(data->client, false,
					      &data->min_baseline);
	if (retval) {
        dev_err("Failed to read min baseline form device: %d\n",
			retval);
		goto out_disable_calibrate;
	}

	data->baseline_ready = true;

out_disable_calibrate:
	data->mode &= ~ETP_ENABLE_CALIBRATE;
    error = elan_i2c_set_mode(data->client, data->mode);
	if (error) {
        dev_err("Failed to disable calibration mode after acquiring baseline: %d\n",
			error);
		if (!retval)
			retval = error;
	}
out:
	return retval ?: count;
}

static ssize_t min_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);
	int retval;

	if (!data->baseline_ready) {
		retval = -ENODATA;
		goto out;
	}

	retval = snprintf(buf, PAGE_SIZE, "%d", data->min_baseline);

out:
	return retval;
}

static ssize_t max_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_tp_data *data = i2c_get_clientdata(client);
	int retval;
	if (!data->baseline_ready) {
		retval = -ENODATA;
		goto out;
	}

	retval = snprintf(buf, PAGE_SIZE, "%d", data->max_baseline);

out:
	return retval;
}

/*
 ******************************************************************
 * Elan isr functions
 ******************************************************************
 */
static void elan_report_contact(struct elan_tp_data *data,
				int contact_num, bool contact_valid,
                uint8_t *finger_data)
{
	struct input_dev *input = data->input;
	unsigned int pos_x, pos_y;
	unsigned int pressure, mk_x, mk_y;
	unsigned int area_x, area_y, major, minor;
	unsigned int scaled_pressure;

	if (contact_valid) {
		pos_x = ((finger_data[0] & 0xf0) << 4) |
						finger_data[1];
		pos_y = ((finger_data[0] & 0x0f) << 8) |
						finger_data[2];
		mk_x = (finger_data[3] & 0x0f);
		mk_y = (finger_data[3] >> 4);
		pressure = finger_data[4];

		if (pos_x > data->max_x || pos_y > data->max_y) {
			dev_dbg(input->dev.parent,
				"[%d] x=%d y=%d over max (%d, %d)",
				contact_num, pos_x, pos_y,
				data->max_x, data->max_y);
			return;
		}

		/*
		 * To avoid treating large finger as palm, let's reduce the
		 * width x and y per trace.
		 */
		area_x = mk_x * (data->width_x - ETP_FWIDTH_REDUCE);
		area_y = mk_y * (data->width_y - ETP_FWIDTH_REDUCE);

		major = max(area_x, area_y);
		minor = min(area_x, area_y);

		scaled_pressure = pressure + data->pressure_adjustment;

		if (scaled_pressure > ETP_MAX_PRESSURE)
			scaled_pressure = ETP_MAX_PRESSURE;

        /*input_mt_slot(input, contact_num);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_POSITION_X, pos_x);
		input_report_abs(input, ABS_MT_POSITION_Y, data->max_y - pos_y);
		input_report_abs(input, ABS_MT_PRESSURE, scaled_pressure);
		input_report_abs(input, ABS_TOOL_WIDTH, mk_x);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, major);
        input_report_abs(input, ABS_MT_TOUCH_MINOR, minor);*/
	} else {
        /*input_mt_slot(input, contact_num);
        input_mt_report_slot_state(input, MT_TOOL_FINGER, false);*/
	}
}

static void elan_report_absolute(struct elan_tp_data *data, uint8_t *packet)
{
	struct input_dev *input = data->input;
    uint8_t *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	int i;
    uint8_t tp_info = packet[ETP_TOUCH_INFO_OFFSET];
    uint8_t hover_info = packet[ETP_HOVER_INFO_OFFSET];
	bool contact_valid, hover_event;

	hover_event = hover_info & 0x40;
	for (i = 0; i < ETP_MAX_FINGERS; i++) {
		contact_valid = tp_info & (1U << (3 + i));
		elan_report_contact(data, i, contact_valid, finger_data);

		if (contact_valid)
			finger_data += ETP_FINGER_DATA_LEN;
	}

    /*input_report_key(input, BTN_LEFT, tp_info & 0x01);
	input_report_key(input, BTN_RIGHT, tp_info & 0x02);
	input_report_abs(input, ABS_DISTANCE, hover_event != 0);
	input_mt_report_pointer_emulation(input, true);
    input_sync(input);*/
}

static int elan_isr(int irq, void *dev_id)
{
	struct elan_tp_data *data = dev_id;
	struct device *dev = &data->client->dev;
	int error;
    uint8_t report[ETP_MAX_REPORT_LEN];

	/*
	 * When device is connected to i2c bus, when all IAP page writes
	 * complete, the driver will receive interrupt and must read
	 * 0000 to confirm that IAP is finished.
	*/
	if (data->in_fw_update) {
		goto out;
	}

    error = elan_i2c_get_report(data->client, report);
	if (error)
		goto out;

    if (report[ETP_REPORT_ID_OFFSET] != ETP_REPORT_ID) {
        dev_err("invalid report id data (%x)\n",
			report[ETP_REPORT_ID_OFFSET]);
    } else {
		elan_report_absolute(data, report);
    }

out:
    return 0;
}

/*
 ******************************************************************
 * Elan initialization functions
 ******************************************************************
 */
static int elan_setup_input_device(struct elan_tp_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input;
	unsigned int max_width = max(data->width_x, data->width_y);
	unsigned int min_width = min(data->width_x, data->width_y);
	int error;

    /*error = input_mt_init_slots(input, ETP_MAX_FINGERS,
                    INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
    if (error) {
        dev_err("failed to initialize MT slots: %d\n", error);
        return error;
    }*/
	/* Set up ST parameters */
    /*input_set_abs_params(input, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_X, data->x_res);
	input_abs_set_res(input, ABS_Y, data->y_res);
	input_set_abs_params(input, ABS_PRESSURE, 0, ETP_MAX_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, ETP_FINGER_WIDTH, 0, 0);
    input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);*/

	/* And MT parameters */
    /*input_set_abs_params(input, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, data->x_res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, data->y_res);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			     ETP_MAX_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
			     ETP_FINGER_WIDTH * max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0,
                 ETP_FINGER_WIDTH * min_width, 0, 0);*/

	data->input = input;

	return 0;
}


static int elan_probe(struct i2c_client *client,
		      const struct i2c_device_id *dev_id)
{
	const struct elan_transport_ops *transport_ops;
	struct device *dev = &client->dev;
	struct elan_tp_data *data;
	unsigned long irqflags;
	int error;

	i2c_set_clientdata(client, data);

	data->ops = transport_ops;
	data->client = client;

    /* Make sure there is something at this address */
	error = i2c_smbus_read_byte(client);
	if (error < 0) {
		dev_dbg(&client->dev, "nothing at this address: %d\n", error);
		return -ENXIO;
	}

	/* Initialize the touchpad. */
	error = elan_initialize(data);
	if (error)
		return error;

	error = elan_query_device_info(data);
	if (error)
		return error;

	error = elan_query_device_parameters(data);
	if (error)
		return error;

	dev_info(dev,
		 "Elan Touchpad: Module ID: 0x%04x, Firmware: 0x%04x, Sample: 0x%04x, IAP: 0x%04x\n",
		 data->product_id,
		 data->fw_version,
		 data->sm_version,
		 data->iap_version);

	dev_dbg(dev,
		"Elan Touchpad Extra Information:\n"
		"    Max ABS X,Y:   %d,%d\n"
		"    Width X,Y:   %d,%d\n"
		"    Resolution X,Y:   %d,%d (dots/mm)\n"
		"    ic type: 0x%x\n"
		"    info pattern: 0x%x\n",
		data->max_x, data->max_y,
		data->width_x, data->width_y,
		data->x_res, data->y_res,
		data->ic_type, data->pattern);

	/* Set up input device properties based on queried parameters. */
	error = elan_setup_input_device(data);
	if (error)
		return error;
	return 0;
}
