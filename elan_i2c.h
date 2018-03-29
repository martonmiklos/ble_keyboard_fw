/*
 * Elan I2C/SMBus Touchpad driver
 *
 * Copyright (c) 2013 ELAN Microelectronics Corp.
 *
 * Author: 林政維 (Duson Lin) <dusonlin@emc.com.tw>
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

#ifndef _ELAN_I2C_H
#define _ELAN_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#include "app_error.h"

#define msleep(x) {nrf_delay_ms(x);}
#define dev_err(dev, ...) {NRF_LOG_ERROR(__VA_ARGS__)}
#define dev_warn(dev, ...) {NRF_LOG_WARNING(__VA_ARGS__)}
#define dev_dbg(dev, ...) {NRF_LOG_DEBUG(__VA_ARGS__)}
#define dev_info(dev, ...) {NRF_LOG_INFO(__VA_ARGS__)}

typedef uint32_t umode_t;
typedef uint8_t u8;
typedef uint8_t __u8;
typedef uint16_t u16;
typedef uint16_t __u16;
typedef uint16_t __le16;
typedef __u16 __be16;
#define I2C_NAME_SIZE 64

#define PAGE_SIZE 64

struct firmware {
    size_t size;
    const uint8_t *data;

    /* firmware loader private fields */
    void *priv;
};


struct i2c_client;

struct device {
    struct i2c_client *client;
};

struct i2c_msg {
    __u16 addr;	/* slave address			*/
    __u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
    __u16 len;		/* msg length				*/
    __u8 *buf;		/* pointer to msg data			*/
};

struct i2c_client {
    unsigned short flags;		/* div., see below		*/
    unsigned short addr;		/* chip address - NOTE: 7bit	*/
                    /* addresses are stored in the	*/
                    /* _LOWER_ 7 bits		*/
    char name[I2C_NAME_SIZE];
    struct i2c_adapter *adapter;	/* the adapter we sit on	*/
    struct i2c_driver *driver;	/* and our access routines	*/
    struct device dev;		/* the device structure		*/
    int irq;			/* irq issued by device		*/
    void *data;
};
struct attribute {
    const char		*name;
    umode_t			mode;
};


/* interface for exporting device attributes */
struct device_attribute {
    struct attribute	attr;
    ssize_t (*show)(struct device *dev, struct device_attribute *attr,
            char *buf);
    ssize_t (*store)(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count);
};

static inline uint16_t get_unaligned_le16(const uint8_t *p)
{
    return p[0] | p[1] << 8;
}


#define ETP_ENABLE_ABS		0x0001
#define ETP_ENABLE_CALIBRATE	0x0002
#define ETP_DISABLE_CALIBRATE	0x0000
#define ETP_DISABLE_POWER	0x0001
#define ETP_PRESSURE_OFFSET	25

/* IAP Firmware handling */
#define ETP_PRODUCT_ID_FORMAT_STRING	"%d.0"
#define ETP_FW_NAME		"elan_i2c_" ETP_PRODUCT_ID_FORMAT_STRING ".bin"
#define ETP_IAP_START_ADDR	0x0083
#define ETP_FW_IAP_PAGE_ERR	(1 << 5)
#define ETP_FW_IAP_INTF_ERR	(1 << 4)
#define ETP_FW_PAGE_SIZE	64
#define ETP_FW_SIGNATURE_SIZE	6

struct completion;

enum tp_mode {
	IAP_MODE = 1,
	MAIN_MODE
};

int elan_i2c_initialize(struct i2c_client *client);
int elan_i2c_sleep_control(struct i2c_client *client, bool sleep);
int elan_i2c_power_control(struct i2c_client *client, bool enable);
int elan_i2c_set_mode(struct i2c_client *client, u8 mode);

int elan_i2c_calibrate(struct i2c_client *client);
int elan_i2c_calibrate_result(struct i2c_client *client, u8 *val);

int elan_i2c_get_baseline_data(struct i2c_client *client,
                      bool max_baseline, u8 *value);

int elan_i2c_get_version(struct i2c_client *client,
                bool iap, u8 *version);
int elan_i2c_get_sm_version(struct i2c_client *client,
                   u16 *ic_type, u8 *version,
                   u8 *clickpad);
int elan_i2c_get_product_id(struct i2c_client *client, u16 *id);
int elan_i2c_get_checksum(struct i2c_client *client,
                 bool iap, u16 *csum);
int elan_i2c_get_pressure_adjustment(struct i2c_client *client,
                        int *adjustment);

int elan_i2c_get_max(struct i2c_client *client,
                unsigned int *max_x, unsigned int *max_y);
int elan_i2c_get_resolution(struct i2c_client *client,
                   u8 *hw_res_x, u8 *hw_res_y);
int elan_i2c_get_num_traces(struct i2c_client *client,
                   unsigned int *x_traces,
                   unsigned int *y_traces);

int elan_i2c_iap_get_mode(struct i2c_client *client, enum tp_mode *mode);
int elan_i2c_iap_reset(struct i2c_client *client);

int elan_i2c_get_pattern(struct i2c_client *client, u8 *pattern);

int elan_i2c_get_report(struct i2c_client *client, u8 *report);


#endif /* _ELAN_I2C_H */
