/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Internal interface */

#ifndef OPENHMDI_H
#define OPENHMDI_H

#include "openhmd.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define OHMD_MAX_DEVICES 16

#define OHMD_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define OHMD_MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))

#define OHMD_STRINGIFY(_what) #_what

typedef struct ohmd_driver ohmd_driver;

typedef struct
{
	char driver[OHMD_STR_SIZE];
	char vendor[OHMD_STR_SIZE];
	char product[OHMD_STR_SIZE];
	char path[OHMD_STR_SIZE];
	ohmd_driver* driver_ptr;
} ohmd_device_desc;

typedef struct {
	int num_devices;
	ohmd_device_desc devices[OHMD_MAX_DEVICES];
} ohmd_device_list;

struct ohmd_driver {
	void (*get_device_list)(ohmd_driver* driver, ohmd_device_list* list);
	ohmd_device* (*open_device)(ohmd_driver* driver, ohmd_device_desc* desc);
	void (*destroy)(ohmd_driver* driver);
	ohmd_context* ctx;
};

struct ohmd_device {
	struct {
		int hres;
		int vres;
		float hsize;
		float vsize;

		float lens_sep;
		float lens_vpos;

		float fov;
		float ratio;

		float idp;
		float zfar;
		float znear;
	} properties;

	int (*getf)(ohmd_device* device, ohmd_float_value type, float* out);
	void (*update)(ohmd_device* device);
	void (*close)(ohmd_device* device);
	ohmd_context* ctx;
};

struct ohmd_context {
	ohmd_driver* drivers[16];
	int num_drivers;

	ohmd_device_list list;

	ohmd_device* active_devices[256];
	int num_active_devices;

	char error_msg[OHMD_STR_SIZE];
};

// helper functions
void ohmd_set_default_device_properties(ohmd_device* device);

// drivers
ohmd_driver* ohmd_create_oculus_rift_drv(ohmd_context* ctx);

#include "log.h"
#include "platform.h"
#include "omath.h"
#include "fusion.h"

#endif
