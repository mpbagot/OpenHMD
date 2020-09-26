// Copyright 2018, Philipp Zabel.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Sony PlayStation Move controller driver */

#define FEATURE_BUFFER_SIZE	49

#define TICK_LEN		1e-7 // 0.1 µs ticks

#include "psvr.h"

extern "C" {

#include <string.h>
#include <wchar.h>
#include <hidapi.h>
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>

#include "psmove.h"

typedef struct {
	ohmd_device base;
	ohmd_device_flags device_flags;

	PSMove* move_handle;
	fusion sensor_fusion;
	vec3f raw_accel;
	vec3f raw_gyro;
	quatf orient, trackpad_init_orient;
	double last_set_led;
	psmove_packet sensor;
} controller_priv;

static void quat_to_euler(quatf q, float *roll, float *pitch, float* yaw) {
    // roll (x-axis rotation)
    if (roll != NULL) {
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        *roll = std::atan2(sinr_cosp, cosr_cosp);
    }

    // pitch (y-axis rotation)
    if (pitch != NULL) {
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            *pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            *pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    if (yaw != NULL) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        *yaw = std::atan2(siny_cosp, cosy_cosp);
    }
}

static void psmove_update_device(ohmd_device* device)
{
	controller_priv* priv = (controller_priv*)device;
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int size = 0;

	// Keep tracking sphere lit up
	double t = ohmd_get_tick();
	if(t - priv->last_set_led >= 5.0){
		if (priv->device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER)
			psmove_set_leds(priv->move_handle, 0xff, 0x00, 0x00);
		else
			psmove_set_leds(priv->move_handle, 0x00, 0x00, 0xff);

		psmove_update_leds(priv->move_handle);

		priv->last_set_led = t;
	}

	while (psmove_poll(priv->move_handle)) {
		psmove_get_orientation(priv->move_handle, &priv->orient.w, &priv->orient.x, &priv->orient.y, &priv->orient.z);

		// set buttons and trigger
		priv->sensor.buttons = psmove_get_buttons(priv->move_handle);
		priv->sensor.trigger[0] = psmove_get_trigger(priv->move_handle);

		// TODO Calibration of forward with start and orientation reset with select
		// Should this instead be done in the main driver code?

		// Set trackpad x and y if move is held
		unsigned int pressed, released;
		psmove_get_button_events(priv->move_handle, &pressed, &released);
		// Set the trackpad initial orientation
		if (pressed & Btn_MOVE) {
				// Memcpy the current orientation to the trackpad variable
				memcpy(&priv->trackpad_init_orient, &priv->orient, sizeof(quatf));
		}

		// Clear the trackpad intitial orientation
		if (released & Btn_MOVE) {
				// Just write 0s over the quaternion struct, since that's easier than 0-ing the values individually.
				memset(&priv->trackpad_init_orient, 0, sizeof(quatf));

				// Also, reset trackpad position
				priv->sensor.trackpad[0] = 0.0f;
				priv->sensor.trackpad[1] = 0.0f;
		}

		// Once the initial orientation is set, update the trackpad if the move button is held
		if (priv->sensor.buttons & Btn_MOVE) {
				// Calculate the pitch and roll difference
				float init_roll, init_pitch, current_roll, current_pitch, pitch_diff, roll_diff;
				quat_to_euler(priv->orient, &init_pitch, NULL, &init_roll);
				quat_to_euler(priv->trackpad_init_orient, &current_pitch, NULL, &current_roll);

				// TODO Roll seems to be world relative, which is not what I want

				pitch_diff = current_pitch - init_pitch;
				roll_diff = current_roll - init_roll;

				// Convert rotation diffs to -1 to 1 x-y trackpad positions
				float x = (4 * roll_diff) / M_PI;
				x = x < -1.0f ? -1.0f : x;
				x = x > 1.0f ? 1.0f : x;
				float y = (4 * pitch_diff) / M_PI;
				y = y < -1.0f ? -1.0f : y;
				y = y > 1.0f ? 1.0f : y;

				priv->sensor.trackpad[0] = x;
				priv->sensor.trackpad[1] = y;
		}
	}

	if (g_nPlayer) {
		if (!g_UserGenerator.GetSkeletonCap().IsCalibrated(g_nPlayer))
		{
			printf("not calibrated!\n");
			return;
		}
		if (!g_UserGenerator.GetSkeletonCap().IsTracking(g_nPlayer))
		{
			printf("not tracked!\n");
			return;
		}

		// Get the head position
		XnSkeletonJoint jointName = priv->device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER ? XN_SKEL_LEFT_HAND : XN_SKEL_RIGHT_HAND;
		XnSkeletonJointPosition handJoint;
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(g_nPlayer, jointName, handJoint);

		// Implement smoothing based on confidence value

		// current z - calibration point z
		// current x - calibration point x
		// calibration point y = 180 + (current y - calibration point y)
		// distance unit is meters for openhmd. OpenNI returns cm, so just divide by 100
		float conf = handJoint.fConfidence;

		// TODO if confidence is < 0.33. Try to use accelerometer to estimate position

		// Rolling average scale the position
		priv->base.position.x *= 1 - conf;
		priv->base.position.y *= 1 - conf;
		priv->base.position.z *= 1 - conf;

		priv->base.position.x += conf * ((handJoint.position.X - calib_point.x) / 1000.0f);
		priv->base.position.y += conf * (1.8f + (handJoint.position.Y - calib_point.y) / 1000.0f);
		priv->base.position.z += conf * ((handJoint.position.Z - calib_point.z) / 1000.0f);

		printf("update_device: X: %f, Y: %f, Z: %f\n", priv->base.position.x, priv->base.position.y, priv->base.position.z);
	}

	if(size < 0){
		LOGE("error reading from device");
	}
}

static int psmove_getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	controller_priv* priv = (controller_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT:
		*(quatf*)out = priv->orient;
		break;

	case OHMD_POSITION_VECTOR:
		// Position from priv
		*(vec3f*)out = priv->base.position;
		break;

#if 0
	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;
#endif

	case OHMD_CONTROLS_STATE:
		out[0] = !!(priv->sensor.buttons & Btn_SELECT);
		out[1] = !!(priv->sensor.buttons & Btn_START);
		out[2] = !!(priv->sensor.buttons & Btn_TRIANGLE);
		out[3] = !!(priv->sensor.buttons & Btn_CIRCLE);
		out[4] = !!(priv->sensor.buttons & Btn_CROSS);
		out[5] = !!(priv->sensor.buttons & Btn_SQUARE);
		out[6] = !!(priv->sensor.buttons & Btn_PS);
		out[7] = !!(priv->sensor.buttons & Btn_MOVE);
		out[8] = priv->sensor.trigger[0] / 255.0;
		out[9] = priv->sensor.trigger[0] > 205; // Trigger click
		out[10] = !!(priv->sensor.buttons & Btn_MOVE); // TODO Trackpad click
		out[11] = priv->sensor.trackpad[0]; // Trackpad X
		out[12] = priv->sensor.trackpad[1]; // Trackpad Y
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to controller getf (%ud)", type);
		return -1;
	}

	return 0;
}

static void psmove_close_device(ohmd_device* device)
{
	controller_priv* priv = (controller_priv*)device;

	LOGD("closing Playstation Move Controller device");

	// hid_close(priv->controller_imu);
	psmove_disconnect(priv->move_handle);

	free(device);
}

ohmd_device* open_psmove_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	controller_priv* priv = (controller_priv*)ohmd_alloc(driver->ctx, sizeof(controller_priv));

	if(!priv)
		return NULL;

	priv->base.ctx = driver->ctx;
	priv->device_flags = desc->device_flags;

	int idx = atoi(desc->path);

	// Open the controller device
	priv->move_handle = psmove_connect_by_id(idx);
	if (priv->move_handle == NULL) {
		goto cleanup;
	}

	// read stored calibration data
	psmove_enable_orientation(priv->move_handle, PSMove_True);
	if (psmove_has_orientation(priv->move_handle) == 0) {
		goto cleanup;
	}

	psmove_set_orientation_fusion_type(priv->move_handle, OrientationFusion_ComplementaryMARG);

	// Light up the tracking sphere
	if (priv->device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER)
		psmove_set_leds(priv->move_handle, 0xff, 0x00, 0x00);
	else
		psmove_set_leds(priv->move_handle, 0x00, 0x00, 0xff);
	psmove_update_leds(priv->move_handle);
	priv->last_set_led = ohmd_get_tick();

	// Set default device properties
	ohmd_set_default_device_properties(&priv->base.properties);

	// Set device properties
	priv->base.properties.control_count = 13;
	priv->base.properties.controls_hints[0] = OHMD_GENERIC; // Select
	priv->base.properties.controls_hints[1] = OHMD_GENERIC; // Start
	priv->base.properties.controls_hints[2] = OHMD_BUTTON_Y; // Triangle
	priv->base.properties.controls_hints[3] = OHMD_BUTTON_B; // Circle
	priv->base.properties.controls_hints[4] = OHMD_BUTTON_A; // Cross
	priv->base.properties.controls_hints[5] = OHMD_BUTTON_X; // Square
	priv->base.properties.controls_hints[6] = OHMD_GENERIC; // PlayStation
	priv->base.properties.controls_hints[7] = OHMD_GENERIC; // Move
	priv->base.properties.controls_hints[8] = OHMD_TRIGGER; // Trigger
	priv->base.properties.controls_hints[9] = OHMD_GENERIC; // Trigger click
	priv->base.properties.controls_hints[9] = OHMD_GENERIC; // Trackpad click
	priv->base.properties.controls_hints[11] = OHMD_ANALOG_X; // X Trackpad
	priv->base.properties.controls_hints[12] = OHMD_ANALOG_Y; // Y Trackpad

	priv->base.properties.controls_types[0] = OHMD_DIGITAL;
	priv->base.properties.controls_types[1] = OHMD_DIGITAL;
	priv->base.properties.controls_types[2] = OHMD_DIGITAL;
	priv->base.properties.controls_types[3] = OHMD_DIGITAL;
	priv->base.properties.controls_types[4] = OHMD_DIGITAL;
	priv->base.properties.controls_types[5] = OHMD_DIGITAL;
	priv->base.properties.controls_types[6] = OHMD_DIGITAL;
	priv->base.properties.controls_types[7] = OHMD_DIGITAL;
	priv->base.properties.controls_types[8] = OHMD_ANALOG;
	priv->base.properties.controls_types[9] = OHMD_DIGITAL;
	priv->base.properties.controls_types[10] = OHMD_DIGITAL;
	priv->base.properties.controls_types[11] = OHMD_ANALOG;
	priv->base.properties.controls_types[12] = OHMD_ANALOG;

	// set up device callbacks
	priv->base.update = psmove_update_device;
	priv->base.close = psmove_close_device;
	priv->base.getf = psmove_getf;

	ofusion_init(&priv->sensor_fusion);

	return (ohmd_device*)priv;

cleanup:
	if(priv)
		free(priv);

	return NULL;
}
}
