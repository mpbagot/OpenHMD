// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2016, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Sony PSVR Driver */


#define FEATURE_BUFFER_SIZE 256

#define TICK_LEN (1.0f / 1000000.0f) // 1 MHz ticks

#include "psvr.h"

xn::Context g_Context;
xn::ScriptNode g_ScriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Recorder* g_pRecorder;

vec3f calib_point = {{0.0f, 0.0f, 0.0f}};

XnUserID g_nPlayer = 0;
XnBool g_bCalibrated = FALSE;

extern "C" {
#include <string.h>
#include <wchar.h>
#include <hidapi.h>
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>

#include "psmove.h"

#define SAMPLE_XML_PATH "/home/mitchell/psvr.xml"

typedef struct {
	ohmd_device base;

	hid_device* hmd_handle;
	hid_device* hmd_control;
	fusion sensor_fusion;
	vec3f raw_accel, raw_gyro;
	uint8_t last_seq;
	uint8_t buttons;
	psvr_sensor_packet sensor;

} psvr_priv;

XnBool AssignPlayer(XnUserID user)
{
	if (g_nPlayer != 0)
		return FALSE;

	XnPoint3D com;
	g_UserGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return FALSE;

	printf("Matching for existing calibration\n");
	g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	g_UserGenerator.GetSkeletonCap().StartTracking(user);
	g_nPlayer = user;
	return TRUE;

}
void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	if (!g_bCalibrated) // check on player0 is enough
	{
		printf("Look for pose\n");
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
		return;
	}

	AssignPlayer(user);
}

void FindPlayer()
{
	if (g_nPlayer != 0)
	{
		return;
	}
	XnUserID aUsers[5];
	XnUInt16 nUsers = 5;
	g_UserGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}

void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("Lost user %d\n", user);
	if (g_nPlayer == user)
	{
		g_nPlayer = 0;
		FindPlayer();
	}
}
void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt)
{
	printf("Found pose \"%s\" for user %d\n", strPose, user);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt)
{
	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!g_bCalibrated)
		{
			g_UserGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			g_nPlayer = user;
			g_UserGenerator.GetSkeletonCap().StartTracking(user);
			g_bCalibrated = TRUE;

			// Save the head calibration location
			XnSkeletonJointPosition headJoint;
			g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(g_nPlayer, XN_SKEL_HEAD, headJoint);
			//
			calib_point.x = headJoint.position.X;
			calib_point.y = headJoint.position.Y;
			calib_point.z = headJoint.position.Z;
		}

		XnUserID aUsers[5];
		XnUInt16 nUsers = 5;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
}

void accel_from_psvr_vec(const int16_t* smp, vec3f* out_vec)
{
	out_vec->x = (float)smp[1] *  (9.81 / 16384);
	out_vec->y = (float)smp[0] *  (9.81 / 16384);
	out_vec->z = (float)smp[2] * -(9.81 / 16384);
}

void gyro_from_psvr_vec(const int16_t* smp, vec3f* out_vec)
{
	out_vec->x = (float)smp[1] * 0.00105f;
	out_vec->y = (float)smp[0] * 0.00105f;
	out_vec->z = (float)smp[2] * 0.00105f * -1.0f;
}


static uint32_t calc_delta_and_handle_rollover(uint32_t next, uint32_t last)
{
	uint32_t tick_delta = next - last;

	// The 24-bit tick counter has rolled over,
	// adjust the "negative" value to be positive.
	if (tick_delta > 0xffffff) {
		tick_delta += 0x1000000;
	}

	return tick_delta;
}

static void handle_tracker_sensor_msg(psvr_priv* priv, unsigned char* buffer, int size)
{
	uint32_t last_sample_tick = priv->sensor.samples[1].tick;

	if(!psvr_decode_sensor_packet(&priv->sensor, buffer, size)){
		LOGE("couldn't decode tracker sensor message");
	}

	psvr_sensor_packet* s = &priv->sensor;

	uint32_t tick_delta = 500;

	// Startup correction, ignore last_sample_tick if zero.
	if (last_sample_tick > 0) {
		tick_delta = calc_delta_and_handle_rollover(
			s->samples[0].tick, last_sample_tick);

		// The PSVR device can buffer sensor data from previous
		// sessions which we can get at the start of new sessions.
		// @todo Maybe just skip the first 10 sensor packets?
		// @todo Maybe reset sensor fusion?
		if (tick_delta < 475 || tick_delta > 525) {
			LOGD("tick_delta = %u", tick_delta);
			tick_delta = 500;
		}
	}

	vec3f mag = {{0.0f, 0.0f, 0.0f}};

	for (int i = 0; i < 2; i++) {
		float dt = tick_delta * TICK_LEN;
		accel_from_psvr_vec(s->samples[i].accel, &priv->raw_accel);
		gyro_from_psvr_vec(s->samples[i].gyro, &priv->raw_gyro);

		ofusion_update(&priv->sensor_fusion, dt, &priv->raw_gyro, &priv->raw_accel, &mag);

		if (i == 0) {
			tick_delta = calc_delta_and_handle_rollover(
				s->samples[1].tick, s->samples[0].tick);
		}
	}

	priv->buttons = s->buttons;
}

static void teardown(psvr_priv* priv)
{
	if (priv->hmd_handle != NULL) {
		hid_close(priv->hmd_handle);
		priv->hmd_handle = NULL;
	}

	if (priv->hmd_control != NULL) {
		hid_close(priv->hmd_control);
		priv->hmd_control = NULL;
	}
}

static void update_device(ohmd_device* device)
{
	psvr_priv* priv = (psvr_priv*)device;

	int size = 0;
	unsigned char buffer[FEATURE_BUFFER_SIZE];

	while(true){
		int size = hid_read(priv->hmd_handle, buffer, FEATURE_BUFFER_SIZE);
		if(size < 0){
			LOGE("error reading from device");
			return;
		} else if(size == 0) {
			break; // No more messages, return.
		}

		handle_tracker_sensor_msg(priv, buffer, size);
	}

	// After reading from device, update position from Kinect
	// Read next available data
	g_Context.WaitAnyUpdateAll();//WaitOneUpdateAll(g_DepthGenerator);

	// Process the data
	if (g_nPlayer != 0)
	{
		XnPoint3D com;
		g_UserGenerator.GetCoM(g_nPlayer, com);
		if (com.Z == 0)
		{
			g_nPlayer = 0;
		}
	}

	// Find a player to track if there's no currently tracked player (but only if a play has previously been tracked)
	if (g_nPlayer == 0 && g_bCalibrated) {
			FindPlayer();
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
		XnSkeletonJointPosition headJoint;
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(g_nPlayer, XN_SKEL_HEAD, headJoint);

		// Implement smoothing based on confidence value

		// current z - calibration point z
		// current x - calibration point x
		// calibration point y = 180 + (current y - calibration point y)
		// distance unit is meters for openhmd. OpenNI returns cm, so just divide by 100
		float conf = headJoint.fConfidence;

		// TODO if confidence is < 0.33. Try to use accelerometer to estimate position

		// Rolling average scale the position
		priv->base.position.x *= 1 - conf;
		priv->base.position.y *= 1 - conf;
		priv->base.position.z *= 1 - conf;

		priv->base.position.x += conf * ((headJoint.position.X - calib_point.x) / 1000.0f);
		priv->base.position.y += conf * (1.8f + (headJoint.position.Y - calib_point.y) / 1000.0f);
		priv->base.position.z += conf * ((headJoint.position.Z - calib_point.z) / 1000.0f);
	}

	if(size < 0){
		LOGE("error reading from device");
	}
}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	psvr_priv* priv = (psvr_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT:
		*(quatf*)out = priv->sensor_fusion.orient;
		break;

	case OHMD_POSITION_VECTOR:
		*(vec3f*)out = priv->base.position;
		break;

	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;

	case OHMD_CONTROLS_STATE:
		out[0] = (priv->buttons & PSVR_BUTTON_VOLUME_PLUS) != 0;
		out[1] = (priv->buttons & PSVR_BUTTON_VOLUME_MINUS) != 0;
		out[2] = (priv->buttons & PSVR_BUTTON_MIC_MUTE) != 0;
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	psvr_priv* priv = (psvr_priv*)device;

	// set cinematic mode for the hmd
	hid_write(priv->hmd_control, psvr_cinematicmode_on, sizeof(psvr_cinematicmode_on));

	LOGD("Closing Sony PSVR device.");

	teardown(priv);

	free(device);
}

static hid_device* open_device_idx(int manufacturer, int product, int iface, int device_index)
{
	struct hid_device_info* devs = hid_enumerate(manufacturer, product);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	hid_device* ret = NULL;

	while (cur_dev) {
		LOGI("%04x:%04x %s", manufacturer, product, cur_dev->path);

		if (cur_dev->interface_number == iface) {
			if(idx == device_index){
				LOGI("\topening '%s'", cur_dev->path);
				ret = hid_open_path(cur_dev->path);
				break;
			}

			idx++;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);

	return ret;
}

static ohmd_device* open_hmd_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	psvr_priv* priv = (psvr_priv*)ohmd_alloc(driver->ctx, sizeof(psvr_priv));

	XnStatus rc = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	char errorStr[1024];

	if(!priv)
		return NULL;

	priv->base.ctx = driver->ctx;

	int idx = atoi(desc->path);

	// Open the HMD device
	priv->hmd_handle = open_device_idx(SONY_ID, PSVR_HMD, 4, idx);

	if(!priv->hmd_handle)
		goto cleanup;

	if(hid_set_nonblocking(priv->hmd_handle, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}

	// Open the HMD Control device
	priv->hmd_control = open_device_idx(SONY_ID, PSVR_HMD, 5, idx);

	if(!priv->hmd_control)
		goto cleanup;

	if(hid_set_nonblocking(priv->hmd_control, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}

	// turn the display on
	if (hid_write(priv->hmd_control, psvr_power_on, sizeof(psvr_power_on)) == -1) {
		ohmd_set_error(driver->ctx, "failed to write to device (power on)");
		goto cleanup;
	}

	// set VR mode for the hmd
	if (hid_write(priv->hmd_control, psvr_vrmode_on, sizeof(psvr_vrmode_on)) == -1) {
		ohmd_set_error(driver->ctx, "failed to write to device (set VR mode)");
		goto cleanup;
	}

	// ####################################################

	// Try to init the kinect here

	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_ScriptNode, &errors);
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "InitFromXml failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "Find depth generator failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "Find user generator failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	{
		ohmd_set_error(driver->ctx, "User generator doesn't support either skeleton or pose detection.\n");
		goto cleanup;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_HEAD_HANDS);

	rc = g_Context.StartGeneratingAll();
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "StartGenerating failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}

	XnCallbackHandle hUserCBs, hCalibrationCompleteCB, hPoseCBs;
	g_UserGenerator.RegisterUserCallbacks(NewUser, LostUser, NULL, hUserCBs);
	rc = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, NULL, hCalibrationCompleteCB);
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "Register to calibration complete failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}
	rc = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, NULL, hPoseCBs);
	if (rc != XN_STATUS_OK) {
		ohmd_set_error(driver->ctx, "Register to pose detected failed: %s\n", xnGetStatusString(rc));
		goto cleanup;
	}

	// ####################################################

	// Set default device properties
	ohmd_set_default_device_properties(&priv->base.properties);

	// Set device properties TODO: Get from device
	priv->base.properties.hsize = 0.126; //from calculated specs
	priv->base.properties.vsize = 0.071; //from calculated specs
	priv->base.properties.hres = 1920;
	priv->base.properties.vres = 1080;

	// Measurements taken from
	// https://github.com/gusmanb/PSVRFramework/wiki/Optical-characteristics
	priv->base.properties.lens_sep = 0.0630999878f;
	priv->base.properties.lens_vpos = 0.0394899882f;

	priv->base.properties.fov = DEG_TO_RAD(103.57f); //TODO: Confirm exact mesurements
	priv->base.properties.ratio = (1920.0f / 1080.0f) / 2.0f;

	priv->base.properties.control_count = 3;
	priv->base.properties.controls_hints[0] = OHMD_VOLUME_PLUS;
	priv->base.properties.controls_hints[1] = OHMD_VOLUME_MINUS;
	priv->base.properties.controls_hints[2] = OHMD_MIC_MUTE;
	priv->base.properties.controls_types[0] = OHMD_DIGITAL;
	priv->base.properties.controls_types[1] = OHMD_DIGITAL;
	priv->base.properties.controls_types[2] = OHMD_DIGITAL;

	// calculate projection eye projection matrices from the device properties
	ohmd_calc_default_proj_matrices(&priv->base.properties);

	// set up device callbacks
	priv->base.update = update_device;
	priv->base.close = close_device;
	priv->base.getf = getf;

	ofusion_init(&priv->sensor_fusion);

	return (ohmd_device*)priv;

cleanup:
	if (priv) {
		teardown(priv);
		free(priv);
	}


	if (g_pRecorder) {
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);

		if (g_pRecorder != NULL)
		{
			g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
			g_pRecorder->Release();
			delete g_pRecorder;
		}
		g_pRecorder = NULL;
	}


	return NULL;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	if ((desc->device_flags & (OHMD_DEVICE_FLAGS_LEFT_CONTROLLER |
				   OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER)) ==
	    (OHMD_DEVICE_FLAGS_LEFT_CONTROLLER |
	     OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER))
		return open_ds4_controller_device(driver, desc);
	else if (desc->device_class == OHMD_DEVICE_CLASS_CONTROLLER)
		return open_psmove_device(driver, desc);
	else
		return open_hmd_device(driver, desc);
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	struct hid_device_info* devs = hid_enumerate(SONY_ID, 0);
	struct hid_device_info* cur_dev = devs;

	// Loop HID devices to find stuff
	int id = 0, hmd_idx = 0, controller_idx = 0, psmove_idx = 0;
	while (cur_dev) {
		if (cur_dev->product_id == PSVR_HMD) {
			ohmd_device_desc* desc;

			// Warn if hidapi does not provide interface numbers
			if (cur_dev->interface_number == -1) {
				LOGE("hidapi does not provide PSVR interface numbers\n");
	#ifdef __APPLE__
				LOGE("see https://github.com/signal11/hidapi/pull/380\n");
	#endif
				break;
			}

			// Register one device for each IMU sensor interface
			if (cur_dev->interface_number == 4) {
				desc = &list->devices[list->num_devices++];

				strcpy(desc->driver, "OpenHMD Sony PSVR Driver");
				strcpy(desc->vendor, "Sony");
				strcpy(desc->product, "PSVR");

				desc->revision = 0;

				snprintf(desc->path, OHMD_STR_SIZE, "%d", hmd_idx);

				desc->driver_ptr = driver;
				desc->id = id++;

				desc->device_class = OHMD_DEVICE_CLASS_HMD;
				desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

				hmd_idx++;
			}
		} else if (cur_dev->product_id == DUALSHOCK_4) {
			ohmd_device_desc* desc = &list->devices[list->num_devices++];

			strcpy(desc->driver, "OpenHMD Sony DualShock 4 driver");
			strcpy(desc->vendor, "Sony");
			// "Motion controller - Left" or "Motion controller - Right"
			snprintf(desc->product, OHMD_STR_SIZE, "%S", cur_dev->product_string);

			desc->revision = 0;

			snprintf(desc->path, OHMD_STR_SIZE, "%d", controller_idx);

			desc->driver_ptr = driver;
			desc->id = id++;

			desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;
			// For now assign both hands
			desc->device_flags = (ohmd_device_flags)(OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING | OHMD_DEVICE_FLAGS_LEFT_CONTROLLER | OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER);
			// desc->device_flags |= OHMD_DEVICE_FLAGS_LEFT_CONTROLLER;
			// desc->device_flags |= OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER;

			controller_idx++;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);

	if (psmove_init(PSMOVE_CURRENT_VERSION)) {

		// Then, loop psmoveapi devices
		int count = psmove_count_connected();
		for (int i = count - 1; i >= 0; i--) {
			ohmd_device_desc* desc = &list->devices[list->num_devices++];

			strcpy(desc->driver, "OpenHMD Sony PlayStation Move controller driver");
			strcpy(desc->vendor, "Sony");
			snprintf(desc->product, OHMD_STR_SIZE, "%S", "Motion Controller");//cur_dev->product_string);

			desc->revision = 0;

			snprintf(desc->path, OHMD_STR_SIZE, "%d", psmove_idx);

			desc->driver_ptr = driver;
			desc->id = id++;

			desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;
			desc->device_flags = (ohmd_device_flags)(OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING | ((psmove_idx & 1) ?
			OHMD_DEVICE_FLAGS_LEFT_CONTROLLER :
			OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER));
			// For now assign alternating handedness
			// desc->device_flags |=

			psmove_idx++;
		}
	}
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down Sony PSVR driver");
	free(drv);
}
ohmd_driver* ohmd_create_psvr_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = (ohmd_driver*)ohmd_alloc(ctx, sizeof(ohmd_driver));

	if(!drv)
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;
	drv->ctx = ctx;

	return drv;
}
}
