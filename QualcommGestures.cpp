

/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <utils/Log.h>
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <hardware/gestures.h>
#include "QualcommGestures.h"
#include "GesturesMgr.h"

#define LOG_NIDEBUG 0
#define LOG_TAG "QualcommGestures"

static hw_module_methods_t gesture_module_methods = {
    open: open_gesture_device,
};

static hw_module_t gesture_common  = {
  tag: HARDWARE_MODULE_TAG,
  version_major: 0,
  version_minor: 01,
  id: GESTURE_HARDWARE_MODULE_ID,
  name: "Qgesture",
  author:"Qcom",
  methods: &gesture_module_methods,
  dso: NULL,
};

gesture_module_t HAL_MODULE_INFO_SYM = {
  common: gesture_common,
  get_number_of_gesture_devices: get_number_of_gesture_devices,
};

gesture_device_ops_t gesture_ops = {
  set_callbacks:              android::set_gesture_CallBacks,
  start:                      android::start_gesture,
  stop:                       android::stop_gesture,
  set_parameters:             android::set_gesture_parameters,
  get_parameters:             android::get_gesture_parameters,
  send_command:               android::send_gesture_command,
  dump:                       android::gesture_dump,
};

namespace android {

typedef struct {
  gesture_device hw_dev;
  GesturesMgr *hardware;
  int gesture_released;
  int gestureId;
} gesture_hardware_t;

GesturesMgr *util_get_Hal_obj( struct gesture_device * device)
{
    GesturesMgr *hardware = NULL;
    if(device && device->priv){
        gesture_hardware_t *gestureHal = (gesture_hardware_t *)device->priv;
        hardware = gestureHal->hardware;
    }
    return hardware;
}

/**
 * The functions need to be provided by the gesture HAL.
 *
 * If getNumberOfGestureDevices() returns N, the valid gestureId
 * for opening gesture device is 0 to N-1. 
 */
extern "C" int get_number_of_gesture_devices()
{
    /* try to query every time we get the call!*/

    LOGE("Q%s: E", __func__);
    return GesturesMgr::GetNumberOfGestureDevices( );
}

/* HAL should return NULL if it fails to open gesture device. */
extern "C" int open_gesture_device(
  const struct hw_module_t* module, const char* id,
          struct hw_device_t** hw_device)
{
    LOGE("%s:  E", __func__);
    int rc = -1;
    gesture_device *device = NULL;
    if(module && id && hw_device) {
        int gestureId = atoi(id);

        if (!strcmp(module->name, gesture_common.name)) {
            gesture_hardware_t *gestureHal =
                (gesture_hardware_t *) malloc(sizeof (gesture_hardware_t));
            if(!gestureHal) {
                *hw_device = NULL;
				    LOGE("%s:  end in no mem", __func__);
				    return rc;
		    }
            /* we have the gesture obj malloced */
            memset(gestureHal, 0, sizeof (gesture_hardware_t));
            gestureHal->hardware = GesturesMgr::CreateInstance(gestureId);
            if (gestureHal->hardware) {
				gestureHal->gestureId = gestureId;
		        device = &gestureHal->hw_dev;
                device->common.close = close_gesture_device;
                device->ops = &gesture_ops;
                device->priv = (void *)gestureHal;
                rc =  0;
            } else {
                free(gestureHal);
                device = NULL;
            }
        }
    }
	/* pass actual hw_device ptr to framework. This amkes that we actally be use memberof() macro */
    *hw_device = (hw_device_t*)&device->common;
    LOGE("%s: X end rc %d", __func__, rc);
    return rc;
}

extern "C" int close_gesture_device(hw_device_t *hw_dev)
{
    LOGE("Q%s: device =%p E", __func__, hw_dev);
    int rc =  -1;
    gesture_device_t *device = (gesture_device_t *)hw_dev;

    if(device) {
        gesture_hardware_t *gestureHal = (gesture_hardware_t *)device->priv;
        if(gestureHal) {
            GesturesMgr *hardware = util_get_Hal_obj( device);
            if(hardware != NULL) {
                GesturesMgr::DestroyInstance(hardware);
            }
            free(gestureHal);
            device->priv = NULL;
        }
        rc = 0;
    }
    LOGE("Q%s: X", __func__);
    return rc;
}

void set_gesture_CallBacks(struct gesture_device * device,
                           gesture_notify_callback notify_cb,
                           gesture_data_callback data_cb,
                           void *user,
                           bool isreg)
{
    LOGE("Q%s: E", __func__);
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL){
        hardware->setCallbacks(notify_cb, data_cb, user, isreg);
    }
}

int start_gesture(struct gesture_device * device)
{
    LOGE("Q%s: E", __func__);
    int rc = -1;
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL){
        rc = hardware->startGesture();
    }
    LOGE("Q%s: X", __func__);
    return rc;
}

void stop_gesture(struct gesture_device * device)
{
    LOGE("Q%s: E", __func__);
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL){
        hardware->stopGesture( );
    }
}

int set_gesture_parameters(struct gesture_device * device, const char *parms)

{
    LOGE("Q%s: E", __func__);
    int rc = -1;
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL && parms){
        rc = hardware->setParameters(parms);
  }
  return rc;
}

char* get_gesture_parameters(struct gesture_device * device)
{
    LOGE("Q%s: E", __func__);
    GesturesMgr *hardware = util_get_Hal_obj(device);
    char *parms = NULL;
    if(hardware != NULL){
        hardware->getParameters(&parms);
    }
    return parms;
}

int send_gesture_command(struct gesture_device * device,
            int32_t cmd, int32_t arg1, int32_t arg2)
{
    LOGE("Q%s: E", __func__);
    int rc = -1;
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL){
        rc = hardware->sendCommand( cmd, arg1, arg2);
    }
    return rc;
}

int gesture_dump(struct gesture_device * device, int fd)
{
    LOGE("Q%s: E", __func__);
    int rc = -1;
    GesturesMgr *hardware = util_get_Hal_obj(device);
    if(hardware != NULL){
        rc = hardware->dump( fd );
    }
    return rc;
}

}; // namespace android
