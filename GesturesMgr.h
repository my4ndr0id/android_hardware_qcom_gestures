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

#ifndef ANDROID_HARDWARE_GESTURES_MANAGER_H
#define ANDROID_HARDWARE_GESTURES_MAANGER_H

#include "common.h"
#include "hardware/gestures.h"
#include "GestureParameters.h"

namespace android {

class GestureComm;
class GestureNotifier;
class GestureSession;

class GesturesMgr {
public:
    static GesturesMgr* CreateInstance(int  device_id);
    static void DestroyInstance(GesturesMgr* pMgr);
    static int GetNumberOfGestureDevices();

    /** Set the notification and data callbacks */
    void setCallbacks(
            gesture_notify_callback notify_cb,
            gesture_data_callback data_cb,
            void *user,
            bool isreg);

    /**
     * Start gesture processing.
     */
    int startGesture();

    /**
     * Stop a previously started gesture processing.
     */
    void stopGesture();

    /**
     * Set the gesture device parameters. This returns BAD_VALUE if 
     * any parameter is invalid or not supported. 
     */
    int setParameters(const char *parms);

    /** Retrieve the gesture device parameters.  The buffer returned
        by the gesture HAL must be freed by the caller.
     */
    int getParameters(char **parms);

    /**
     * Send command to camera driver.
     */
    int sendCommand(int32_t cmd, int32_t arg1, int32_t arg2);

    /**
     * Release the hardware resources owned by this object.  Note that this is
     * *not* done in the destructor.
     */
    void release();

    /**
     * Dump state of the gesture device hardware
     */
    int dump(int fd);

    virtual ~GesturesMgr();
    static void CommCallback(int32_t msg_type,
                             void* msg,
                             int32_t len,
                             void *user);

public:
    static GesturesMgr* m_pInstances[GESTURE_DEVICES_MAX];
    static int ref_count[GESTURE_DEVICES_MAX];
    static int8_t num_gesture_devices;
    static char gesture_dev_name[GESTURE_DEVICES_MAX][GESTURE_DEV_NAME_LEN];
    static pthread_mutex_t g_mutex;

private:
    GesturesMgr(int device_id);
    int getMyDeviceId() {return mDeviceId;};
    void HandleGestureResult(int32_t msg_type, void* msg, int32_t len);
    int configSession(GestureSession *pSession, GestureParameters& param);

private:
    bool              mActive;
    int               mDeviceId;
    GestureParameters mParameters;
    GestureComm *     m_pComm;
    GestureNotifier * m_pNotifier;
    GestureSession *  m_pSession;
    pthread_mutex_t   mLock;
};

}; // namespace android

#endif
