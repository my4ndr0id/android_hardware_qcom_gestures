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

#include <fcntl.h>
#include "common.h"
#include "GestureComm.h"
#include "GestureNotifier.h"
#include "GestureSession.h"
#include "GestureParameters.h"
#include "GesturesMgr.h"

namespace android {
GesturesMgr* GesturesMgr::m_pInstances[GESTURE_DEVICES_MAX] = {0};
int GesturesMgr::ref_count[GESTURE_DEVICES_MAX] = {0};
int8_t GesturesMgr::num_gesture_devices = 0;
char GesturesMgr::gesture_dev_name[GESTURE_DEVICES_MAX][GESTURE_DEV_NAME_LEN] = {{0}};
pthread_mutex_t GesturesMgr::g_mutex = PTHREAD_MUTEX_INITIALIZER;

GesturesMgr* GesturesMgr::CreateInstance(int  device_id)
{
    GesturesMgr* pMgr = NULL;

    pthread_mutex_lock(&g_mutex);
    if(m_pInstances[device_id] == NULL) {
        pMgr = new GesturesMgr(device_id);
        m_pInstances[device_id] = pMgr;
        ref_count[device_id] = 0;
    }
    ref_count[device_id]++;
    pthread_mutex_unlock(&g_mutex);

    return pMgr;
}

void GesturesMgr::DestroyInstance(GesturesMgr* pMgr)
{
    if (!pMgr) {
        return;
    }

    int device_id = pMgr->getMyDeviceId();
    pthread_mutex_lock(&g_mutex);
    ref_count[device_id]--;
    if(ref_count[device_id] <= 0) {
        if(m_pInstances[device_id]) {
            delete m_pInstances[device_id];
            m_pInstances[device_id] = NULL;
            ref_count[device_id] = 0;
        }
    }
    pthread_mutex_unlock(&g_mutex);
}

int GesturesMgr::GetNumberOfGestureDevices()
{
    int num_of_device = 0;

    /* lock the mutex */
    pthread_mutex_lock(&g_mutex);
    if (num_gesture_devices <= 0) {
        GestureComm::QueryGesturesDeviceInfo(num_gesture_devices, gesture_dev_name);
    }
    CDBG("%s: num_gesture_devices = %d\n", __func__, num_gesture_devices);
    num_of_device = num_gesture_devices;
    pthread_mutex_unlock(&g_mutex);
    return num_of_device;
}

/* constructor */
GesturesMgr::GesturesMgr(int deviceId)
                  : mActive(false),
                    mDeviceId(deviceId),
                    mParameters(),
                    m_pComm(NULL),
                    m_pNotifier(NULL),
                    m_pSession(NULL)
{
    CDBG("%s: E", __func__);

    pthread_mutex_init(&mLock, NULL);

    if ((deviceId <0) || (deviceId>=num_gesture_devices)) {
        CDBG_ERROR("Invalid device ID: id = %d (out of %d)", mDeviceId, num_gesture_devices);
        return;
    }

    /* Open Gesture communication tunnel */
    m_pComm = new GestureComm(deviceId, CommCallback, this);
    if(!m_pComm) {
        CDBG_ERROR("Create GestureComm failed: id = %d", mDeviceId);
        return;
    }

    if (m_pComm->Open(gesture_dev_name[deviceId]) != GESTURE_STATUS_NO_ERROR) {
        CDBG_ERROR("Open communication channel failed");
        return;
    }

    m_pNotifier = new GestureNotifier();
    if(!m_pNotifier) {
        CDBG_ERROR("Create GesturesNotifier failed: id = %d", mDeviceId);
        return;
    }

    m_pSession = new GestureSession(this, m_pComm, m_pNotifier);
    if(!m_pSession) {
        CDBG_ERROR("Create GesturesSession failed: id = %d", mDeviceId);
        return;
    }

    // initialize parameters to default values
    mParameters.setCameraInput(1); // using front camera by default
    mParameters.setGestureMode(101); //GESTURE_MODE_NEAR_SWIPE
    mParameters.setGestureSubMode(0);
    mParameters.setCursorType(0);
    mParameters.setClickMode(0);
    mParameters.setCoordinateMode(0);
    mParameters.setOrientation(0);
    mParameters.setTouchEnabled(false);

    mActive = true;
    CDBG("%s: X", __func__);
}

GesturesMgr::~GesturesMgr()
{
    CDBG("%s: E", __func__);
    release();
    CDBG("%s: X", __func__);
}

void GesturesMgr::release()
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    mActive = false;

    if (NULL != m_pSession) {
        m_pSession->stateMachine(
                    GESTURE_SESSION_EVENT_TERMINATE,
                    NULL, NULL, NULL);
        delete m_pSession;
        m_pSession = NULL;
    }
    if (NULL != m_pComm) {
        m_pComm->Close();
        delete m_pComm;
        m_pComm = NULL;
    }
    if (NULL != m_pNotifier) {
        delete m_pNotifier;
        m_pNotifier = NULL;
    }
    pthread_mutex_unlock(&mLock);
    pthread_mutex_destroy(&mLock);

    CDBG("%s: X", __func__);
}

void GesturesMgr::setCallbacks(
    gesture_notify_callback notify_cb,
    gesture_data_callback data_cb,
    void *user,
    bool isreg)
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        pthread_mutex_unlock(&mLock);
        return;
    }

    if (m_pNotifier != NULL) {
        if (isreg) {
            m_pNotifier->registerCallbacks(notify_cb,data_cb,user);
        } else {
            m_pNotifier->unregisterCallbacks(user);
        }
    }
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
}

int GesturesMgr::dump(int fd)
{
    CDBG_ERROR("%s: not supported yet", __func__);
    return -1;
}

int GesturesMgr::sendCommand(int32_t command, int32_t arg1, int32_t arg2)
{
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        return -GESTURE_STATUS_E_GENERAL;
    }

    int rc = GESTURE_STATUS_NO_ERROR;
    return rc;
}

int GesturesMgr::startGesture()
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        pthread_mutex_unlock(&mLock);
        return -GESTURE_STATUS_E_GENERAL;
    }

    int ret = GESTURE_STATUS_NO_ERROR;
    if (NULL == m_pSession) {
        m_pSession = new GestureSession(this, m_pComm, m_pNotifier);
        if (NULL != m_pSession) {
            ret = configSession(m_pSession, mParameters);
        } else {
            CDBG_ERROR("%s: Create Gesture session failed", __func__);
            ret = -GESTURE_STATUS_E_NO_MEMORY;
        }
    }

    if (GESTURE_STATUS_NO_ERROR == ret) {
        ret = m_pSession->start();
    }

    pthread_mutex_unlock(&mLock);
    CDBG("%s: X, ret = %d", __func__, ret);
    return ret;
}

void GesturesMgr::stopGesture()
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        pthread_mutex_unlock(&mLock);
        return;
    }

    if (NULL != m_pSession) {
        m_pSession->stop();
    } else {
        CDBG_ERROR("%s: No Gesture session is running, no need to stop", __func__);
    }
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
}

int GesturesMgr::setParameters(const char *parms)
{
    pthread_mutex_lock(&mLock);
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        pthread_mutex_unlock(&mLock);
        return -GESTURE_STATUS_E_GENERAL;
    }

    GestureParameters param(parms);
    int rc = configSession(m_pSession, param);
    pthread_mutex_unlock(&mLock);

    return rc;
}

int GesturesMgr::configSession(GestureSession *pSession, GestureParameters& param)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    gs_ctl_parameter_t ctl_param;
    if (NULL == pSession) {
        return ret;
    }

    // Config gesture mode
    int mode = param.getGestureMode();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_MODE;
    ctl_param.mode = mode;
    ret = pSession->configByItem(
                        (void *)(&ctl_param),
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set Mode (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setGestureMode(mode);

    // Config gesture sub mode
    int submode = param.getGestureSubMode();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_SUBMODE;
    ctl_param.submode = submode;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set SubMode (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setGestureSubMode(submode);

    // Config enable touch
    bool enabled = param.getTouchEnabled();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_ENABLE_TOUCH;
    ctl_param.enable_touch = enabled;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set enable touch (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setTouchEnabled(enabled);

    // Config orientation
    int orientation = param.getOrientation();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_ORIENTATION;
    ctl_param.orientation = orientation;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set orientation (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setOrientation(orientation);

    // Config orientation
    int coordMode = param.getCoordinateMode();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_COORDINATE_MODE;
    ctl_param.coordinate_mode = coordMode;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set coordinate mode (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setCoordinateMode(coordMode);

    // Config cursor type
    int cursor = param.getCursorType();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_CURSOR;
    ctl_param.cursor = cursor;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set cursor type (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setCursorType(cursor);

    // Config orientation
    int clickMode = param.getClickMode();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_CLICK_MODE;
    ctl_param.click_mode = clickMode;
    ret = pSession->configByItem(
                        (void *)(&ctl_param), 
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set coordinate mode (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setClickMode(clickMode);

    // Config extended config
    char *ext = param.getExtendedConfig();
    if (NULL != ext && strlen(ext)>0) {
        int len = strlen(ext);
        if (len > GS_CTL_MAX_EXTENDED_CONFIG) {
            len = GS_CTL_MAX_EXTENDED_CONFIG;
        }
        memset(&ctl_param, 0, sizeof(ctl_param));
        ctl_param.type = GS_CTL_PARAMETER_EXTENDED_CONFIG;
        memcpy(ctl_param.extended_config, ext, len);
        ret = pSession->configByItem(
                            (void *)(&ctl_param), 
                            sizeof(ctl_param));
        if (GESTURE_STATUS_NO_ERROR != ret) {
            CDBG_ERROR("%s: fail set extended config (rc = %d)", __func__, ret);
            return ret;
        }
        mParameters.setExtendedConfig(ext);
    }

    // Config camera id
    int cameraId = param.getCameraInput();
    memset(&ctl_param, 0, sizeof(ctl_param));
    ctl_param.type = GS_CTL_PARAMETER_CAMERA;
    ctl_param.camera = cameraId;
    ret = pSession->configByItem(
                        (void *)(&ctl_param),
                        sizeof(ctl_param));
    if (GESTURE_STATUS_NO_ERROR != ret) {
        CDBG_ERROR("%s: fail set CameraId (rc = %d)", __func__, ret);
        return ret;
    }
    mParameters.setCameraInput(cameraId);

    // Config coordinate range
    float range[6];
    memset(range, 0, sizeof(range));
    if (param.getCoordinateRange(range)) {
        memset(&ctl_param, 0, sizeof(ctl_param));
        ctl_param.type = GS_CTL_PARAMETER_COORDINATE_RANGE;
        ctl_param.coordinate_range.x_min = range[0];
        ctl_param.coordinate_range.x_max = range[1];
        ctl_param.coordinate_range.y_min = range[2];
        ctl_param.coordinate_range.y_max = range[3];
        ctl_param.coordinate_range.z_min = range[4];
        ctl_param.coordinate_range.z_max = range[5];
        ret = pSession->configByItem(
                            (void *)(&ctl_param),
                            sizeof(ctl_param));
        if (GESTURE_STATUS_NO_ERROR != ret) {
            CDBG_ERROR("%s: fail set coordinateRange (rc = %d)", __func__, ret);
            return ret;
        }
        mParameters.setCoordinateRange(range[0], range[1],
                                       range[2], range[3],
                                       range[4], range[5]);
    }

    return ret;
}

int GesturesMgr::getParameters(char **parms)
{
    pthread_mutex_lock(&mLock);
    if (!mActive) {
        CDBG("%s: GestureMgr not in active mode, return here", __func__);
        pthread_mutex_unlock(&mLock);
        return -GESTURE_STATUS_E_GENERAL;
    }

    if (NULL == parms) {
        CDBG_ERROR("%s: parm is NULL", __func__);
        pthread_mutex_unlock(&mLock);
        return -GESTURE_STATUS_E_INVALID_INPUT;
    }

    *parms = NULL;
    char* str = mParameters.flatten();
    pthread_mutex_unlock(&mLock);

    *parms = str;
    return GESTURE_STATUS_NO_ERROR;
}

void GesturesMgr::CommCallback(int32_t msg_type,
                             void* msg,
                             int32_t len,
                             void *user) 
{
    if (!user) {
        CDBG_ERROR("%s: userPtr is NULL. X", __func__);
        return;
    }

    GesturesMgr *pMe = (GesturesMgr *)user;
    pMe->HandleGestureResult(msg_type, msg, len);
}

#define MAX_TRY_LOCK_NUM 5
#define TRY_LOCK_SLEEP_TIME 60000

void GesturesMgr::HandleGestureResult(int32_t msg_type, void* msg, int32_t len) {
    for (int i = 0; i < MAX_TRY_LOCK_NUM; i++) {
        int ret = pthread_mutex_trylock(&mLock);
        if (0 == ret) {
            if (!mActive) {
                CDBG("%s: GestureMgr not in active mode, return here", __func__);
                pthread_mutex_unlock(&mLock);
                break;
            }

            if (NULL != m_pSession) {
                if (GESTURE_EVT_TYPE_RESULT == msg_type) {
                    GESTURE_EVENT_PAYLOAD_TYPE payload;
                    memset(&payload, 0, sizeof(payload));
                    payload.msg_type = GESTURE_SESSION_EVENT_RCVD_RESULT;
                    payload.msg = msg;
                    payload.len = len;
                    m_pSession->stateMachine(GESTURE_SESSION_EVENT_RCVD_RESULT, &payload, NULL, NULL);
                }
            }
            pthread_mutex_unlock(&mLock);
            break;
        } else {
            usleep(TRY_LOCK_SLEEP_TIME);
        }
    }
}

}; // namespace android
