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

#ifndef ANDROID_HARDWARE_GESTURES_COMM_H
#define ANDROID_HARDWARE_GESTURES_COMM_H

#include "common.h"
extern "C" {
#include "gestures.h"
}

namespace android {

typedef enum
{
    GESTURE_EVT_TYPE_RESULT,
    GESTURE_EVT_TYPE_MAX
} gesture_evt_type_t;

typedef void (*gesture_result_callback)(int32_t msg_type,
                                       void* msg,
                                       int32_t len,
                                       void *user);

class GestureComm {
public:
    GestureComm(int device_id, gesture_result_callback cb, void* user);
    virtual ~GestureComm();
    int Open(const char* str_device_name);
    int Close();
    int SendMsg(int type, void* msg, int32_t len);
    static void QueryGesturesDeviceInfo(
        int8_t &numDevices,
        char deviceName[GESTURE_DEVICES_MAX][GESTURE_DEV_NAME_LEN]);

private:
    int mDeviceId;
    gesture_result_callback mCB;
    void* mUser;

    bool    m_isOpened;
    int32_t mCtrlfd;     // ctrl fd
    int32_t mDSfd;       // domain socket fd

    pthread_mutex_t mLock;
    pthread_cond_t mWait;
    int32_t mStatus;

    int32_t mFDs[2];
    pthread_t mPID;
    int32_t mState;
    int mTimeoutms;

private:
    int SubscribeEvt(bool bReg);
    int PollSig(uint32_t cmd);
    void PollSigDone();
    void* PollFunction();
    int LaunchPollThread();
    void ProcPipe(int32_t fd);
    void ProcEvt(int32_t fd);
    void ProcSocket(int32_t fd);
    int ReleasePollThread();
    static void* StartPollThread(void* pUsrData);  
};

}; // namespace android

#endif
