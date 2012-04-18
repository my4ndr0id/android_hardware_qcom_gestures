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

#ifndef ANDROID_HARDWARE_GESTURE_STREAM_H
#define ANDROID_HARDWARE_GESTURE_STREAM_H

typedef enum
{
    GESTURE_SESSION_EVENT_NULL,
    GESTURE_SESSION_EVENT_CONFIG,
    GESTURE_SESSION_EVENT_START,
    GESTURE_SESSION_EVENT_RCVD_RESULT,
    GESTURE_SESSION_EVENT_STOP,
    GESTURE_SESSION_EVENT_TERMINATE
} GESTURE_SESSION_EVENT_ENUM;

typedef struct
{
    int32_t msg_type;
    void* msg;
    size_t len;
} GESTURE_EVENT_PAYLOAD_TYPE;

namespace android {

class GesturesMgr;
class GestureComm;
class GestureNotifier;

class GestureSession {
public:
    GestureSession(GesturesMgr* parent, GestureComm* comm, GestureNotifier* notifier);
    virtual ~GestureSession();
    int configByItem(void *msg, size_t len);
    int start();
    int stop();

    friend class GesturesMgr;
private:
    typedef enum {
        GESTURE_SESSION_STATE_INITIAL,
        GESTURE_SESSION_STATE_READY,
        GESTURE_SESSION_STATE_PROCESSING,
        GESTURE_SESSION_STATE_TERMINATED
    }GESTURE_SESSION_STATE_ENUM;

    int stateMachine(
        GESTURE_SESSION_EVENT_ENUM evt,
        void* payload,
        void** rsp,
        int32_t *pRspLength);
    int processEvtInInitialState(
        GESTURE_SESSION_EVENT_ENUM evt,
        void* payload,
        void** rsp,
        int32_t *pRspLength);
    int processEvtInReadyState(
        GESTURE_SESSION_EVENT_ENUM evt,
        void* payload,
        void** rsp,
        int32_t *pRspLength);
    int processEvtInProcessingState(
        GESTURE_SESSION_EVENT_ENUM evt,
        void* payload,
        void** rsp,
        int32_t *pRspLength);
    int processEvtInTerminatedState(
        GESTURE_SESSION_EVENT_ENUM evt,
        void* payload,
        void** rsp,
        int32_t *pRspLength);

    int processResultEvent(void* payload);

    GESTURE_SESSION_STATE_ENUM m_nState;
    GesturesMgr*               m_pMgr;
    GestureComm*               m_pComm;
    GestureNotifier*           m_pNotifier;
};

}; // namespace android

#endif
