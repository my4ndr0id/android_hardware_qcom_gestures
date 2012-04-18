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

#include "common.h"
#include "GesturesMgr.h"
#include "GestureComm.h"
#include "GestureNotifier.h"
#include "GestureSession.h"

namespace android {

GestureSession::GestureSession(
                        GesturesMgr* parent, 
                        GestureComm* comm, 
                        GestureNotifier* notifier)
        : m_nState(GESTURE_SESSION_STATE_INITIAL),
          m_pMgr(parent),
          m_pComm(comm),
          m_pNotifier(notifier)
{
    CDBG("%s", __func__);
}

GestureSession::~GestureSession()
{
    CDBG("%s: E", __func__);
    if(GESTURE_SESSION_STATE_TERMINATED != m_nState) {
        stateMachine(GESTURE_SESSION_EVENT_TERMINATE, NULL, NULL, NULL);
    }
    CDBG("%s: X", __func__);
}

int GestureSession::start()
{
    return stateMachine(
            GESTURE_SESSION_EVENT_START,
            NULL, NULL, NULL);
}

int GestureSession::stop()
{
    return stateMachine(
            GESTURE_SESSION_EVENT_STOP,
            NULL, NULL, NULL);
}

int GestureSession::stateMachine(
    GESTURE_SESSION_EVENT_ENUM evt,
    void* payload,
    void** rsp,
    int32_t *pRspLength)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    switch(m_nState) {
    case GESTURE_SESSION_STATE_INITIAL:
        ret = processEvtInInitialState(evt,payload,rsp,pRspLength);
        break;
    case GESTURE_SESSION_STATE_READY:
        ret = processEvtInReadyState(evt,payload,rsp,pRspLength);
        break;
    case GESTURE_SESSION_STATE_PROCESSING:
        ret = processEvtInProcessingState(evt,payload,rsp,pRspLength);
        break;
    case GESTURE_SESSION_STATE_TERMINATED:
        ret = processEvtInTerminatedState(evt,payload,rsp,pRspLength);
        break;
    default:
        CDBG_ERROR("%s: Error state (%d)", __func__, m_nState);
        ret = -GESTURE_STATUS_E_GENERAL;
        break;
    }
    return ret;
}

int GestureSession::processEvtInInitialState(
    GESTURE_SESSION_EVENT_ENUM evt,
    void* payload,
    void** rsp,
    int32_t *pRspLength)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    switch (evt) {
    case GESTURE_SESSION_EVENT_CONFIG:
        {
            if (NULL != payload) {
                GESTURE_EVENT_PAYLOAD_TYPE* pParamPayload =
                    (GESTURE_EVENT_PAYLOAD_TYPE*)payload;
                if(NULL != m_pComm) {
                    ret = m_pComm->SendMsg(pParamPayload->msg_type, pParamPayload->msg, pParamPayload->len);
                    if (GESTURE_STATUS_NO_ERROR == ret) {
                        m_nState = GESTURE_SESSION_STATE_READY;
                    }
                } else {
                    CDBG_ERROR("%s: GestureComm is NULL", __func__);
                    ret = -GESTURE_STATUS_E_GENERAL;
                }
            } else {
                CDBG_ERROR("%s: Parameter payload is NULL", __func__);
                ret = -GESTURE_STATUS_E_GENERAL;
            }
        }
        break;
    case GESTURE_SESSION_EVENT_TERMINATE:
        m_nState = GESTURE_SESSION_STATE_TERMINATED;
        break;
    case GESTURE_SESSION_EVENT_START:
    case GESTURE_SESSION_EVENT_RCVD_RESULT:
    case GESTURE_SESSION_EVENT_STOP:
    default:
        CDBG_ERROR("%s: cannot handle evt (%d) in state (%d)", __func__, evt, m_nState);
        ret = -GESTURE_STATUS_E_GENERAL;
        break;
    }
    return ret;
}

int GestureSession::processEvtInReadyState(
    GESTURE_SESSION_EVENT_ENUM evt,
    void* payload,
    void** rsp,
    int32_t *pRspLength)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    switch (evt) {
    case GESTURE_SESSION_EVENT_CONFIG:
        {
            if (NULL != payload) {
                GESTURE_EVENT_PAYLOAD_TYPE* pParamPayload =
                    (GESTURE_EVENT_PAYLOAD_TYPE*)payload;
                if(NULL != m_pComm) {
                    ret = m_pComm->SendMsg(pParamPayload->msg_type, pParamPayload->msg, pParamPayload->len);
                } else {
                    CDBG_ERROR("%s: GestureComm is NULL", __func__);
                    ret = -GESTURE_STATUS_E_GENERAL;
                }
            }
        }
        break;
    case GESTURE_SESSION_EVENT_START:
        {
            if(NULL != m_pComm) {
                ret = m_pComm->SendMsg(GESTURE_CTL_START, NULL, 0);
                if (GESTURE_STATUS_NO_ERROR == ret) {
                    m_nState = GESTURE_SESSION_STATE_PROCESSING;
                } else {
                    CDBG_ERROR("%s: start gesture failed", __func__);
                }
            } else {
                CDBG_ERROR("%s: GestureComm is NULL", __func__);
                ret = -GESTURE_STATUS_E_GENERAL;
            }
        }
        break;
    case GESTURE_SESSION_EVENT_TERMINATE:
        m_nState = GESTURE_SESSION_STATE_TERMINATED;
        break;
    case GESTURE_SESSION_EVENT_RCVD_RESULT:
    case GESTURE_SESSION_EVENT_STOP:
    default:
        CDBG_ERROR("%s: cannot handle evt (%d) in state (%d)", __func__, evt, m_nState);
        ret = -GESTURE_STATUS_E_GENERAL;
        break;
    }
    return ret;
}

int GestureSession::processResultEvent(void* payload)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    gesture_result_t* gs_results = NULL;

    if (NULL != payload) {
        gs_ctl_result_t* pResults = (gs_ctl_result_t *)payload;
        gs_results = (gesture_result_t*)malloc(sizeof(gesture_result_t));
        if (NULL != gs_results) {
            gs_results->number_of_events = pResults->number_of_outcomes;
            gs_results->events =
                (gesture_event_t*)malloc(sizeof(gesture_event_t) * gs_results->number_of_events);
            if (NULL == gs_results->events) {
                CDBG_ERROR("%s: No memory for gesture_event_t", __func__);
                ret = -GESTURE_STATUS_E_NO_MEMORY;
                goto end;
            }
            memset(gs_results->events, 0, sizeof(gesture_event_t) * gs_results->number_of_events);
            for (int i = 0; i < gs_results->number_of_events; i++) {
                gs_results->events[i].version = pResults->outcomes[i].version;
                gs_results->events[i].id = pResults->outcomes[i].id;
                gs_results->events[i].type = pResults->outcomes[i].type;
                gs_results->events[i].subtype = pResults->outcomes[i].subtype;
                gs_results->events[i].timestamp = pResults->outcomes[i].timestamp;
                gs_results->events[i].confidence = pResults->outcomes[i].confidence;
                gs_results->events[i].velocity = pResults->outcomes[i].velocity;
                gs_results->events[i].location.num_of_points = 2; // currently only end and begin points
                if (gs_results->events[i].location.num_of_points > 0) {
                    int location_size = 
                        sizeof(gesture_vector_t) * gs_results->events[i].location.num_of_points;
                    gs_results->events[i].location.pPoints = 
                        (gesture_vector_t*)malloc(location_size);
                    if (NULL != gs_results->events[i].location.pPoints) {
                        memset(gs_results->events[i].location.pPoints, 0, location_size);
                        memcpy(gs_results->events[i].location.pPoints,
                               &pResults->outcomes[i].location,
                               location_size);
                    } else {
                        CDBG_ERROR("%s: No memory for gesture_vector_t", __func__);
                        ret = -GESTURE_STATUS_E_NO_MEMORY;
                        goto end;
                    }
                }

                gs_results->events[i].extendinfo.len = GS_CTL_MAX_OUTCOMES;
                gs_results->events[i].extendinfo.buf =
                    malloc(gs_results->events[i].extendinfo.len);
                if (NULL != gs_results->events[i].extendinfo.buf) {
                    memcpy(gs_results->events[i].extendinfo.buf, 
                           pResults->outcomes[i].ext,
                           gs_results->events[i].extendinfo.len);
                } else {
                    CDBG_ERROR("%s: No memory for extend info buffer", __func__);
                    ret = -GESTURE_STATUS_E_NO_MEMORY;
                    goto end;
                }
            }
            // mgr/notifier will take care of freeing the memory
            m_pNotifier->broadcastData(gs_results);
            ret = GESTURE_STATUS_NO_ERROR;
        }
    } else {
        CDBG_ERROR("%s: NULL payload for gesture result", __func__);
        ret = -GESTURE_STATUS_E_GENERAL;
    }

end:
    if ((GESTURE_STATUS_NO_ERROR != ret) && (NULL != gs_results)) {
        DestroyGestureResult(&gs_results);
    }
    return ret;
}

int GestureSession::processEvtInProcessingState(
    GESTURE_SESSION_EVENT_ENUM evt,
    void* payload,
    void** rsp,
    int32_t *pRspLength)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    switch (evt) {
    case GESTURE_SESSION_EVENT_RCVD_RESULT:
        {
            GESTURE_EVENT_PAYLOAD_TYPE* pResultPayload =
                (GESTURE_EVENT_PAYLOAD_TYPE*)payload;
            if (GESTURE_SESSION_EVENT_RCVD_RESULT == pResultPayload->msg_type) {
                ret = processResultEvent(pResultPayload->msg);
            } else {
                CDBG("%s: msg type (%d) not processed", __func__, pResultPayload->msg_type);
            }
        }
        break;
    case GESTURE_SESSION_EVENT_STOP:
    case GESTURE_SESSION_EVENT_TERMINATE:
        {
            if(NULL != m_pComm) {
                ret = m_pComm->SendMsg(GESTURE_CTL_STOP, NULL, 0);
                if (GESTURE_STATUS_NO_ERROR != ret) {
                    CDBG_ERROR("%s: stop gesture failed", __func__);
                }
                // Change state anyway, even we have error sending stop cmd to backend
                if (GESTURE_SESSION_EVENT_STOP == evt) {
                    m_nState = GESTURE_SESSION_STATE_READY;
                } else {
                    m_nState = GESTURE_SESSION_STATE_TERMINATED;
                }
            } else {
                CDBG_ERROR("%s: GestureComm is NULL", __func__);
                ret = -GESTURE_STATUS_E_GENERAL;
            }
        }
        break;
    case GESTURE_SESSION_EVENT_CONFIG:
        {
            // Backend gesture lib allows to config even when gesture is in process,
            // so we allow this event in this state.
            if (NULL != payload) {
                GESTURE_EVENT_PAYLOAD_TYPE* pParamPayload =
                    (GESTURE_EVENT_PAYLOAD_TYPE*)payload;
                if(NULL != m_pComm) {
                    ret = m_pComm->SendMsg(pParamPayload->msg_type, pParamPayload->msg, pParamPayload->len);
                } else {
                    CDBG_ERROR("%s: GestureComm is NULL", __func__);
                    ret = -GESTURE_STATUS_E_GENERAL;
                }
            }
        }
        break;
    case GESTURE_SESSION_EVENT_START:
    default:
        CDBG_ERROR("%s: cannot handle evt (%d) in state (%d)", __func__, evt, m_nState);
        ret = -GESTURE_STATUS_E_GENERAL;
        break;
    }
    return ret;
}

int GestureSession::processEvtInTerminatedState(
    GESTURE_SESSION_EVENT_ENUM evt,
    void* payload,
    void** rsp,
    int32_t *pRspLength)
{
    CDBG_ERROR("%s: cannot handle evt (%d) in state (%d)", __func__, evt, m_nState);
    return GESTURE_STATUS_NO_ERROR;
}

int GestureSession::configByItem(void *msg, size_t len)
{
    int ret = GESTURE_STATUS_NO_ERROR;
    GESTURE_EVENT_PAYLOAD_TYPE payload;
    memset(&payload, 0, sizeof(payload));

    payload.msg_type = GESTURE_CTL_PARAM;
    payload.msg = msg;
    payload.len = len;
    ret = stateMachine(
            GESTURE_SESSION_EVENT_CONFIG,
            &payload,
            NULL, NULL);
    return ret;
}

}; // namespace android
