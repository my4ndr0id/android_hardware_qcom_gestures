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

#ifndef ANDROID_HARDWARE_GESTURE_NOTIFIER_H
#define ANDROID_HARDWARE_GESTURE_NOTIFIER_H

#include "hardware/gestures.h"
#include <pthread.h>

namespace android {
#define MAX_CALLBACK_NUM 10

typedef struct 
{
    gesture_notify_callback notify_cb;
    gesture_data_callback data_cb;
    void *user;
    bool used;
} gesture_notifier_callback_t;

typedef enum 
{
    GESTURE_NODE_TYPE_DATA,
    GESTURE_NODE_TYPE_NOTIFY
} gesture_node_type_t;

typedef struct QNode
{
    struct QNode* next;
    void* data;
} QNode;

typedef struct DataNode
{
    gesture_result_t* gs_results;
} DataNode;

typedef struct NotifyNode
{
    int32_t msg_type;
    int32_t ext1;
    int32_t ext2;
} NotifyNode;

extern "C" void DestroyGestureResult(gesture_result_t** gs_results);

class NotifierQueue {
private:
    bool mInitialized;
    int mMaxQueueSize;
    gesture_node_type_t m_qType;
    QNode* m_head;
    QNode* m_tail;
    int m_qSize;
    pthread_mutex_t mQueueLock;
    pthread_cond_t mQueueWait;

    void enqueueInternal(QNode* node);
    QNode* dequeueInternal();

public:
    NotifierQueue();
    virtual ~NotifierQueue();
    bool enqueue(void *element);
    void flush();
    void* dequeue();
    void init(int maxQueueSize, gesture_node_type_t type);
    void deinit();
    bool isInitialized();
    bool isEmpty();
};

class GestureNotifier {
public:
    GestureNotifier();
    virtual ~GestureNotifier();

    void registerCallbacks(
            gesture_notify_callback notify_cb,
            gesture_data_callback data_cb,
            void *user);

    void unregisterCallbacks(void *user);

    void broadcastData(gesture_result_t* gs_results);
    void broadcastNotify(int32_t msg_type,
                         int32_t ext1,
                         int32_t ext2);

private:
    void broadcastDataInternal(gesture_result_t* gs_results);
    void broadcastNotifyInternal(int32_t msg_type,
                         int32_t ext1,
                         int32_t ext2);
    static void* StartDataCBThread(void* pUsrData);
    static void* StartNotifyCBThread(void* pUsrData);

    gesture_notifier_callback_t mCallbackList[MAX_CALLBACK_NUM];
    pthread_mutex_t mLock;
    pthread_t mDataPID;
    pthread_t mNotifyPID;
    NotifierQueue mDataQueue;   // Queue for  data callback
    NotifierQueue mNotifyQueue; // Queue for notify callback
};

}; // namespace android

#endif
