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
#include "GestureNotifier.h"

namespace android {

#define MAX_QUEUE_SIZE 5

void DestroyGestureResult(gesture_result_t** gs_results)
{
    if (NULL != gs_results) {
        return;
    }
    gesture_result_t* ptr = *gs_results;
    if ((NULL != ptr) && (NULL != ptr->events)) {
        for (int i = 0; i < ptr->number_of_events; i++) {
            if (NULL != ptr->events[i].location.pPoints) {
                free(ptr->events[i].location.pPoints);
                ptr->events[i].location.pPoints = NULL;
            }
            if (NULL != ptr->events[i].extendinfo.buf) {
                free(ptr->events[i].extendinfo.buf);
                ptr->events[i].extendinfo.buf = NULL;
            }
        }
        free(ptr->events);
    }
    free(ptr);
    *gs_results = NULL;
}

/* constructor */
GestureNotifier::GestureNotifier()
{
    pthread_mutex_init(&mLock, NULL);
    memset(mCallbackList, 0, sizeof(mCallbackList));
    mDataQueue.init(MAX_QUEUE_SIZE, GESTURE_NODE_TYPE_DATA);
    mNotifyQueue.init(MAX_QUEUE_SIZE, GESTURE_NODE_TYPE_NOTIFY);
    pthread_create(&mDataPID, NULL, StartDataCBThread, (void *)this);
    pthread_create(&mNotifyPID, NULL, StartNotifyCBThread, (void *)this);
}

GestureNotifier::~GestureNotifier()
{
    CDBG("%s: E", __func__);
    if (mDataQueue.isInitialized()) {
        mDataQueue.deinit();
    }
    if (mNotifyQueue.isInitialized()) {
        mNotifyQueue.deinit();
    }
    if (pthread_join(mDataPID, NULL) != 0) {
        CDBG("%s: pthread dead already\n", __func__);
    }
    if (pthread_join(mNotifyPID, NULL) != 0) {
        CDBG("%s: pthread dead already\n", __func__);
    }

    pthread_mutex_lock(&mLock);
    memset(mCallbackList, 0, sizeof(mCallbackList));
    pthread_mutex_unlock(&mLock);

    pthread_mutex_destroy(&mLock);
    CDBG("%s: X", __func__);
}

void GestureNotifier::registerCallbacks(
        gesture_notify_callback notify_cb,
        gesture_data_callback data_cb,
        void *user)
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    int i = 0;
    for (i = 0; i < MAX_CALLBACK_NUM; i++) {
        if (false == mCallbackList[i].used) {
            mCallbackList[i].data_cb = data_cb;
            mCallbackList[i].notify_cb = notify_cb;
            mCallbackList[i].user = user;
            mCallbackList[i].used = true;
            break;
        }
    }
    if (MAX_CALLBACK_NUM == i) {
        CDBG_ERROR("%s: CallBack registeration exceeds max num for user(%p)", __func__, user);
    }
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
}

void GestureNotifier::unregisterCallbacks(void *user)
{
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    for (int i = 0; i < MAX_CALLBACK_NUM; i++) {
        if (mCallbackList[i].user == user) {
            memset(&mCallbackList[i], 0, sizeof(gesture_notifier_callback_t));
            break;
        }
    }
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
}

void GestureNotifier::broadcastData(gesture_result_t* gs_results)
{
    CDBG("%s: E", __func__);
    if (NULL == gs_results) {
        CDBG("%s: Gesture result is NULL, no need to broadcast", __func__);
        return;
    }

    DataNode* pNode = (DataNode*)malloc(sizeof(DataNode));
    if (NULL != pNode) {
        memset(pNode, 0, sizeof(DataNode));
        pNode->gs_results = gs_results;
        if (!mDataQueue.enqueue(pNode)) {
            DestroyGestureResult(&gs_results);
            free(pNode);
            pNode = NULL;
        }
    } else {
        DestroyGestureResult(&gs_results);
    }
    CDBG("%s: X", __func__);
}

void GestureNotifier::broadcastNotify(int32_t msg_type, int32_t ext1, int32_t ext2)
{
    CDBG("%s: E", __func__);
    NotifyNode* pNode = (NotifyNode*)malloc(sizeof(NotifyNode));
    if (NULL != pNode) {
        memset(pNode, 0, sizeof(NotifyNode));
        pNode->msg_type = msg_type;
        pNode->ext1 = ext1;
        pNode->ext2 = ext2;
        if (!mNotifyQueue.enqueue(pNode)) {
            free(pNode);
            pNode = NULL;
        }
    }
    CDBG("%s: X", __func__);
}

void GestureNotifier::broadcastDataInternal(gesture_result_t* gs_results)
{
    pthread_mutex_lock(&mLock);
    for (size_t i = 0; i < MAX_CALLBACK_NUM; i++) {
        if (mCallbackList[i].used &&
            (NULL != mCallbackList[i].data_cb) && 
            (NULL != mCallbackList[i].user)) {
            mCallbackList[i].data_cb(gs_results, mCallbackList[i].user);
        }
    }
    pthread_mutex_unlock(&mLock);
}

void GestureNotifier::broadcastNotifyInternal(int32_t msg_type, int32_t ext1, int32_t ext2)
{
    pthread_mutex_lock(&mLock);
    for (size_t i = 0; i < MAX_CALLBACK_NUM; i++) {
        if (mCallbackList[i].used &&
            (NULL != mCallbackList[i].notify_cb) &&
            (NULL != mCallbackList[i].user)) {
            mCallbackList[i].notify_cb(msg_type, ext1, ext2, mCallbackList[i].user);
        }
    }
    pthread_mutex_unlock(&mLock);
}

void* GestureNotifier::StartDataCBThread(void* pUsrData) {
    CDBG("%s: DataCB thread starting", __func__);
    GestureNotifier* pMe = (GestureNotifier*)pUsrData;
    void* pNode = NULL;
    while (NULL != (pNode = pMe->mDataQueue.dequeue())) {
        DataNode* pData = (DataNode*)pNode;
        pMe->broadcastDataInternal(pData->gs_results);
        DestroyGestureResult(&pData->gs_results);
        free(pData);
    }
    CDBG("%s: DataCB thread terminated", __func__);
    return NULL;
}

void* GestureNotifier::StartNotifyCBThread(void* pUsrData) {
    CDBG("%s: NotifyCB thread starting", __func__);
    GestureNotifier* pMe = (GestureNotifier*)pUsrData;
    void* pNode = NULL;
    while (NULL != (pNode = pMe->mNotifyQueue.dequeue())) {
        NotifyNode* pNofity = (NotifyNode*)pNode;
        pMe->broadcastNotifyInternal(pNofity->msg_type, pNofity->ext1, pNofity->ext2);
        free(pNofity);
    }
    CDBG("%s: NotifyCB thread terminated", __func__);
    return NULL;
}

NotifierQueue::NotifierQueue()
                       : mInitialized(false),
                         mMaxQueueSize(-1),
                         m_qType(GESTURE_NODE_TYPE_DATA),
                         m_head(NULL),
                         m_tail(NULL),
                         m_qSize(0)
{
    pthread_mutex_init(&mQueueLock, NULL);
    pthread_cond_init(&mQueueWait, NULL);
}

NotifierQueue::~NotifierQueue(){
    flush();
    pthread_mutex_destroy(&mQueueLock);
    pthread_cond_destroy(&mQueueWait);
}

void NotifierQueue::init(int maxQueueSize, gesture_node_type_t type){
    pthread_mutex_lock(&mQueueLock);
    mMaxQueueSize = maxQueueSize;
    m_qType = type;
    mInitialized = true;
    pthread_cond_signal(&mQueueWait);
    pthread_mutex_unlock(&mQueueLock);
}

void NotifierQueue::deinit(){
    pthread_mutex_lock(&mQueueLock);
    mInitialized = false;
    pthread_cond_signal(&mQueueWait);
    pthread_mutex_unlock(&mQueueLock);
}

bool NotifierQueue::isInitialized(){
    return mInitialized;
}

void NotifierQueue::enqueueInternal(QNode* node)
{
    node->next = NULL;
    if (NULL == m_head) {
        m_head = node;
        m_tail = node;
    } else {
        m_tail->next = node;
        m_tail = node;
    }
    m_qSize++;
}

QNode* NotifierQueue::dequeueInternal()
{
    QNode* node = m_head;
    if (NULL != m_head) {
        m_head = m_head->next;
        if (NULL == m_head) {
            m_tail = NULL;
        }
        m_qSize--;
    }
    return node;
}

bool NotifierQueue::enqueue(
                 void * element){
    pthread_mutex_lock(&mQueueLock);
    if(mInitialized == false) {
        pthread_mutex_unlock(&mQueueLock);
        return false;
    }

    if ((-1 != mMaxQueueSize) && (m_qSize >= mMaxQueueSize)) {
        // reach the max, remove the oldes item
        QNode* pNode = dequeueInternal();
        if (NULL != pNode) {
            if (GESTURE_NODE_TYPE_DATA == m_qType) {
                DataNode* pData = (DataNode*)pNode->data;
                DestroyGestureResult(&pData->gs_results);
            }
            free(pNode);
        }
    }

    bool ret = true;
    QNode* node = (QNode *)malloc(sizeof(QNode));
    if (NULL != node) {
        memset(node, 0, sizeof(QNode));
        node->data = element;
        enqueueInternal(node);
        pthread_cond_signal(&mQueueWait);
    } else {
        CDBG_ERROR("%s: No memory for QNode", __func__);
        ret = false;
    }
    pthread_mutex_unlock(&mQueueLock);

    return ret;
}

void* NotifierQueue::dequeue(){
    void* element = NULL;
    pthread_mutex_lock(&mQueueLock);
    while(mInitialized && (0 == m_qSize)){
        pthread_cond_wait(&mQueueWait, &mQueueLock);
    }

    if(!mInitialized){
        pthread_mutex_unlock(&mQueueLock);
        return NULL;
    }

    QNode* pNode = dequeueInternal();
    if (NULL != pNode) {
        element = pNode->data;
        free(pNode);
    }
    pthread_mutex_unlock(&mQueueLock);
    return element;
}

void NotifierQueue::flush(){
    pthread_mutex_lock(&mQueueLock);
    if(NULL != m_head)
        return;

    while(NULL != m_head) {
        QNode* delNode = m_head;
        m_head = m_head->next;

        if (GESTURE_NODE_TYPE_DATA == m_qType) {
            DataNode* pData = (DataNode*)delNode->data;
            DestroyGestureResult(&pData->gs_results);
        }
        free(delNode);
    }
    m_tail = NULL;
    m_qSize = 0;
    pthread_mutex_unlock(&mQueueLock);
}

}; // namespace android
