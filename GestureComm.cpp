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

#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <linux/videodev2.h>
#include <linux/media.h>
#include <linux/ion.h>
#include <media/msm_gestures.h>
extern "C" {
#include "mm_sock.h"
}
#include "GesturesMgr.h"
#include "GestureComm.h"

namespace android {

typedef struct {
    uint8_t cmd;
} gesture_sig_evt_t;

#define GESTURE_DEV_OPEN_TRIES 2
#define GESTURE_DEV_OPEN_RETRY_SLEEP 20

#define STR_GESTURE "gesture"
#define STR_NODE "/dev/v4l-subdev%d"

typedef enum {
    /* exit */
    GESTURE_PIPE_CMD_EXIT,
    /* max count */
    GESTURE_PIPE_CMD_MAX
} gesture_pipe_cmd_type_t;

typedef enum {
    GESTURE_POLL_TASK_STATE_POLL,     /* polling pid in polling state. */
    GESTURE_POLL_TASK_STATE_MAX
} gesture_poll_task_state_type_t;

GestureComm::GestureComm(int device_id, gesture_result_callback cb, void* user)
                :mDeviceId(device_id),
                 mCB(cb),
                 mUser(user),
                 m_isOpened(false),
                 mCtrlfd(-1),
                 mDSfd(-1),
                 mStatus(0),
                 mState(GESTURE_POLL_TASK_STATE_POLL)
{
}

GestureComm::~GestureComm() {
    Close();
}

void GestureComm::QueryGesturesDeviceInfo(
                    int8_t &numDevices, 
                    char deviceName[GESTURE_DEVICES_MAX][GESTURE_DEV_NAME_LEN]) 
{
    numDevices = 0;
    struct media_device_info mdev_info;
    int num_media_devices = 0;
	char dev_name[32];
	int rc = 0, dev_fd = 0;
	while (1) {
        snprintf(dev_name, sizeof(dev_name), "/dev/media%d", num_media_devices);
	    dev_fd = open(dev_name, O_RDWR | O_NONBLOCK);
	    if (dev_fd < 0) {
            CDBG("%s: Done discovering media devices\n", __func__);
            break;
	    }
	    num_media_devices++;
	    rc = ioctl(dev_fd, MEDIA_IOC_DEVICE_INFO, &mdev_info);
	    if (rc < 0) {
            CDBG_ERROR("Error: ioctl media_dev failed: %s\n", strerror(errno));
            close(dev_fd);
            break;
        }
	
        if (strncmp(mdev_info.model, "qcamera", sizeof(mdev_info.model) != 0)) {
            close(dev_fd);
            continue;
        }
	
        int num_entities = 1;
        while (1) {
            struct media_entity_desc entity;
            memset(&entity, 0, sizeof(entity));
            entity.id = num_entities++;
            rc = ioctl(dev_fd, MEDIA_IOC_ENUM_ENTITIES, &entity);
            if (rc < 0) {
                CDBG("Done enumerating media entities\n");
                rc = 0;
                break;
            }
            if (!strncmp(entity.name, STR_GESTURE, sizeof(entity.name)) &&
                 GESTURE_DEVICES_MAX > numDevices) {
                snprintf(deviceName[numDevices], GESTURE_DEV_NAME_LEN, STR_NODE, entity.revision);
                CDBG_ERROR("Device name: %s\n", deviceName[numDevices]);
                numDevices++;
            }
        }

        if (dev_fd > 0) {
            close(dev_fd);
        }
    }

    CDBG("%s: num_gesture_devices = %d\n", __func__, numDevices);
}

int GestureComm::Open(const char* dev_name) {
    char sock_path[GESTURE_DEV_NAME_LEN];
    int32_t rc = GESTURE_STATUS_NO_ERROR;
    int8_t n_try= GESTURE_DEV_OPEN_TRIES;
    uint8_t sleep_msec = GESTURE_DEV_OPEN_RETRY_SLEEP;
    uint8_t i;

    if (m_isOpened) {
        CDBG("%s: Already opened, return here", __func__);
        return rc;
    }

    pthread_mutex_init(&mLock, NULL);
    pthread_cond_init(&mWait, NULL);

    do{
        n_try--;
        mCtrlfd = open(dev_name, O_RDWR | O_NONBLOCK);
        if ((mCtrlfd > 0) || (errno != EIO) || (n_try <= 0 )) {
            CDBG_ERROR("%s:  opened (fd = %d), break out while loop", __func__, mCtrlfd);
            break;
        }
        CDBG("%s:failed with I/O error retrying after %d milli-seconds",
             __func__,sleep_msec);
        usleep(sleep_msec*1000);
    }while(n_try>0);

    if (mCtrlfd <= 0) {
        CDBG_ERROR("%s: cannot open control fd of '%s' Errno = %d\n",
                 __func__, dev_name, errno);
        return -GESTURE_STATUS_E_GENERAL;
    }

//Commented for now, to be removed once implemented
#if 0
    /* open domain socket*/
    n_try = GESTURE_DEV_OPEN_TRIES;
    snprintf(sock_path, sizeof(sock_path), "/data/gesture_socket%d", mDeviceId);
    do{
        n_try--;
        mDSfd = mm_socket_create(sock_path, MM_SOCK_TYPE_UDP);
        CDBG_ERROR("%s:  ds_fd = %d", __func__, mDSfd);
        CDBG_ERROR("Errno:%d",errno);
        if((mDSfd > 0) || (n_try <= 0 )) {
            CDBG_ERROR("%s:  opened, break out while loop", __func__);
            break;
        }
        CDBG("%s:failed with I/O error retrying after %d milli-seconds",
             __func__,sleep_msec);
        usleep(sleep_msec*1000);
    }while(n_try>0);

    CDBG_ERROR("%s:  after while loop for domain socket open", __func__);
    if (mDSfd <= 0) {
        CDBG_ERROR("%s: cannot open domain socket fd of '%s' Errno = %d\n",
                 __func__, sock_path, errno);
        return -GESTURE_STATUS_E_GENERAL;
    }
#endif

    rc = LaunchPollThread();
    CDBG("%s: '%s', ctrl_fd=%d, rc=%d\n",
             __func__, dev_name, mCtrlfd, rc);

    if (rc == GESTURE_STATUS_NO_ERROR) {
        rc = SubscribeEvt(true);
    }

    m_isOpened = true;
    return rc;
}

int GestureComm::SubscribeEvt(bool bReg) {
    int rc = GESTURE_STATUS_NO_ERROR;
    struct v4l2_event_subscription sub;

    memset(&sub, 0, sizeof(sub));
    sub.type = MSM_GES_APP_NOTIFY_EVENT;
    if (bReg) {
        /* subscribe */
        rc = ioctl(mCtrlfd, VIDIOC_SUBSCRIBE_EVENT, &sub);
        CDBG("%s: subscribe event 0x%x, rc = %d, errno = %d", __func__, sub.type, rc, errno);
    } else {
        /* unsubscribe */
        rc = ioctl(mCtrlfd, VIDIOC_UNSUBSCRIBE_EVENT, &sub);
        CDBG("%s: unsubscribe event 0x%x, rc = %d, errno = %d", __func__, sub.type, rc, errno);
    }
    return rc;
}

int GestureComm::PollSig(uint32_t cmd)
{
    CDBG("%s: E", __func__);
    /* send through pipe */
    /* get the mutex */
    gesture_sig_evt_t cmd_evt;
    memset(&cmd_evt, 0, sizeof(cmd_evt));
    cmd_evt.cmd = cmd;
    int len;
    pthread_mutex_lock(&mLock);
    /* reset the statue to 0 */
    mStatus = 0;
    /* send cmd to worker */
    len = write(mFDs[1], &cmd_evt, sizeof(cmd_evt));
    if(len < 1) {
      CDBG_ERROR("%s: len = %d, errno = %d", __func__, len, errno);
    }
    CDBG("%s: begin IN mutex write done, len = %d", __func__, len);
    /* wait till worker task gives positive signal */
    if(!mStatus) {
        CDBG("%s: wait", __func__);
        pthread_cond_wait(&mWait, &mLock);
    }
    /* done */
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
    return GESTURE_STATUS_NO_ERROR;
}

void GestureComm::PollSigDone() {
    CDBG("%s: E", __func__);
    pthread_mutex_lock(&mLock);
    mStatus = 1;
    pthread_cond_signal(&mWait);
    pthread_mutex_unlock(&mLock);
    CDBG("%s: X", __func__);
}

void* GestureComm::PollFunction() {
    int rc = 0, i;
    struct pollfd fds[3];
    int timeoutms;

    int num_fds = 0;
    fds[num_fds].fd = mFDs[0];
    fds[num_fds].events = POLLIN|POLLRDNORM;
    num_fds++;
    fds[num_fds].fd = mCtrlfd;
    fds[num_fds].events = POLLPRI;
    num_fds++;
    fds[num_fds].fd = mDSfd;
    fds[num_fds].events = POLLIN|POLLRDNORM;
    num_fds++;

    do {
        timeoutms = mTimeoutms;
        rc = poll(fds, num_fds, timeoutms);
        if(rc > 0) {
            if ((fds[0].revents & POLLIN) || (fds[0].revents & POLLRDNORM)) {
                ProcPipe(fds[0].fd);
            }

            if (fds[1].revents & POLLPRI) {
                ProcEvt(fds[1].fd);
            }

            /* Events on user created socket */
            if ((fds[2].revents & POLLIN) || (fds[2].revents & POLLRDNORM)) {
                ProcSocket(fds[2].fd);
            }

        } else {
            /* in error case sleep 10 us and then continue. hard coded here */
            usleep(10);
            continue;
        }
    } while (mState == GESTURE_POLL_TASK_STATE_POLL);
    CDBG("%s: X", __func__);
    return NULL;
}

void* GestureComm::StartPollThread(void* pUsrData) {
    GestureComm *pMe = (GestureComm*)pUsrData;
    pMe->PollSigDone();
    return pMe->PollFunction();
}

int GestureComm::LaunchPollThread() {
    int rc = GESTURE_STATUS_NO_ERROR;

    rc = pipe(mFDs);
    if(rc < 0) {
        CDBG_ERROR("%s: pipe open rc=%d\n", __func__, rc);
        rc = - GESTURE_STATUS_E_GENERAL;
    }
    CDBG_ERROR("%s: read fd = %d, write fd = %d",
        __func__, mFDs[0], mFDs[1]);
    mTimeoutms = 3000;

    /* launch the thread */
    pthread_mutex_lock(&mLock);
    mStatus = 0;

    pthread_create(&mPID, NULL, StartPollThread, (void *)this);

    if(!mStatus) {
        pthread_cond_wait(&mWait, &mLock);
    }
    pthread_mutex_unlock(&mLock);
    return rc;
}

void GestureComm::ProcPipe(int32_t fd)
{
    int32_t read_len;
    int i;
    gesture_sig_evt_t cmd_evt;
    memset(&cmd_evt, 0, sizeof(cmd_evt));
    read_len = read(fd, &cmd_evt, sizeof(cmd_evt));
    CDBG("%s: read_fd = %d, read_len = %d, expect_len = %d",
         __func__, fd, (int)read_len, (int)sizeof(cmd_evt));
    if ((read_len > 0) && (cmd_evt.cmd == GESTURE_PIPE_CMD_EXIT)) {
        mState = GESTURE_POLL_TASK_STATE_MAX;
        PollSigDone();
   }
}

static gs_ctl_result_t* generateTestGsResult()
{
    int num_of_events = 2;
    int event_size = sizeof(gs_ctl_result_t) + (num_of_events-1) * sizeof(gs_ctl_result_t);
    gs_ctl_result_t *evt = (gs_ctl_result_t*)malloc(event_size);
    if (NULL == evt) {
        return NULL;
    }

    memset(evt, 0, event_size);
    evt->number_of_outcomes = num_of_events;

    for (int i = 0; i < evt->number_of_outcomes; i++) {
        evt->outcomes[i].version = 0;
        evt->outcomes[i].type = 200+i;
        evt->outcomes[i].subtype = 301;
        evt->outcomes[i].id = i;
        evt->outcomes[i].timestamp = 12345*i;
        evt->outcomes[i].confidence = 0.9;
        evt->outcomes[i].velocity = 0.0;

        evt->outcomes[i].location.begin.x = 5;
        evt->outcomes[i].location.begin.y = 5;
        evt->outcomes[i].location.begin.z = 5;
        evt->outcomes[i].location.begin.error = 0;
        evt->outcomes[i].location.end.x = 75;
        evt->outcomes[i].location.end.y = 75;
        evt->outcomes[i].location.end.z = 75;
        evt->outcomes[i].location.end.error = 0;
    }

    return evt;
}     

void GestureComm::ProcEvt(int32_t fd)
{
    struct v4l2_event ev;
    memset(&ev, 0, sizeof(ev));
    int rc = ioctl(fd, VIDIOC_DQEVENT, &ev);
    if (rc >=  0) {
        struct msm_ges_evt ges_evt;
        gs_ctl_result_t evt;
        ges_evt.evt_data = &evt;
        ges_evt.evt_len = sizeof(gs_ctl_result_t);

        rc = ioctl(fd, MSM_GES_GET_EVT_PAYLOAD, &ges_evt);
        if (rc >= 0) {
            if (mCB) {
                mCB(GESTURE_EVT_TYPE_RESULT, ges_evt.evt_data, ges_evt.evt_len, mUser);
            }
        }
    }
}

void GestureComm::ProcSocket(int32_t fd)
{
#if 0
    /* BEGIN: Hardcoded here for test purpose */
    gs_ctl_result_t* evt = generateTestGsResult();
    if ((NULL != evt) && mCB) {
        mCB(GESTURE_EVT_TYPE_RESULT, (void*)evt, sizeof(gs_ctl_result_t), mUser);
    }
    if (NULL != evt) {
        free(evt);
    }
    /* END: Hardcoded here for test purpose */
#endif

    char msg[64];
    uint32_t buf_size = 64;
    int32_t rcvd_len = 0;
    rcvd_len = mm_socket_recvmsg(fd, msg, buf_size, NULL);
    if ((rcvd_len > 0) && mCB) {
        mCB(GESTURE_EVT_TYPE_RESULT, msg, rcvd_len, mUser); // could be other evt type from socket
    } 
}

int GestureComm::ReleasePollThread()
{
    CDBG("%s:  E", __func__);
    int rc = GESTURE_STATUS_NO_ERROR;
    if(GESTURE_POLL_TASK_STATE_MAX == mState) {
        CDBG("%s: err, poll thread is not running.\n", __func__);
        return -GESTURE_STATUS_E_INVALID_OPERATION;
    }

    PollSig(GESTURE_PIPE_CMD_EXIT);
    if (pthread_join(mPID, NULL) != 0) {
        CDBG("%s: pthread dead already\n", __func__);
    }

    if(mFDs[0]) {
        close(mFDs[0]);
        mFDs[0] = 0;
    }
    if(mFDs[1]) {
        close(mFDs[1]);
        mFDs[1] = 0;
    }

    CDBG("%s:  X", __func__);
    return GESTURE_STATUS_NO_ERROR;
}

int GestureComm::SendMsg(int type, void* msg, int32_t len)
{
    int rc = GESTURE_STATUS_NO_ERROR;

    struct msm_ctrl_cmd ctrl_cmd;
    memset(&ctrl_cmd, 0, sizeof(ctrl_cmd));
    ctrl_cmd.type = type;
    ctrl_cmd.length = (uint16_t)len;
    ctrl_cmd.timeout_ms = 5000;
    ctrl_cmd.value = msg;
    ctrl_cmd.status = 1;

    struct v4l2_control control;
    memset(&control, 0, sizeof(control));
    control.id = MSM_GESTURE_CID_CTRL_CMD;
    control.value = (int32_t)&ctrl_cmd;
    rc = ioctl (mCtrlfd, MSM_GES_IOCTL_CTRL_COMMAND, &control);
    CDBG("%s: type=%d, rc = %d\n", __func__, type, rc);
    return rc;
}

int GestureComm::Close() {
    CDBG("%s:  E", __func__);
    int rc = GESTURE_STATUS_NO_ERROR;

    if (!m_isOpened) {
        CDBG("%s: already closed", __func__);
        return rc;
    }

    mCB = NULL;
    mUser = NULL;

    SubscribeEvt(false);
    ReleasePollThread();

    if(mCtrlfd >= 0) {
        rc = close(mCtrlfd);
        if(rc != GESTURE_STATUS_NO_ERROR) {
            /* this is a dead end. */
            CDBG_ERROR("%s: !!!!FATAL ERROR!!!! ctrl_fd = %d, rc = %d, errno=%d",
                 __func__, mCtrlfd, rc, errno);
        }
        mCtrlfd = -1;
    }
    if(mDSfd >= 0) {
        mm_socket_close(mDSfd);
        mDSfd = -1;
    }

    pthread_mutex_destroy(&mLock);
    pthread_cond_destroy(&mWait);
    m_isOpened = false;
    CDBG("%s:  X", __func__);
    return rc;
}

}; // namespace android
