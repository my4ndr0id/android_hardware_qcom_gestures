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

#define LOG_DEBUG
#define _ANDROID_

#include <pthread.h>

#ifndef ANDROID_HARDWARE_GESTURES_COMMON_H
#define ANDROID_HARDWARE_GESTURES_COMMON_H

#define GESTURE_DEVICES_MAX 2
#define GESTURE_DEV_NAME_LEN 32

/*
 * Gesture error return
 */
typedef enum 
{
    GESTURE_STATUS_NO_ERROR,
    GESTURE_STATUS_E_GENERAL,
    GESTURE_STATUS_E_NO_MEMORY,
    GESTURE_STATUS_E_NOT_SUPPORTED,
    GESTURE_STATUS_E_INVALID_INPUT,
    GESTURE_STATUS_E_INVALID_OPERATION,
} gesture_status_t;

#undef CDBG
#ifndef LOG_DEBUG
  #ifdef _ANDROID_
    #undef LOG_NIDEBUG
    #undef LOG_TAG
    #define LOG_NIDEBUG 0
    #define LOG_TAG "gestureHAL"
    #include <utils/Log.h>
  #else
    #include <stdio.h>
  #endif
  #define CDBG(fmt, args...) do{}while(0)
#else
  #ifdef _ANDROID_
    #undef LOG_NIDEBUG
    #undef LOG_TAG
    #define LOG_NIDEBUG 0
    #define LOG_TAG "gestureHAL"
    #include <utils/Log.h>
    #define CDBG(fmt, args...) LOGI(fmt, ##args)
  #else
    #include <stdio.h>
    #define CDBG(fmt, args...) fprintf(stderr, fmt, ##args)
  #endif
#endif

#ifdef _ANDROID_
  #define CDBG_HIGH(fmt, args...)  LOGE(fmt, ##args)
  #define CDBG_ERROR(fmt, args...)  LOGE(fmt, ##args)
#else
  #define CDBG_HIGH(fmt, args...) fprintf(stderr, fmt, ##args)
  #define CDBG_ERROR(fmt, args...) fprintf(stderr, fmt, ##args)
#endif

#endif
