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

#ifndef ANDROID_HARDWARE_QUALCOMM_GESTURES_H
#define ANDROID_HARDWARE_QUALCOMM_GESTURES_H

extern "C" {

int get_number_of_gesture_devices();

int open_gesture_device(const struct hw_module_t* module, const char* id,
      struct hw_device_t** device);

int close_gesture_device(hw_device_t *);

namespace android {
    void set_gesture_CallBacks(struct gesture_device *,
                               gesture_notify_callback notify_cb,
                               gesture_data_callback data_cb,
                               void *user,
                               bool isreg);
    
    int start_gesture(struct gesture_device *);
    
    void stop_gesture(struct gesture_device *);
    
    int set_gesture_parameters(struct gesture_device * device, const char *parms);

    char* get_gesture_parameters(struct gesture_device * device);

    int send_gesture_command(struct gesture_device *,
              int32_t cmd, int32_t arg1, int32_t arg2);
    
    int gesture_dump(struct gesture_device *, int fd);
    
    }; // namespace android

} //extern "C"

#endif

