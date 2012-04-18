/*
**
** Copyright (c) 2012 Code Aurora Forum. All rights reserved.  
** Copyright (c) 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#include "common.h"
#include <string.h>
#include <stdlib.h>
#include <GestureParameters.h>

namespace android {
// Parameter keys to communicate between gesture device application and driver.
const char* GestureParameters::PARAM_KEYS[] = {
        "gs-ctrl-mode",
        "gs-ctrl-submode",
        "gs-ctrl-enable-touch",
        "gs-ctrl-orientation",
        "gs-ctrl-coord-mode",
        "gs-ctrl-cursor-type",
        "gs-ctrl-click-mode",
        "gs-ctrl-camera-input",
        "gs-ctrl-extended-config",
        "gs-ctrl-coord-range"
};

GestureParameters::GestureParameters()
{
    memset(&mValues, 0, sizeof(mValues));
}

GestureParameters::GestureParameters(const char* params)
{
    memset(&mValues, 0, sizeof(mValues));
    unflatten(params);
}

GestureParameters::~GestureParameters()
{
    clearValues();
}

void GestureParameters::clearValues()
{
    for (int i = 0; i < GESTURE_PARAM_ENUM_MAX; i++) {
        if (NULL != mValues[i]) {
            free(mValues[i]);
        }
    }
    memset(&mValues, 0, sizeof(mValues));
}

int GestureParameters::getKeyIndex(const char* key)
{
    if (NULL == key) {
        return -1;
    }
    for (int i = 0; i < GESTURE_PARAM_ENUM_MAX; i++) {
        if (strcmp(key, PARAM_KEYS[i]) == 0) {
            return i;
        }
    }
    return -1;
}

int GestureParameters::calcBufferSize()
{
/*
    GESTURE_PARAM_ENUM_MODE,
    GESTURE_PARAM_ENUM_SUBMODE,
    GESTURE_PARAM_ENUM_ENABLE_TOUCH,
    GESTURE_PARAM_ENUM_ORIENTATION,
    GESTURE_PARAM_ENUM_COORDINATE_MODE,
    GESTURE_PARAM_ENUM_CURSOR_TYPE,
    GESTURE_PARAM_ENUM_CLICK_MODE,
    GESTURE_PARAM_ENUM_CAMERA_INPUT,
    GESTURE_PARAM_ENUM_EXTENDED_CONFIG,
    GESTURE_PARAM_ENUM_COORDINATE_RANGE
*/
    int size = (MAX_PARAM_KEY_LEN + DEFAULT_PARAM_VALUE_LEN) * GESTURE_PARAM_ENUM_MAX;
    if (NULL != mValues[GESTURE_PARAM_ENUM_EXTENDED_CONFIG]) {
        size += strlen(mValues[GESTURE_PARAM_ENUM_EXTENDED_CONFIG]) + 1;
    }
    return size;
}

char* GestureParameters::flatten()
{
    int buf_size = calcBufferSize();
    char* flattened = (char *)malloc(buf_size);
    if (NULL != flattened) {
        memset(flattened, 0, buf_size);
        char* pos = flattened;
        int count = 0;
        for (int i = GESTURE_PARAM_ENUM_MODE; i < GESTURE_PARAM_ENUM_MAX; i++) {
            if (NULL != mValues[i]) {
                sprintf(pos, "%s=%s;", PARAM_KEYS[i], mValues[i]);
                count++;
                pos = flattened + strlen(flattened);
            }
        }
        if (count > 0) {
            flattened[strlen(flattened)] = '\0';
        }
    } else {
        CDBG_ERROR("%s: No memory for parameters", __func__);
    }
    return flattened;
}

void GestureParameters::unflatten(const char* params)
{
    const char *a = params;
    const char *b;
    char key[MAX_PARAM_KEY_LEN];
    char *value = NULL;

    clearValues();

    for (;;) {
        // Find the bounds of the key name.
        b = strchr(a, '=');
        if (b == 0)
            break;

        // Create the key string.
        memset(key, 0, sizeof(key));
        memcpy(key, a, (b-a));

        // Find the value.
        a = b+1;
        b = strchr(a, ';');
        if (b == 0) {
            // If there's no semicolon, this is the last item.
            value = strdup(a);
            if (NULL != value) {
                int idx = getKeyIndex(key);
                if (idx >= GESTURE_PARAM_ENUM_MODE && idx < GESTURE_PARAM_ENUM_MAX) {
                    mValues[idx] = value;
                } else {
                    free(value);
                    value = NULL;
                }
            }
            break;
        }

        value = (char*)malloc(b-a+1);
        if (NULL != value) {
            memset(value, 0, b-a+1);
            memcpy(value, a, b-a);
            int idx = getKeyIndex(key);
            if (idx >= GESTURE_PARAM_ENUM_MODE && idx < GESTURE_PARAM_ENUM_MAX) {
                mValues[idx] = value;
            } else {
                free(value);
                value = NULL;
            }
        }
        a = b+1;
    }
}

void GestureParameters::set(int keyIdx, char *value)
{
    if (strchr(value, '=') || strchr(value, ';')) {
        //XXX LOGE("Value \"%s\"contains invalid character (= or ;)", value);
        return;
    }

    if (NULL != mValues[keyIdx]) {
        free(mValues[keyIdx]);
    }
    mValues[keyIdx] = value;
}

void GestureParameters::setInt(int keyIdx, int value)
{
    char* str = (char *)malloc(DEFAULT_PARAM_VALUE_LEN);
    if (NULL != str) {
        snprintf(str, DEFAULT_PARAM_VALUE_LEN, "%d", value);
        set(keyIdx, str);
    }
}

void GestureParameters::setFloat(int keyIdx, float value)
{
    char* str = (char *)malloc(DEFAULT_PARAM_VALUE_LEN);
    if (NULL != str) {
        snprintf(str, DEFAULT_PARAM_VALUE_LEN, "%g", value);
        set(keyIdx, str);
    }
}

void GestureParameters::setBoolean(int keyIdx, bool value)
{
    char* str = (char *)malloc(DEFAULT_PARAM_VALUE_LEN);
    if (NULL != str) {
        snprintf(str, DEFAULT_PARAM_VALUE_LEN, "%d", value);
        set(keyIdx, str);
    }
}

char *GestureParameters::get(int keyIdx)
{
    return mValues[keyIdx];
}

int GestureParameters::getInt(int keyIdx)
{
    const char *v = get(keyIdx);
    if (v == 0)
        return -1;
    return strtol(v, 0, 0);
}

float GestureParameters::getFloat(int keyIdx)
{
    const char *v = get(keyIdx);
    if (v == 0) return -1;
    return strtof(v, 0);
}

bool GestureParameters::getBoolean(int keyIdx) {
    int val = getInt(keyIdx);
    if (val == 0) {
        return false;
    } else {
        return true;
    }
}

void GestureParameters::remove(int keyIdx)
{
    if (NULL != mValues[keyIdx]) {
        free(mValues[keyIdx]);
        mValues[keyIdx] = NULL;
    }
}

int GestureParameters::getGestureMode() {
    return getInt(GESTURE_PARAM_ENUM_MODE);
}

void GestureParameters::setGestureMode(int gsMode) {
    setInt(GESTURE_PARAM_ENUM_MODE, gsMode);
}

int GestureParameters::getGestureSubMode() {
    return getInt(GESTURE_PARAM_ENUM_SUBMODE);
}

void GestureParameters::setGestureSubMode(int subMode) {
    setInt(GESTURE_PARAM_ENUM_SUBMODE, subMode);
}

bool GestureParameters::getTouchEnabled() {
    return getBoolean(GESTURE_PARAM_ENUM_ENABLE_TOUCH);
}

void GestureParameters::setTouchEnabled(bool enabled) {
    setBoolean(GESTURE_PARAM_ENUM_ENABLE_TOUCH, enabled);
}

int GestureParameters::getOrientation() {
    return getInt(GESTURE_PARAM_ENUM_ORIENTATION);
}

void GestureParameters::setOrientation(int gsOrientation) {
    setInt(GESTURE_PARAM_ENUM_ORIENTATION, gsOrientation);
}

int GestureParameters::getCoordinateMode() {
    return getInt(GESTURE_PARAM_ENUM_COORDINATE_MODE);
}

void GestureParameters::setCoordinateMode(int coordMode) {
    setInt(GESTURE_PARAM_ENUM_COORDINATE_MODE, coordMode);
}

int GestureParameters::getCursorType() {
    return getInt(GESTURE_PARAM_ENUM_CURSOR_TYPE);
}

void GestureParameters::setCursorType(int cursor) {
    setInt(GESTURE_PARAM_ENUM_CURSOR_TYPE, cursor);
}

int GestureParameters::getClickMode() {
    return getInt(GESTURE_PARAM_ENUM_CLICK_MODE);
}

void GestureParameters::setClickMode(int clickMode) {
    setInt(GESTURE_PARAM_ENUM_CLICK_MODE, clickMode);
}

void GestureParameters::setCameraInput(int camId) {
    setInt(GESTURE_PARAM_ENUM_CAMERA_INPUT, camId);
}

int GestureParameters::getCameraInput() {
    return getInt(GESTURE_PARAM_ENUM_CAMERA_INPUT);
}

void GestureParameters::setExtendedConfig(char* ext) {
    set(GESTURE_PARAM_ENUM_EXTENDED_CONFIG, strdup(ext));
}

char* GestureParameters::getExtendedConfig() {
    return get(GESTURE_PARAM_ENUM_EXTENDED_CONFIG);
}

void GestureParameters::setCoordinateRange(
                        float x_min, float x_max,
                        float y_min, float y_max,
                        float z_min, float z_max) {
    char* str = (char *)malloc(RANGE_PARAM_VALUE_LEN);
    if (NULL != str) {
        snprintf(str, RANGE_PARAM_VALUE_LEN,
                 "(%g,%g),(%g,%g),(%g,%g)",
                 x_min, x_max,
                 y_min, y_max,
                 z_min, z_max);
        set(GESTURE_PARAM_ENUM_COORDINATE_RANGE, str);
        CDBG_ERROR("%s: %s", __func__, str);
    }
}

bool GestureParameters::getCoordinateRange(float range[6]) {
    char* str = get(GESTURE_PARAM_ENUM_COORDINATE_RANGE);
    if (NULL != str) {
        if (sscanf(str, "(%g,%g),(%g,%g),(%g,%g)",
               &range[0], &range[1], &range[2], &range[3], &range[4], &range[5]) < 6) {
            return false;
        }
        CDBG_ERROR("%s: %s", __func__, str);
        return true;
    } else {
        return false;
    }
}

}; // namespace android
