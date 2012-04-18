/*
 * Copyright (c) 2012 Code Aurora Forum. All rights reserved.  
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_HARDWARE_GESTURE_PARAMETERS_H
#define ANDROID_HARDWARE_GESTURE_PARAMETERS_H

namespace android {

#define MAX_PARAM_KEY_LEN 64
#define DEFAULT_PARAM_VALUE_LEN 16
#define RANGE_PARAM_VALUE_LEN 256

typedef enum {
    GESTURE_PARAM_ENUM_MODE,
    GESTURE_PARAM_ENUM_SUBMODE,
    GESTURE_PARAM_ENUM_ENABLE_TOUCH,
    GESTURE_PARAM_ENUM_ORIENTATION,
    GESTURE_PARAM_ENUM_COORDINATE_MODE,
    GESTURE_PARAM_ENUM_CURSOR_TYPE,
    GESTURE_PARAM_ENUM_CLICK_MODE,
    GESTURE_PARAM_ENUM_CAMERA_INPUT,
    GESTURE_PARAM_ENUM_EXTENDED_CONFIG,
    GESTURE_PARAM_ENUM_COORDINATE_RANGE,
    GESTURE_PARAM_ENUM_MAX
} param_enum_t;

class GestureParameters
{
public:
    GestureParameters();
    GestureParameters(const char* params);
    ~GestureParameters();

    char* flatten();
    void unflatten(const char* params);

    /**
     * Gets the current Gesture Mode.
     *
     * @return int value of Gesture Mode.
     */
    int getGestureMode();

    /**
     * Sets the current Gesture Mode.
     *
     * @return null
     */
    void setGestureMode(int gsMode);

    /**
     * Gets the current Gesture Sub Mode.
     *
     * @return int value of Gesture Sub Mode.
     */
    int getGestureSubMode();

    /**
     * Sets the current Gesture Sub Mode.
     *
     * @return null
     */
    void setGestureSubMode(int subMode);

    /**
     * Gets the flag if touch is enabled.
     *
     * @return true/false.
     */
    bool getTouchEnabled();

    /**
     * Sets the flag if touch is enabled.
     *
     * @return null
     */
    void setTouchEnabled(bool enabled);

    /**
     * Gets the current Gesture Orientation.
     *
     * @return int value of Gesture Orientation.
     */
    int getOrientation();

    /**
     * Sets the current Gesture Orientation.
     *
     * @return null
     */
    void setOrientation(int gsOrientation);

    /**
     * Gets the current Coordinate Mode.
     *
     * @return int value of coordinate Mode.
     */
    int getCoordinateMode();

    /**
     * Sets the current Coordinate Mode.
     *
     * @return null
     */
    void setCoordinateMode(int coordMode);

    /**
     * Gets the current Cursor Type.
     *
     * @return int value of cursor type.
     */
    int getCursorType();

    /**
     * Sets the current Cursor Type.
     *
     * @return null
     */
    void setCursorType(int cursor);

    /**
     * Gets the current Click Mode.
     *
     * @return int value of click mode.
     */
    int getClickMode();

    /**
     * Sets the current Click Mode.
     *
     * @return null
     */
    void setClickMode(int clickMode);

    /**
     * Sets the camera input device id.
     *
     * @return null
     */
    void setCameraInput(int camId);

    /**
     * Gets the current camera input device id.
     *
     * @return int value of camera id.
     */
    int getCameraInput();

    /**
     * Sets the extended config field.
     *
     * @param ext   String of extended configuration, cannot contain 
     *              '=' and ';' that dedicate to parameter key/value
     *              format
     * @return null
     */
    void setExtendedConfig(char* ext);

    /**
     * Gets the extended config field.
     *
     * @return String value of extended config.
     */
    char* getExtendedConfig();

    /**
     * Gets the coordinate ranges. Each range
     * contains a minimum and maximum coordinate.
     *
     * @return true/false.
     */
    bool getCoordinateRange(float range[6]);

    /**
     * Sets the coordinate ranges.
     *
     * @return null
     */
    void setCoordinateRange(
                        float x_min, float x_max,
                        float y_min, float y_max,
                        float z_min, float z_max);

private:
    // Parameter keys to communicate between gesture application and driver.
    // The access (read/write, read only, or write only) is viewed from the
    // perspective of applications, not driver.
    static const char* PARAM_KEYS[GESTURE_PARAM_ENUM_MAX];
    char* mValues[GESTURE_PARAM_ENUM_MAX];

    void clearValues();
    int getKeyIndex(const char* key);
    int calcBufferSize();

    void set(int keyIdx, char *value);
    void setInt(int keyIdx, int value);
    void setFloat(int keyIdx, float value);
    void setBoolean(int keyIdx, bool value);
    char *get(int keyIdx);
    int getInt(int keyIdx);
    float getFloat(int keyIdx);
    bool getBoolean(int keyIdx);
    void remove(int keyIdx);

};

}; // namespace android

#endif
