/*
 * Copyright (C) 2016 The Android Open Source Project
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

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/select.h>
#include <unistd.h>

#include <cutils/log.h>
#include <cutils/properties.h>
#include <utils/Timers.h>

#include "nanohub.h"
#include "sensType.h"
#include "eventnums.h"
#include "nanohub_sensors.h"
#include "nanohubPacket.h"

#define LOG_TAG "NANOHUB"

/*****************************************************************************/
static int min(int a, int b) {
    return (a < b) ? a : b;
}

static int handle_to_sensor_type(int handle)
{
    switch (handle) {
        case NANOHUB_ACCEL:
            return SENSOR_TYPE_ACCELEROMETER;
		case NANOHUB_MAG:
        	return SENSOR_TYPE_MAGNETIC_FIELD;
		case NANOHUB_GYRO:
			return SENSOR_TYPE_GYROSCOPE;
		case NANOHUB_ORIEN:
            return SENSOR_TYPE_ORIENTATION;
		case NANOHUB_RV:
            return SENSOR_TYPE_ROTATION_VECTOR;
		case NANOHUB_LA:
            return SENSOR_TYPE_LINEAR_ACCELERATION;
		case NANOHUB_GRAV:
			return SENSOR_TYPE_GRAVITY;
		case NANOHUB_GAMERV:
			return SENSOR_TYPE_GAME_ROTATION_VECTOR;
		case NANOHUB_GEORV:
			return SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR;
		case NANOHUB_SD:
			return SENSOR_TYPE_STEP_DETECTOR;
		case NANOHUB_SC:
			return SENSOR_TYPE_STEP_COUNTER;
        default:
        	return -1;
    }
}

static int handle_to_nanohub_type(int handle)
{
    switch (handle) {
        case NANOHUB_ACCEL:
            return SENS_TYPE_ACCEL;
		case NANOHUB_MAG:
			return SENS_TYPE_MAG;
		case NANOHUB_GYRO:
        	return SENS_TYPE_GYRO;
		case NANOHUB_ORIEN:
        	return SENS_TYPE_ORIENTATION;
		case NANOHUB_RV:
            return SENS_TYPE_ROTATION_VECTOR;
		case NANOHUB_LA:
            return SENS_TYPE_LINEAR_ACCEL;
		case NANOHUB_GRAV:
			return SENS_TYPE_GRAVITY;
		case NANOHUB_GAMERV:
			return SENS_TYPE_GAME_ROT_VECTOR;
		case NANOHUB_GEORV:
			return SENS_TYPE_GEO_MAG_ROT_VEC;
		case NANOHUB_SD:
			return SENS_TYPE_STEP_DETECT;
		case NANOHUB_SC:
			return SENS_TYPE_STEP_COUNT;
        default:
        	return -1;
    }
}

static int nanohub_type_to_handle(int handle)
{
    switch (handle) {
        case SENS_TYPE_ACCEL:
            return NANOHUB_ACCEL;
		case SENS_TYPE_MAG:
			return NANOHUB_MAG;
		case SENS_TYPE_GYRO:
        	return NANOHUB_GYRO;
        case SENS_TYPE_ORIENTATION:
		    return NANOHUB_ORIEN;
        case SENS_TYPE_ROTATION_VECTOR:
			return NANOHUB_RV;
        case SENS_TYPE_LINEAR_ACCEL:
			return NANOHUB_LA;
		case SENS_TYPE_GRAVITY:
			return NANOHUB_GRAV;
		case SENS_TYPE_GAME_ROT_VECTOR:
			return NANOHUB_GAMERV;
		case SENS_TYPE_GEO_MAG_ROT_VEC:
			return NANOHUB_GEORV;
		case SENS_TYPE_STEP_DETECT:
			return NANOHUB_SD;
		case SENS_TYPE_STEP_COUNT:
			return NANOHUB_SC;
        default:
        	return -1;
    }
}

/*
 * Constructor.
 *
 * Setup and open the ring buffer.
 */
NanoHub::NanoHub()
{
    const char *nanohub_path = "/dev/nanohub";

    mDataFd = open(nanohub_path, O_RDWR);
    if (mDataFd < 0) {
        ALOGE("open file '%s' failed: %s\n", nanohub_path, strerror(errno));
    }
    memset(mSensorConfig, 0, sizeof(struct sensor_config) * NANOHUB_ID_MAX);
}

NanoHub::~NanoHub()
{
    /* Silence all the sensors, so that we can stop the buffer */
    for (size_t i = 0 ; i < NANOHUB_ID_MAX ; i++) {
        activate(i, 0);
    }
    close(mDataFd);
}

/*
 * getFd: retrieve the ring file descriptor.
 *
 * Needed for CrosECSensor creator to listen to the buffer.
 */
int NanoHub::getFd(void)
{
    return mDataFd;
}

int NanoHub::flush(int handle)
{
    int err;
    int sensor_handle = handle_to_sensor_type(handle);
    struct sensor_config *config;

    if (sensor_handle < 0) {
        return -1;
    }

    config = &mSensorConfig[sensor_handle];
    config->evtType = EVT_NO_SENSOR_CONFIG_EVENT;
    config->sensorType = handle_to_nanohub_type(handle);
    config->calibrate = 0;
    config->reserved = 0;
    config->flush = 1;


    ALOGE("Flush Handle:%d", handle);

    err = write(mDataFd, config, sizeof(struct sensor_config));
    if (err < 0) {
        ALOGE("Write flush error");
        return -1;
    }

    return 0;
}

int NanoHub::activate(int handle, int enabled)
{
    int err;
    int sensor_handle = handle_to_sensor_type(handle);
    struct sensor_config *config;

    if (sensor_handle < 0) {
        return -1;
    }

    config = &mSensorConfig[sensor_handle];
    config->evtType = EVT_NO_SENSOR_CONFIG_EVENT;
    config->sensorType = handle_to_nanohub_type(handle);
    config->enable = enabled;
    config->reserved = 0;
    config->calibrate = 0;
    config->flush = 0;

    err = write(mDataFd, config, sizeof(struct sensor_config));
    if (err < 0) {
        ALOGE("set active error:%d",err);
        return -1;
    }

    return 0;
}

int NanoHub::batch(int handle, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
    int err;
    int sensor_handle = handle_to_sensor_type(handle);
    struct sensor_config *config;

    if (sensor_handle < 0) {
        return -1;
    }

    config = &mSensorConfig[sensor_handle];
    config->evtType = EVT_NO_SENSOR_CONFIG_EVENT;
    config->sensorType = handle_to_nanohub_type(handle);
    config->reserved = 0;
    config->calibrate = 0;
    config->flush = 0;
    config->rate = SENSOR_HZ(1000000000/sampling_period_ns);
    config->latency = max_report_latency_ns;

    err = write(mDataFd, config, sizeof(struct sensor_config));
    if (err < 0) {
        ALOGE("Set batch err:%d",err);
        return -1;
    }

    return 0;
}

int NanoHub::processEvent(sensors_event_t* data, const struct NanohubReadEventResponse *event)
{
    int i;
    uint64_t lastTime;
    int nanohub_type;
    int sensor_id;
    int sensor_type;
    struct EvtPacket *eventPacket = (struct EvtPacket *)event;
    struct TripleAxisDataEvent *tri_event =(struct TripleAxisDataEvent *)&eventPacket->referenceTime;

    nanohub_type = 0x0ff & eventPacket->sensType;

    sensor_id = nanohub_type_to_handle(nanohub_type);
    sensor_type = handle_to_sensor_type(sensor_id);

    for (i = 0; i < tri_event->samples[0].firstSample.numSamples; i++) {

        if (i == 0) {
            lastTime = eventPacket->referenceTime;
        } else {
            lastTime += tri_event->samples[i].deltaTime;
        }

        data->timestamp = lastTime;

        data->version = sizeof(sensors_event_t);
        data->sensor = sensor_id;
        data->type = sensor_type;
        data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
        data->acceleration.x = tri_event->samples[i].x;
        data->acceleration.y = tri_event->samples[i].y;
        data->acceleration.z = tri_event->samples[i].z;
        data++;
    }

    for (i = 0; i < tri_event->samples[0].firstSample.numFlushes; i++) {
        data->version = META_DATA_VERSION;
        data->sensor = 0;
        data->type = SENSOR_TYPE_META_DATA;
        data->reserved0 = 0;
        data->timestamp = 0;
        data->meta_data.what = META_DATA_FLUSH_COMPLETE;
        data->meta_data.sensor = sensor_id;
        data++;
    }

    return tri_event->samples[0].firstSample.numSamples + tri_event->samples[0].firstSample.numFlushes;
}

int NanoHub::readEvents(sensors_event_t* data, int count)
{
    int rc;

    if (count < 1) {
        return -EINVAL;
    }

    rc = read(mDataFd, &mEvents, sizeof(struct NanohubReadEventResponse));
    if (rc < 0) {
        ALOGE("rc %d while reading ring\n", rc);
        return rc;
    }

    rc = processEvent(data, &mEvents);

    return rc;
}
