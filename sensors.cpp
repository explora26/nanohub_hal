/*
 * Copyright (C) 2008-2015 The Android Open Source Project
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

#define LOG_TAG "NanoHubSensor"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>

#include <utils/Atomic.h>
#include <utils/Log.h>

#include <hardware/sensors.h>

#include "nanohub.h"
#include "sensors.h"

/*****************************************************************************/

/*****************************************************************************/

#define CONVERT_A        0.01f
#define CONVERT_M        0.01f
#define CONVERT_GYRO     0.01f
#define RANGE_A                     (8*GRAVITY_EARTH)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

static const struct sensor_t sSensorList[] = {
    {.name =       "Accelerometer Sensor",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_ACCEL,
     .type =       SENSOR_TYPE_ACCELEROMETER,
     .maxRange =   RANGE_A,
     .resolution = CONVERT_A,
     .power =      0.17f,
     .minDelay =   5000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   3000,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      200000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Magnetic field Sensor",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_MAG,
     .type =       SENSOR_TYPE_MAGNETIC_FIELD,
     .maxRange =   200.0f,
     .resolution = CONVERT_M,
     .power =      5.0f,
     .minDelay =   20000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      200000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Gyroscope Sensor",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_GYRO,
     .type =       SENSOR_TYPE_GYROSCOPE,
     .maxRange =   40.0f,
     .resolution = CONVERT_GYRO,
     .power =      6.1f,
     .minDelay =   5000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      200000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Orientation Sensor",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_ORIEN,
     .type =       SENSOR_TYPE_ORIENTATION,
     .maxRange =   360.0f,
     .resolution = 0.1f,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Rotation Vector",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_RV,
     .type =       SENSOR_TYPE_ROTATION_VECTOR,
     .maxRange =   1.0f,
     .resolution = 0.0001f,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Linear Acceleration",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_LA,
     .type =       SENSOR_TYPE_LINEAR_ACCELERATION,
     .maxRange =   RANGE_A,
     .resolution = 0.01,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Gravity",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_GRAV,
     .type =       SENSOR_TYPE_GRAVITY,
     .maxRange =   GRAVITY_EARTH,
     .resolution = 0.01,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Game Rotation Vector",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_GAMERV,
     .type =       SENSOR_TYPE_GAME_ROTATION_VECTOR,
     .maxRange =   1.0f,
     .resolution = 0.0001f,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   300,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Geomagnetic Rotation Vector",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_GEORV,
     .type =       SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
     .maxRange =   1.0f,
     .resolution = 0.0001f,
     .power =      11.27f,
     .minDelay =   10000,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   20,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =      80000,
     .flags = SENSOR_FLAG_CONTINUOUS_MODE,
     .reserved =          {}
    },
    {.name =       "Step Detector",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_SD,
     .type =       SENSOR_TYPE_STEP_DETECTOR,
     .maxRange =   200.0f,
     .resolution = 1.0f,
     .power =      0.17f,
     .minDelay =   0,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   1220,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =           0,
     .flags = SENSOR_FLAG_SPECIAL_REPORTING_MODE,
     .reserved =          {}
    },
    {.name =       "Step Counter",
     .vendor =     "Google Inc.",
     .version =    1,
     .handle =     NANOHUB_SC,
     .type =       SENSOR_TYPE_STEP_COUNTER,
     .maxRange =   200.0f,
     .resolution = 1.0f,
     .power =      0.17f,
     .minDelay =   0,
     .fifoReservedEventCount = 0,
     .fifoMaxEventCount =   1220,
     .stringType =         0,
     .requiredPermission = 0,
     .maxDelay =           0,
     .flags = SENSOR_FLAG_ON_CHANGE_MODE,
     .reserved =          {}
    },
};

static struct sensor_t *Ssensor_list_ = NULL;

static int nanohub_open_sensors(const struct hw_module_t *module,
                                const char *id,
                                struct hw_device_t **device);
static int nanohub_get_sensors_list(struct sensors_module_t*,
        struct sensor_t const** list)
{
    *list = sSensorList;
    return ARRAY_SIZE(sSensorList);
}

static struct hw_module_methods_t nanohub_sensors_methods = {
    open: nanohub_open_sensors,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    common: {
      tag: HARDWARE_MODULE_TAG,
      version_major: 1,
      version_minor: 0,
      id: SENSORS_HARDWARE_MODULE_ID,
      name: "sensor hub module",
      author: "Google Inc.",
      methods: &nanohub_sensors_methods,
      dso: NULL,
      reserved: { 0 },
    },
    get_sensors_list: nanohub_get_sensors_list,
    set_operation_mode: NULL,
};

/*****************************************************************************/
nanohub_sensors_poll_context_t::nanohub_sensors_poll_context_t(
        const struct hw_module_t *module)
{
    memset(&device, 0, sizeof(sensors_poll_device_1_t));

    device.common.tag      = HARDWARE_DEVICE_TAG;
    device.common.version  = SENSORS_DEVICE_API_VERSION_1_3;
    device.common.module   = const_cast<hw_module_t *>(module);
    device.common.close    = wrapper_close;
    device.activate        = wrapper_activate;
    device.setDelay        = wrapper_setDelay;
    device.poll            = wrapper_poll;

    // Batch processing
    device.batch           = wrapper_batch;
    device.flush           = wrapper_flush;

    /*
     * One more time, assume only one sensor hub in the system.
     * Find the iio:deviceX with name "cros_ec_ring"
     * Open /dev/iio:deviceX, enable buffer.
     */
    mSensor = new NanoHub();

    mPollFds[nanohubBufFd].fd = mSensor->getFd();
    mPollFds[nanohubBufFd].events = POLLIN;
    mPollFds[nanohubBufFd].revents = 0;

    int wakeFds[2];
    int result = pipe(wakeFds);
    ALOGE_IF(result < 0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
    mWritePipeFd = wakeFds[1];

    mPollFds[nanohubWakeFd].fd = wakeFds[0];
    mPollFds[nanohubWakeFd].events = POLLIN;
    mPollFds[nanohubWakeFd].revents = 0;
}

nanohub_sensors_poll_context_t::~nanohub_sensors_poll_context_t() {
    delete mSensor;
    close(mPollFds[nanohubWakeFd].fd);
    close(mWritePipeFd);
}

int nanohub_sensors_poll_context_t::activate(int handle, int enabled) {
    int err = mSensor->activate(handle, enabled);

    if (enabled && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        ALOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int nanohub_sensors_poll_context_t::setDelay(int /* handle */,
                                             int64_t /* ns */) {
    /* No supported */
    return 0;
}

int nanohub_sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int nbEvents = 0;
    int n = 0;
    do {
        // see if we have some leftover from the last poll()
        if (mPollFds[nanohubBufFd].revents & POLLIN) {
            int nb = mSensor->readEvents(data, count);
            if (nb < count) {
                // no more data for this sensor
                mPollFds[nanohubBufFd].revents = 0;
            }
            count -= nb;
            nbEvents += nb;
            data += nb;
        }

        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            do {
                TEMP_FAILURE_RETRY(n = poll(mPollFds, numFds,
                                            nbEvents ? 0 : -1));
            } while (n < 0 && errno == EINTR);
            if (n < 0) {
                ALOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (mPollFds[nanohubWakeFd].revents & POLLIN) {
                char msg(WAKE_MESSAGE);
                int result = read(mPollFds[nanohubWakeFd].fd, &msg, 1);
                ALOGE_IF(result < 0,
                         "error reading from wake pipe (%s)", strerror(errno));
                ALOGE_IF(msg != WAKE_MESSAGE,
                         "unknown message on wake queue (0x%02x)", int(msg));
                mPollFds[nanohubWakeFd].revents = 0;
            }
        }
        // if we have events and space, go read them
    } while (n && count > 10);
    return nbEvents;
}

int nanohub_sensors_poll_context_t::batch(int handle, int /* flags */,
        int64_t sampling_period_ns,
        int64_t max_report_latency_ns)
{
    return mSensor->batch(handle, sampling_period_ns,
                          max_report_latency_ns);
}

int nanohub_sensors_poll_context_t::flush(int handle)
{
    return mSensor->flush(handle);
}


/*****************************************************************************/

int nanohub_sensors_poll_context_t::wrapper_close(struct hw_device_t *dev)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    if (ctx) {
        delete ctx;
    }

    return 0;
}

int nanohub_sensors_poll_context_t::wrapper_activate(struct sensors_poll_device_t *dev,
        int handle, int enabled)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    return ctx->activate(handle, enabled);
}

int nanohub_sensors_poll_context_t::wrapper_setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    return ctx->setDelay(handle, ns);
}

int nanohub_sensors_poll_context_t::wrapper_poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    return ctx->pollEvents(data, count);
}

int nanohub_sensors_poll_context_t::wrapper_batch(struct sensors_poll_device_1 *dev,
        int handle, int flags, int64_t period_ns, int64_t timeout)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    return ctx->batch(handle, flags, period_ns, timeout);
}

int nanohub_sensors_poll_context_t::wrapper_flush(struct sensors_poll_device_1 *dev,
        int handle)
{
    nanohub_sensors_poll_context_t *ctx = reinterpret_cast<nanohub_sensors_poll_context_t *>(dev);
    return ctx->flush(handle);
}

/*****************************************************************************/

static int nanohub_open_sensors(
        const struct hw_module_t* module, const char*,
        struct hw_device_t** device)
{
    nanohub_sensors_poll_context_t *dev = new nanohub_sensors_poll_context_t(module);

    *device = &dev->device.common;

    return 0;
}

