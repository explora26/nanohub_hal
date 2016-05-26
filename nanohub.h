/*
 * Copyright (C) 2008-2014 The Android Open Source Project
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

#ifndef NANOHUB_H
#define NANOHUB_H

#include <errno.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <utils/BitSet.h>

#include <hardware/sensors.h>
#include "nanohubPacket.h"
#include "nanohub_sensors.h"

#define READ_QUEUE_DEPTH 10

#define CROS_EC_EVENT_FLUSH_FLAG 0x1
#define CROS_EC_EVENT_WAKEUP_FLAG 0x2

/*****************************************************************************/

enum nanohub_sensor_id {
    NANOHUB_ACCEL,
    NANOHUB_GYRO,
    NANOHUB_MAG,
    NANOHUB_ORIEN,
    NANOHUB_RV,
    NANOHUB_LA,
    NANOHUB_GRAV,
    NANOHUB_GAMERV,
    NANOHUB_GEORV,
    NANOHUB_SD,
    NANOHUB_SC,
    NANOHUB_ID_MAX,
};

struct sensor_config
{
    uint32_t evtType;
    uint64_t latency;
    uint32_t rate;
    uint8_t sensorType;
    union
    {
        struct
        {
            uint8_t enable : 1;
            uint8_t flush : 1;
            uint8_t calibrate : 1;
            uint8_t reserved : 5;
        };
        uint8_t flags;
    };
} __attribute__((packed));

struct EvtPacket
{
    uint32_t sensType;
    uint64_t referenceTime;
    union
    {
        struct SensorFirstSample firstSample;
        struct SingleAxisDataPoint single[NANOHUB_SENSOR_DATA_MAX / sizeof(struct SingleAxisDataPoint)];
        struct TripleAxisDataPoint triple[NANOHUB_SENSOR_DATA_MAX / sizeof(struct TripleAxisDataPoint)];
        struct WifiScanResult wifiScanResults[NANOHUB_SENSOR_DATA_MAX / sizeof(struct WifiScanResult)];
        uint8_t buffer[NANOHUB_SENSOR_DATA_MAX];
    };
} __attribute__((packed));

class NanoHub {
    struct sensor_config mSensorConfig[NANOHUB_ID_MAX];
    NanohubReadEventResponse mEvents;
    int mDataFd;

    int processEvent(sensors_event_t* data, const struct  NanohubReadEventResponse *event);
public:
    NanoHub();
    virtual ~NanoHub();
    virtual int getFd(void);
    int readEvents(sensors_event_t* data, int count);

    virtual int activate(int handle, int enabled);
    virtual int batch(int handle, int64_t period_ns, int64_t timeout);
    virtual int flush(int handle);
};

#endif  // NANOHUB_H
