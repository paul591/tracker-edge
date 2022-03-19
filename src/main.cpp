/*
 * Copyright (c) 2022 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Particle.h"

#include "tracker_config.h"
#include "tracker.h"

#include "DebounceSwitchRK.h"
#include "ClosedCube_SHT31D.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

PRODUCT_ID(TRACKER_PRODUCT_ID);
PRODUCT_VERSION(TRACKER_PRODUCT_VERSION);

STARTUP(
    Tracker::startup();
);

Logger appLogger("app.main");

SerialLogHandler logHandler(115200, LOG_LEVEL_NONE, {
    { "app.main", LOG_LEVEL_INFO },
});


SHT31D_CC::ClosedCube_SHT31D sts31;

static SHT31D_CC::SHT31D_ErrorCode startSts31() {
    auto ret = sts31.periodicStart(SHT31D_CC::REPEATABILITY_MEDIUM, SHT31D_CC::FREQUENCY_HZ5);
    if (SHT31D_CC::SHT31D_ErrorCode::NO_ERROR != ret) {
        appLogger.error("STS31 sensor setup failure");
    }

    return ret;
}

static SHT31D_CC::SHT31D_ErrorCode stopSts31() {
    return sts31.periodicStop();
}

void appSleep(TrackerSleepContext context) {
    // Put STS31 into idle mode
    stopSts31();
}

void appWake(TrackerSleepContext context) {
    // Put STS31 into periodic sampling
    startSts31();
}

int appPublish(JSONWriter& message, LocationPoint& point, const void* context) {
    auto temp = sts31.periodicFetchData();
    if (SHT31D_CC::SHT31D_ErrorCode::NO_ERROR != temp.error) {
        message.name("c_temp").value((double)temp.t, 1);
    }

    FuelGauge fg;
    auto voltage = fg.getVCell();
    message.name("vbatt").value((double)voltage, 3);
    return SYSTEM_ERROR_NONE;
}


void setup() {
    SystemPowerConfiguration conf;
    conf.powerSourceMaxCurrent(2000); // milliamps
    conf.batteryChargeCurrent(1200); // milliamps
    int res = System.setPowerConfiguration(conf);

    // Optional - Wait for a waiting terminal for showing logs
    waitFor(Serial.isConnected, 10000);

    Tracker::instance().init();

    // Associate user button with debounce handler
    // See https://github.com/rickkas7/DebounceSwitchRK for more information
    // Some example DebouncePressState states:
    //           Single tap (< 3s):  PRESS_START -> SHORT -> RELEASED -> TAP
    //           Double tap (< 3s):  PRESS_START -> SHORT -> RELEASED -> PRESS_START -> SHORT -> RELEASED -> TAP
    //    Long press (> 3s, < 10s):  PRESS_START -> PROGRESS -> LONG -> RELEASED
    //    Very long press  (> 10s):  PRESS_START -> PROGRESS -> VERY_LONG -> RELEASED
    DebounceSwitch::getInstance()->setup();
    DebounceSwitch::getInstance()->addSwitch(TRACKER_89503_USER_BUTTON, DebounceSwitchStyle::PRESS_LOW_PULLUP,
        [](DebounceSwitchState* switchState, void *) {
            appLogger.info("toggle state=%s", switchState->getPressStateName());
        });

    // Configure STS3X temperature (+humidity) sensor
    sts31.begin(TRACKER_89503_STS3X_I2C_ADDR);  // No real error path
    startSts31();

    // Register for Tracker events
    TrackerSleep::instance().registerSleep(appSleep);
    TrackerSleep::instance().registerWake(appWake);
    TrackerLocation::instance().regLocGenCallback(appPublish);

    // We want the user button to wake us up
    TrackerSleep::instance().wakeFor(TRACKER_89503_USER_BUTTON, FALLING);
}

static unsigned int tick {};

void loop() {
    Tracker::instance().loop();

    if ((System.uptime() - tick) > 10) {
        tick = System.uptime();
        // Throw away the humidity measurement because the device is an STS31
        auto temp = sts31.periodicFetchData();
        if (SHT31D_CC::SHT31D_ErrorCode::NO_ERROR != temp.error) {
            appLogger.info("Temperature = %0.1f", temp.t);
        }
    }
}