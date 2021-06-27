/*

  huanyang.c - Huanyang VFD spindle support

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "huanyang.h"

#if SPINDLE_HUANYANG

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#endif

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Huanyang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

typedef enum {
    VFD_Idle = 0,
    VFD_GetRPM,
    VFD_SetRPM,
    VFD_GetMaxRPM,
    VFD_GetMaxRPM50,
    VFD_GetStatus,
    VFD_SetStatus
} vfd_response_t;

static float rpm_programmed = -1.0f, rpm_low_limit = 0.0f, rpm_high_limit = 0.0f;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static settings_changed_ptr settings_changed;
static on_report_options_ptr on_report_options;
static uint32_t rpm_max = 0;
#if SPINDLE_HUANYANG == 1
static float rpm_max50 = 3000;
#endif

static void spindleSetRPM (float rpm, bool block)
{

    if (rpm != rpm_programmed) {

#if SPINDLE_HUANYANG == 2

        uint16_t data = (uint32_t)(rpm) * 10000UL / rpm_max;

        modbus_message_t rpm_cmd = {
            .xx = (void *)VFD_SetRPM,
            .adu[0] = VFD_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x10,
            .adu[4] = data >> 8,
            .adu[5] = data & 0xFF,
            .tx_length = 8,
            .rx_length = 8
        };

#else

        uint32_t data = lroundf(rpm * 5000.0f / (float)rpm_max50); // send Hz * 10  (Ex:1500 RPM = 25Hz .... Send 2500)

        modbus_message_t rpm_cmd = {
            .xx = (void *)VFD_SetRPM,
            .adu[0] = VFD_ADDRESS,
            .adu[1] = ModBus_WriteCoil,
            .adu[2] = 0x02,
            .adu[3] = data >> 8,
            .adu[4] = data & 0xFF,
            .tx_length = 7,
            .rx_length = 6
        };

#endif

        vfd_state.at_speed = false;

        modbus_send(&rpm_cmd, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
            rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
        }
        rpm_programmed = rpm;
    }
}

static void spindleUpdateRPM (float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
#if SPINDLE_HUANYANG == 2

    modbus_message_t mode_cmd = {
        .xx = (void *)VFD_SetStatus,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[5] = (!state.on || rpm == 0.0f) ? 6 : (state.ccw ? 2 : 1),
        .tx_length = 8,
        .rx_length = 8
    };

#else

    modbus_message_t mode_cmd = {
        .xx = (void *)VFD_SetStatus,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x01,
        .adu[3] = (!state.on || rpm == 0.0f) ? 0x08 : (state.ccw ? 0x11 : 0x01),
        .tx_length = 6,
        .rx_length = 6
    };

#endif

    if(vfd_state.ccw != state.ccw)
        rpm_programmed = 0.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, true))
        spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{

#if SPINDLE_HUANYANG == 2

    modbus_message_t mode_cmd = {
        .xx = (void *)VFD_GetRPM,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x70,
        .adu[3] = 0x0C,
        .adu[4] = 0x00,
        .adu[5] = 0x02,
        .tx_length = 8,
        .rx_length = 8
    };

#else

    modbus_message_t mode_cmd = {
        .xx = (void *)VFD_GetRPM,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x03,
        .adu[3] = 0x01,
        .tx_length = 8,
        .rx_length = 8
    };

#endif

    modbus_send(&mode_cmd, false); // TODO: add flag for not raising alarm?

    return vfd_state; // return previous state as we do not want to wait for the response
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->xx) {

            case VFD_GetRPM:
#if SPINDLE_HUANYANG == 2
                spindle_data.rpm = (float)((msg->adu[4] << 8) | msg->adu[5]);
#else
                spindle_data.rpm = (float)((msg->adu[4] << 8) | msg->adu[5]) * (float)rpm_max50 / 5000.0f;
#endif
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= rpm_low_limit && spindle_data.rpm <= rpm_high_limit);
                break;

            case VFD_GetMaxRPM:
                rpm_max = (msg->adu[4] << 8) | msg->adu[5];
                break;

#if SPINDLE_HUANYANG == 1
            case VFD_GetMaxRPM50:
                rpm_max50 = (msg->adu[4] << 8) | msg->adu[5];
                break;
#endif

            default:
                break;
        }
    }
}

static void raise_alarm (uint_fast16_t state)
{
    system_raise_alarm(Alarm_Spindle);
}

static void rx_exception (uint8_t code)
{
    protocol_enqueue_rt_command(raise_alarm);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
#if SPINDLE_HUANYANG == 2
        hal.stream.write("[PLUGIN:HUANYANG VFD P2A v0.03]" ASCII_EOL);
#else
        hal.stream.write("[PLUGIN:HUANYANG VFD v0.03]" ASCII_EOL);
#endif
    }
}

static void huanyang_settings_changed (settings_t *settings)
{
    static bool init_ok = false;
    static driver_cap_t driver_cap;
    static spindle_ptrs_t spindle_org;

    if(settings_changed)
        settings_changed(settings);

    if(!modbus_isup())
        return;

    if(!init_ok && hal.driver_cap.dual_spindle) {
        driver_cap = hal.driver_cap;
        memcpy(&spindle_org, &hal.spindle, sizeof(spindle_ptrs_t));
    }

    if(hal.driver_cap.dual_spindle && settings->mode == Mode_Laser) {

        if(hal.spindle.set_state == spindleSetState) {
            hal.driver_cap = driver_cap;
            memcpy(&spindle_org, &hal.spindle, sizeof(spindle_ptrs_t));
        }

    } else {

        if(settings->spindle.ppr == 0)
            hal.spindle.get_data = spindleGetData;

        if(hal.spindle.set_state != spindleSetState) {

            hal.spindle.set_state = spindleSetState;
            hal.spindle.get_state = spindleGetState;
            hal.spindle.update_rpm = spindleUpdateRPM;
            hal.spindle.reset_data = NULL;

            hal.driver_cap.variable_spindle = On;
            hal.driver_cap.spindle_at_speed = On;
            hal.driver_cap.spindle_dir = On;
        }

#if SPINDLE_HUANYANG == 2
        if(!init_ok) {

            modbus_message_t cmd = {
                .xx = (void *)VFD_GetMaxRPM,
                .adu[0] = VFD_ADDRESS,
                .adu[1] = ModBus_ReadHoldingRegisters,
                .adu[2] = 0xB0,
                .adu[3] = 0x05,
                .adu[4] = 0x00,
                .adu[5] = 0x02,
                .tx_length = 8,
                .rx_length = 8
            };

            modbus_send(&cmd, true);
        }
#else
        if(!init_ok) {

            modbus_message_t cmd = {
                .xx = (void *)VFD_GetMaxRPM50,
                .adu[0] = VFD_ADDRESS,
                .adu[1] = ModBus_ReadCoils,
                .adu[2] = 0x03,
                .adu[3] = 122,
                .tx_length = 8,
                .rx_length = 8
            };

            modbus_send(&cmd, true);
        }
#endif
    }

    init_ok = true;
}

void huanyang_init (modbus_stream_t *stream)
{
    if(stream) {
        settings_changed = hal.settings_changed;
        hal.settings_changed = huanyang_settings_changed;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        stream->on_rx_packet = rx_packet;
        stream->on_rx_exception = rx_exception;

        if(!hal.driver_cap.dual_spindle)
            huanyang_settings_changed(&settings);
    }
}

#endif
