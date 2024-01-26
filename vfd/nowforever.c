/*
  nowforever.c - NowForever VFD spindle support

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io

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

  https://www.youtube.com/watch?v=lncconN83G4
  https://github.com/havardAasen/nowforever_vfd/blob/master/nowforever_vfd.c

*/

#include "../shared.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_NOWFOREVER)

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint32_t modbus_address;
static spindle_id_t spindle_id;
static uint32_t freq_min = 0, freq_max = 0;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_data_t spindle_data = {0};
static spindle_state_t vfd_state = {0};
static spindle_get_data_ptr on_get_data = NULL;
static on_report_options_ptr on_report_options;
static on_spindle_select_ptr on_spindle_select;
static on_spindle_selected_ptr on_spindle_selected;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup();
}

// Read maximum configured RPM from spindle, value is used later for calculating current RPM
// In the case of the original Huanyang protocol, the value is the configured RPM at 50Hz
static void spindleGetRPMRange (void)
{
    modbus_message_t cmd = {
        .context = (void *)VFD_GetRPMRange,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x00,
        .adu[3] = 0x07,
        .adu[4] = 0x00,
        .adu[5] = 0x02,
        .tx_length = 6,
        .rx_length = 7
    };

    modbus_send(&cmd, &callbacks, true);
}

static void spindleSetRPM (float rpm, bool block)
{
    if(rpm != spindle_data.rpm_programmed ) {

        uint16_t freq = (uint16_t)(rpm * 1.667f); // * 100.0f / 60.0f

        freq = min(max(freq, freq_min), freq_max);

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = modbus_address,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x09,
            .adu[3] = 0x01,
            .adu[4] = 0x00,
            .adu[5] = 0x01,
            .adu[6] = 0x02,
            .adu[7] = freq >> 8,
            .adu[8] = freq & 0xFF,
            .tx_length = 9,
            .rx_length = 6
        };

        vfd_state.at_speed = false;

        modbus_send(&rpm_cmd, &callbacks, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            spindle_data.rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
            spindle_data.rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
        }
        spindle_data.rpm_programmed = rpm;
    }
}

static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegisters,
        .adu[2] = 0x09,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .adu[6] = 0x02,
        .adu[7] = 0x00,
        .adu[8] = (!state.on || rpm == 0.0f) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 9,
        .rx_length = 6
    };

    if(vfd_state.ccw != state.ccw)
        spindle_data.rpm_programmed = 0.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x05,
        .adu[3] = 0x02,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .tx_length = 6,
        .rx_length = 5
    };

    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    return vfd_state; // return previous state as we do not want to wait for the response
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                if(msg->adu[2] == 2) {
                    spindle_data.rpm = (float)((msg->adu[3] << 8) | msg->adu[4]) * 0.6f; // * 60.0f / 100.0f
                    vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
                }
                break;

            case VFD_GetRPMRange:
                if(msg->adu[2] == 4) {
                    freq_min = (msg->adu[5] << 8) | msg->adu[6];
                    freq_max = (msg->adu[3] << 8) | msg->adu[4];
                }
                break;

            default:
                break;
        }
    }
}

static void raise_alarm (void *data)
{
    system_raise_alarm(Alarm_Spindle);
}

static void rx_exception (uint8_t code, void *context)
{
    // Alarm needs to be raised directly to correctly handle an error during reset (the rt command queue is
    // emptied on a warm reset). Exception is during cold start, where alarms need to be queued.
    if(sys.cold_start)
        protocol_enqueue_foreground_task(raise_alarm, NULL);
    else
        system_raise_alarm(Alarm_Spindle);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Nowforever VFD v0.01]" ASCII_EOL);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {
        on_get_data = spindle->get_data;
        spindle->get_data = spindleGetData;
    }

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {

        spindle_hal = spindle;
        spindle_data.rpm_programmed = -1.0f;

        modbus_set_silence(NULL);
        modbus_address = vfd_get_modbus_address(spindle_id);

        spindleGetRPMRange();

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

void vfd_nowforever_init (void)
{
    static const vfd_spindle_ptrs_t spindle = {
        .spindle.type = SpindleType_VFD,
        .spindle.cap.variable = On,
        .spindle.cap.at_speed = On,
        .spindle.cap.direction = On,
        .spindle.cap.cmd_controlled = On,
        .spindle.config = spindleConfig,
        .spindle.set_state = spindleSetState,
        .spindle.get_state = spindleGetState,
        .spindle.update_rpm = spindleUpdateRPM
    };

    if((spindle_id = vfd_register(&spindle, "Nowforever")) != -1) {

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;
    }
}

#endif
