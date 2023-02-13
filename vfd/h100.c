/*

  h100.c - H-100 VFD spindle support

  Part of grblHAL

  Copyright (c) 2022-2023 Terje Io

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

#include "../shared.h"

#if VFD_ENABLE == SPINDLE_ALL || VFD_ENABLE == SPINDLE_H100

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint32_t modbus_address, freq_min = 0, freq_max = 0;
static spindle_id_t spindle_id = -1;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_state_t vfd_state = {0};
static spindle_get_data_ptr on_get_data = NULL;
static spindle_data_t spindle_data = {0};
static on_spindle_select_ptr on_spindle_select;
static on_spindle_selected_ptr on_spindle_selected;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

// Read min and max configured frequency from spindle
static void spindleGetRPMLimits (void)
{
    modbus_message_t cmd = {
        .context = (void *)VFD_GetMinRPM,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x00,
        .adu[3] = 0x0B, // PD11
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .tx_length = 8,
        .rx_length = 7
    };

    if(modbus_send(&cmd, &callbacks, true)) {

        cmd.context = (void *)VFD_GetMaxRPM;
        cmd.adu[3] = 0x05; // PD05

        modbus_send(&cmd, &callbacks, true);
    }
}

static void spindleSetRPM (float rpm, bool block)
{
    if (rpm != spindle_data.rpm_programmed) {

        uint16_t freq = (uint16_t)(rpm * 0.167f); // * 10.0f / 60.0f

        freq = min(max(freq, freq_min), freq_max);

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = modbus_address,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x02,
            .adu[4] = freq >> 8,
            .adu[5] = freq & 0xFF,
            .tx_length = 8,
            .rx_length = 8
        };

        vfd_state.at_speed = false;
        spindle_data.rpm_programmed = rpm;

        modbus_send(&rpm_cmd, &callbacks, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            spindle_data.rpm_low_limit = rpm * (1.0f - (settings.spindle.at_speed_tolerance / 100.0f));
            spindle_data.rpm_high_limit = rpm * (1.0f + (settings.spindle.at_speed_tolerance / 100.0f));
        }
    }
}

static void spindleUpdateRPM (float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteCoil,
        .adu[2] = 0x00,
        .adu[3] = (!state.on || rpm == 0.0f) ? 0x4B : (state.ccw ? 0x4A : 0x49),
        .adu[4] = 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };

    if(vfd_state.ccw != state.ccw)
        spindle_data.rpm_programmed = -1.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x00,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = 0x02,
        .tx_length = 8,
        .rx_length = 9
    };

    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    // Get the actual RPM from spindle encoder input when available.
    if(on_get_data) {
        float rpm = on_get_data(SpindleData_RPM)->rpm;
        vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
    }

    return vfd_state; // return previous state as we do not want to wait for the response
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    if(on_get_data) {

        spindle_data_t *data;

        data = on_get_data(request);
        data->rpm_low_limit = spindle_data.rpm_low_limit;
        data->rpm_high_limit = spindle_data.rpm_high_limit;
        data->rpm_programmed = spindle_data.rpm_programmed;
        data->state_programmed.on = spindle_data.state_programmed.on;
        data->state_programmed.ccw = spindle_data.state_programmed.ccw;

        return data;
    }

    return &spindle_data;
}

static float f2rpm (uint16_t f)
{
    return (float)f * 6.0f; // * 60.0f / 10.0f
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_data.rpm = f2rpm((msg->adu[3] << 8) | msg->adu[4]);
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
                break;

            case VFD_GetMinRPM:
                freq_min = (msg->adu[3] << 8) | msg->adu[4];
                break;

            case VFD_GetMaxRPM:
                freq_max = (msg->adu[3] << 8) | msg->adu[4];
                spindle_hal->cap.rpm_range_locked = On;
                spindle_hal->rpm_min = f2rpm(freq_min);
                spindle_hal->rpm_max = f2rpm(freq_max);
                break;

            default:
                break;
        }
    }
}

static void rx_exception (uint8_t code, void *context)
{
    // Alarm needs to be raised directly to correctly handle an error during reset (the rt command queue is
    // emptied on a warm reset). Exception is during cold start, where alarms need to be queued.
    vfd_failed(false);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:H-100 VFD v0.02]" ASCII_EOL);
}

static void onDriverReset (void)
{
    driver_reset();

    if(spindle_hal)
        spindleGetRPMLimits();
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup();
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

        spindleGetRPMLimits();

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

void vfd_h100_init (void)
{
    static const vfd_spindle_ptrs_t spindle = {
        .spindle.type = SpindleType_VFD,
        .spindle.cap.variable = On,
        .spindle.cap.at_speed = On,
        .spindle.cap.direction = On,
        .spindle.config = spindleConfig,
        .spindle.set_state = spindleSetState,
        .spindle.get_state = spindleGetState,
        .spindle.update_rpm = spindleUpdateRPM
    };

    if((spindle_id = vfd_register(&spindle, "H-100")) != -1) {

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onDriverReset;
    }
}

#endif
