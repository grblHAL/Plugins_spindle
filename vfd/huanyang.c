/*

  huanyang.c - Huanyang v1 VFD spindle support

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "../shared.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_HUANYANG1)

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint32_t modbus_address, exceptions = 0;
static float amps = 0.0f, amps_max = 0.0f, rpm_at_50Hz = 0.0f;
static spindle_id_t spindle_id = -1;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static settings_changed_ptr settings_changed;
static driver_reset_ptr driver_reset;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

// Testing Huanyang VFDs (with other devices on the bus) has shown failure to respond if silent period is < 6ms
static const modbus_silence_timeout_t silence =
{
    .b2400   = 16,
    .b4800   = 8,
    .b9600   = 6,
    .b19200  = 6,
    .b38400  = 6,
    .b115200 = 6
};

static const modbus_callbacks_t callbacks = {
    .retries = VFD_RETRIES,
    .retry_delay = VFD_RETRY_DELAY,
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

// Read maximum configured RPM from spindle, value is used later for calculating current RPM
// In the case of the original Huanyang protocol, the value is the configured RPM at 50Hz
static void spindleGetRPMLimits (void)
{
    modbus_message_t cmd = {
        .context = (void *)VFD_GetRPMAt50Hz,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadCoils,
        .adu[2] = 0x03,
        .adu[3] = 0x90, // PD144
        .adu[4] = 0x00,
        .adu[5] = 0x00,
        .tx_length = 8,
        .rx_length = 8
    };

    rpm_at_50Hz = 0.0f;

    if(modbus_send(&cmd, &callbacks, true)) {

        cmd.context = (void *)VFD_GetMinRPM;
        cmd.adu[3] = 0x0B; // PD011

        if(modbus_send(&cmd, &callbacks, true)) {
            cmd.context = (void *)VFD_GetMaxRPM;
            cmd.adu[3] = 0x05; // PD005
            modbus_send(&cmd, &callbacks, true);
        }
    }

    if(rpm_at_50Hz == 0.0f)
        rpm_at_50Hz = 3000.0f;
}

// Read maximum configured current from spindle, value is used later for calculating spindle load
static void spindleGetMaxAmps (void)
{
    modbus_message_t cmd = {
        .context = (void *)VFD_GetMaxAmps,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadCoils,
        .adu[2] = 0x03,
        .adu[3] = 0x8E, // PD142
        .adu[4] = 0x00,
        .adu[5] = 0x00,
        .tx_length = 8,
        .rx_length = 8
    };

    modbus_set_silence(&silence);
    modbus_send(&cmd, &callbacks, true);
}

static void set_rpm (float rpm, bool block)
{
    static uint8_t busy = 0;

    if(busy && !block)
        return;

    if(rpm_at_50Hz != 0.0f && rpm != spindle_data.rpm_programmed) {

        uint32_t data = lroundf(rpm * 5000.0f / rpm_at_50Hz); // send Hz * 10  (Ex:1500 RPM = 25Hz .... Send 2500)

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = modbus_address,
            .adu[1] = ModBus_WriteCoil,
            .adu[2] = 0x02,
            .adu[3] = data >> 8,
            .adu[4] = data & 0xFF,
            .tx_length = 7,
            .rx_length = 6
        };

        busy++;
        modbus_send(&rpm_cmd, &callbacks, block);
        spindle_set_at_speed_range(spindle_hal, &spindle_data, rpm);
        busy--;
    }
}

static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    set_rpm(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    static bool busy = false;

    if(busy)
        return;

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x01,
        .adu[3] = (!state.on || rpm == 0.0f) ? 0x08 : (state.ccw ? 0x11 : 0x01),
        .tx_length = 6,
        .rx_length = 6
    };

    busy = true;

    if(vfd_state.ccw != state.ccw)
        spindle_data.rpm_programmed = -1.0f;

    vfd_state.on = spindle_data.state_programmed.on = state.on;
    vfd_state.ccw = spindle_data.state_programmed.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        set_rpm(rpm, true);

    busy = false;
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    modbus_message_t rpm_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x03,
        .adu[3] = 0x01,
        .tx_length = 8,
        .rx_length = 8
    };

    modbus_send(&rpm_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    modbus_message_t amps_cmd = {
        .context = (void *)VFD_GetAmps,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x03,
        .adu[3] = 0x02,     // Output amps * 10
        .tx_length = 8,
        .rx_length = 8
    };
    modbus_send(&amps_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    vfd_state.at_speed = spindle->get_data(SpindleData_AtSpeed)->state_programmed.at_speed;

    return vfd_state; // return previous state as we do not want to wait for the response
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                exceptions = 0;
                spindle_validate_at_speed(spindle_data, (float)((msg->adu[4] << 8) | msg->adu[5]) * rpm_at_50Hz / 5000.0f);
                break;

            case VFD_GetMinRPM:
                if(rpm_at_50Hz != 0.0f)
                    spindle_hal->rpm_min = (float)((msg->adu[4] << 8) | msg->adu[5]) * rpm_at_50Hz / 5000.0f;
                break;

            case VFD_GetMaxRPM:
                if(rpm_at_50Hz != 0.0f) {
                    spindle_hal->cap.rpm_range_locked = On;
                    spindle_hal->rpm_max = (float)((msg->adu[4] << 8) | msg->adu[5]) * rpm_at_50Hz / 5000.0f;
                }
                break;

            case VFD_GetRPMAt50Hz:
                if(spindle_hal)
                    rpm_at_50Hz = (float)((msg->adu[4] << 8) | msg->adu[5]);
                break;

            case VFD_GetMaxAmps:
                amps_max = (float)((msg->adu[4] << 8) | msg->adu[5]) / 10.0f;
                break;

            case VFD_GetAmps:
                amps = (float)((msg->adu[4] << 8) | msg->adu[5]) / 10.0f;
                break;

            default:
                break;
        }
    }
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup().rtu;
}

static float spindleGetLoad (void)
{
    return amps_max ? (amps / amps_max) * 100.0f : 0.0f;
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

static void rx_exception (uint8_t code, void *context)
{
    if((vfd_response_t)context != VFD_GetRPM || ++exceptions == VFD_ASYNC_EXCEPTION_LEVEL)
        vfd_failed(false);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("HUANYANG VFD", "0.15");
}

static void after_reset (void *data)
{
    spindleGetRPMLimits();
    spindleGetMaxAmps();
}

static void onDriverReset (void)
{
    driver_reset();

    if(spindle_hal)
        task_add_delayed(after_reset, NULL, 50);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {

        spindle_hal = spindle;
        spindle_data.rpm_programmed = -1.0f;
        spindle_data.at_speed_enabled = settings.spindle.at_speed_tolerance >= 0.0f;
        spindle->at_speed_tolerance = settings.spindle.at_speed_tolerance;

        modbus_set_silence(&silence);
        modbus_address = vfd_get_modbus_address(spindle_id);

        spindleGetRPMLimits();
        spindleGetMaxAmps();

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void settingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

    if(changed.spindle) {

        spindle_ptrs_t *spindle = spindle_get_hal(spindle_id, SpindleHAL_Configured);

        spindle->at_speed_tolerance = settings->spindle.at_speed_tolerance;
        spindle_data.at_speed_enabled = settings->spindle.at_speed_tolerance >= 0.0f;
    }
}

void vfd_huanyang_init (void)
{
    static const vfd_spindle_ptrs_t vfd = {
        .spindle = {
            .type = SpindleType_VFD,
            .ref_id = SPINDLE_HUANYANG1,
            .cap = {
                .variable = On,
                .at_speed = On,
                .direction = On,
                .cmd_controlled = On
            },
            .config = spindleConfig,
            .set_state = spindleSetState,
            .get_state = spindleGetState,
            .update_rpm = spindleUpdateRPM,
            .get_data = spindleGetData,
        },
        .vfd.get_load = spindleGetLoad
    };

    if((spindle_id = vfd_register(&vfd, "Huanyang v1")) != -1) {

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        settings_changed = hal.settings_changed;
        hal.settings_changed = settingsChanged;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onDriverReset;
    }
}

#endif
