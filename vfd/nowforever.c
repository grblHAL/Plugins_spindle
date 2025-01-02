/*
  nowforever.c - NowForever VFD spindle support

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

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
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static settings_changed_ptr settings_changed;
static driver_reset_ptr driver_reset;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = VFD_RETRIES,
    .retry_delay = VFD_RETRY_DELAY,
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
        .tx_length = 8,
        .rx_length = 9
    };

    modbus_send(&cmd, &callbacks, true);
}

static void set_rpm (float rpm, bool block)
{
    static uint8_t busy = 0;

    if(busy && !block)
        return;

    if(rpm != spindle_data.rpm_programmed ) {

        uint16_t freq = (uint16_t)(rpm * 1.667f); // * 100.0f / 60.0f

        freq = min(max(freq, freq_min), freq_max);

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = modbus_address,
            .adu[1] = ModBus_WriteRegisters,
            .adu[2] = 0x09,
            .adu[3] = 0x01,
            .adu[4] = 0x00,
            .adu[5] = 0x01,
            .adu[6] = 0x02,
            .adu[7] = freq >> 8,
            .adu[8] = freq & 0xFF,
            .tx_length = 11,
            .rx_length = 8
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
        .adu[1] = ModBus_WriteRegisters,
        .adu[2] = 0x09,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .adu[6] = 0x02,
        .adu[7] = 0x00,
        .adu[8] = (!state.on || rpm == 0.0f) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 11,
        .rx_length = 8
    };

    busy = true;

    if(vfd_state.ccw != state.ccw)
        spindle_data.rpm_programmed = 0.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        set_rpm(rpm, true);

    busy = false;
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
        .tx_length = 8,
        .rx_length = 7
    };

    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    vfd_state.at_speed = spindle->get_data(SpindleData_AtSpeed)->state_programmed.at_speed;

    return vfd_state; // return previous state as we do not want to wait for the response
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

static inline float f2rpm (uint16_t f)
{
    return (float)f * 60.0f / 100.0f;
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                if(msg->adu[2] == 2)
                    spindle_validate_at_speed(spindle_data, f2rpm((msg->adu[3] << 8) | msg->adu[4]));
                break;

            case VFD_GetRPMRange:
                if(msg->adu[2] == 4) {
                    freq_min = (msg->adu[5] << 8) | msg->adu[6];
                    freq_max = (msg->adu[3] << 8) | msg->adu[4];
                    spindle_hal->cap.rpm_range_locked = On;
                    spindle_hal->rpm_min = f2rpm(freq_min);
                    spindle_hal->rpm_max = f2rpm(freq_max);
                }
                break;

            default:
                break;
        }
    }
}

static void rx_exception (uint8_t code, void *context)
{
    vfd_failed(false);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Nowforever VFD", "0.02");
}

static void onDriverReset (void)
{
    driver_reset();

    if(spindle_hal)
        spindleGetRPMRange();
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

static void settingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

    if(changed.spindle) {

        spindle_ptrs_t *spindle = spindle_get_hal(spindle_id, SpindleHAL_Configured);

        spindle->at_speed_tolerance = settings->spindle.at_speed_tolerance;
        spindle_data.at_speed_enabled = settings->spindle.at_speed_tolerance >= 0.0f;
    }
}

void vfd_nowforever_init (void)
{
    static const vfd_spindle_ptrs_t vfd = {
        .spindle = {
            .type = SpindleType_VFD,
            .ref_id = SPINDLE_NOWFOREVER,
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
            .get_data = spindleGetData
        }
    };

    if((spindle_id = vfd_register(&vfd, "Nowforever")) != -1) {

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
