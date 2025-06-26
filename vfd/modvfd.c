/*

  vfd/modvfd.c - MODVFD VFD spindle support for Modbus RTU compatible spindles.

  Requires Modbus RTU support with 8N1 configuration.

  Part of grblHAL

  Copyright (c) 2022 Andrew Marles
  Copyright (c) 2022-2025 Terje Io

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

#if SPINDLE_ENABLE & (1<<SPINDLE_MODVFD)

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint32_t modbus_address, exceptions = 0;
static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static on_spindle_selected_ptr on_spindle_selected;
static on_report_options_ptr on_report_options;
static settings_changed_ptr settings_changed;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = VFD_RETRIES,
    .retry_delay = VFD_RETRY_DELAY,
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

// TODO: there should be a mechanism to read max RPM from the VFD in order to configure RPM/Hz instead of above define.

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup().rtu;
}

static void set_rpm (float rpm, bool block)
{
    static uint8_t busy = 0;

    if(busy && !block)
        return;

    uint16_t data = ((uint32_t)(rpm)) / vfd_config.in_divider * vfd_config.in_multiplier;

    modbus_message_t rpm_cmd = {
        .context = (void *)VFD_SetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = vfd_config.set_freq_reg >> 8,
        .adu[3] = vfd_config.set_freq_reg & 0xFF,
        .adu[4] = data >> 8,
        .adu[5] = data & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };

    busy++;
    modbus_send(&rpm_cmd, &callbacks, block);
    spindle_set_at_speed_range(spindle_hal, &spindle_data, rpm);
    busy--;
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

    uint16_t runstop;

    if(!state.on || rpm == 0.0f)
        runstop = vfd_config.stop_cmd;
    else
        runstop = state.ccw ? vfd_config.run_ccw_cmd : vfd_config.run_cw_cmd;

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = true,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = vfd_config.runstop_reg >> 8,
        .adu[3] = vfd_config.runstop_reg & 0xFF,
        .adu[4] = runstop >> 8,
        .adu[5] = runstop & 0xFF,
        .tx_length = 8,
        .rx_length = 8
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

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = vfd_config.get_freq_reg >> 8,
        .adu[3] = vfd_config.get_freq_reg & 0xFF,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .tx_length = 8,
        .rx_length = 7
    };

    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    vfd_state.at_speed = spindle->get_data(SpindleData_AtSpeed)->state_programmed.at_speed;

    return vfd_state; // return previous state as we do not want to wait for the response
}

static float f2rpm (uint16_t f)
{
    return (float)f * (vfd_config.out_multiplier / vfd_config.out_divider);
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                exceptions = 0;
                spindle_validate_at_speed(spindle_data, f2rpm((msg->adu[3] << 8) | msg->adu[4]));
                break;

//            case VFD_GetMaxRPM:
//                spindle_hal->cap.rpm_range_locked = true;
//                spindle_hal->rpm_max = f2rpm((msg->adu[4] << 8) | msg->adu[5]);
//                break;

            default:
                break;
        }
    }
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
        report_plugin("MODVFD", "0.05");
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {

        spindle_hal = spindle;
        spindle_data.rpm_programmed = -1.0f;

        modbus_set_silence(NULL);
        modbus_address = vfd_get_modbus_address(spindle_id);

//        spindleGetMaxRPM();

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

void vfd_modvfd_init (void)
{
    static const vfd_spindle_ptrs_t vfd = {
        .spindle = {
            .type = SpindleType_VFD,
            .ref_id = SPINDLE_MODVFD,
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
        }
    };

    if((spindle_id = vfd_register(&vfd, "MODVFD")) != -1) {

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        settings_changed = hal.settings_changed;
        hal.settings_changed = settingsChanged;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;
    }
}

#endif
