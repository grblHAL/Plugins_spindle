/*

  vfd/modvfd.c - MODVFD VFD spindle support for Modbus RTU compatible spindles.

  Requires Modbus RTU support with 8N1 configuration.

  Part of grblHAL

  Copyright (c) 2022 Andrew Marles
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

#if VFD_ENABLE == SPINDLE_ALL || VFD_ENABLE == SPINDLE_MODVFD

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint16_t retry_counter = 0;
static uint32_t modbus_address;
static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static spindle_get_data_ptr on_get_data = NULL;
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

// TODO: there should be a mechanism to read max RPM from the VFD in order to configure RPM/Hz instead of above define.


static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup();
}

static void spindleSetRPM (float rpm, bool block)
{
    static uint_fast8_t retries = 0;

    if(retries)
        return; // block reentry

    bool ok;
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

    vfd_state.at_speed = false;
    spindle_data.rpm_programmed = rpm;

    do {
        if(!(ok = modbus_send(&rpm_cmd, &callbacks, block)))
            retries++;
    } while(!ok && block && retries <= VFD_RETRIES);

    if(!ok)
        vfd_failed(false);

    if(settings.spindle.at_speed_tolerance > 0.0f) {
        spindle_data.rpm_low_limit = rpm * (1.0f - (settings.spindle.at_speed_tolerance / 100.0f));
        spindle_data.rpm_high_limit = rpm * (1.0f + (settings.spindle.at_speed_tolerance / 100.0f));
    }

    retries = 0;
}

static void spindleUpdateRPM (float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    static uint_fast8_t retries = 0;

    bool ok;
    uint16_t runstop;

    if(retries)
        return; // block reentry

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

    if(vfd_state.ccw != state.ccw)
        spindle_data.rpm_programmed = -1.0f;

    vfd_state.on = spindle_data.state_programmed.on = state.on;
    vfd_state.ccw = spindle_data.state_programmed.ccw = state.ccw;

    do {
        if(!(ok = modbus_send(&mode_cmd, &callbacks, true)))
            retries++;
    } while(!ok && retries <= VFD_RETRIES);

    if(ok)
        spindleSetRPM(rpm, true);
    else
        vfd_failed(false);

    retries = 0;
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

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = vfd_config.get_freq_reg >> 8 & 0xFF,
        .adu[3] = vfd_config.get_freq_reg & 0xFF,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .tx_length = 8,
        .rx_length = 7
    };

    if(ms > (last_ms + VFD_RETRY_DELAY)){ //don't spam the port
        modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?
        last_ms = ms;
    }

    // Get the actual RPM from spindle encoder input when available.
    if(on_get_data) {
        float rpm = on_get_data(SpindleData_RPM)->rpm;
        vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
    }

    return vfd_state; // return previous state as we do not want to wait for the response
}

static float f2rpm (uint16_t f)
{
    return (float)f * (vfd_config.out_multiplier / vfd_config.out_divider);
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        retry_counter = 0;

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_data.rpm = f2rpm((msg->adu[3] << 8) | msg->adu[4]);
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
                retry_counter = 0;
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
    // Alarm needs to be raised directly to correctly handle an error during reset (the rt command queue is
    // emptied on a warm reset). Exception is during cold start, where alarms need to be queued.
    if(sys.cold_start)
        vfd_failed(false);
    else if ((vfd_response_t)context > 0) {

        // when RX exceptions during one of the VFD messages, need to retry.

        switch((vfd_response_t)context) {

            case VFD_SetRPM:
//              modbus_reset();
                retry_counter++;
                spindleSetRPM(max(spindle_data.rpm_programmed, 0.0f), false);
                break;

            case VFD_GetRPM:
//              modbus_reset();
//              spindleGetState(); no need to retry?
                break;

            default:
                break;
        }

        if (retry_counter >= VFD_RETRIES) {
            system_raise_alarm(Alarm_Spindle);
            retry_counter = 0;
        }

    } else {
        retry_counter = 0;
        system_raise_alarm(Alarm_Spindle);
    }
}

static void OnReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:MODVFD v0.03]" ASCII_EOL);
}

static void onDriverReset (void)
{
    retry_counter = 0;

    driver_reset();
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

//        spindleGetMaxRPM();

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

void vfd_modvfd_init (void)
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

    if((spindle_id = vfd_register(&spindle, "MODVFD")) != -1) {

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = OnReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onDriverReset;
    }
}
#endif
