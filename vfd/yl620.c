/*
  yl620.c - Yalang VFD spindle support

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

/*
    Manual Configuration required for the YL620
    Parameter number        Description                     Value
    -------------------------------------------------------------------------------
    P00.00                  Main frequency                  400.00Hz (match to your spindle)
    P00.01                  Command source                  3

    P03.00                  RS485 Baud rate                 3 (9600)
    P03.01                  RS485 address                   1
    P03.02                  RS485 protocol                  2
    P03.08                  Frequency given lower limit     100.0Hz (match to your spindle cooling-type)
    ===============================================================================================================
    RS485 communication is standard Modbus RTU
    Therefore, the following operation codes are relevant:
    0x03:   read single holding register
    0x06:   write single holding register
    Given a parameter Pnn.mm, the high byte of the register address is nn,
    the low is mm.  The numbers nn and mm in the manual are given in decimal,
    so P13.16 would be register address 0x0d10 when represented in hex.
    Holding register address                Description
    ---------------------------------------------------------------------------
    0x0000                                  main frequency
    0x0308                                  frequency given lower limit
    0x2000                                  command register (further information below)
    0x2001                                  Modbus485 frequency command (x0.1Hz => 2500 = 250.0Hz)
    0x200A                                  Target frequency
    0x200B                                  Output frequency
    0x200C                                  Output current
    Command register at holding address 0x2000
    --------------------------------------------------------------------------
    bit 1:0             b00: No function
                        b01: shutdown command
                        b10: start command
                        b11: Jog command
    bit 3:2             reserved
    bit 5:4             b00: No function
                        b01: Forward command
                        b10: Reverse command
                        b11: change direction
    bit 7:6             b00: No function
                        b01: reset an error flag
                        b10: reset all error flags
                        b11: reserved
*/

#include "../shared.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_YL620A)

#include <math.h>
#include <string.h>

#include "spindle.h"

static uint32_t modbus_address, rpm_max = 0;
static uint16_t retry_counter = 0;
static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static spindle_get_data_ptr on_get_data = NULL;
static on_report_options_ptr on_report_options;
static on_spindle_select_ptr on_spindle_select;
static on_spindle_selected_ptr on_spindle_selected;
//static driver_reset_ptr driver_reset;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

// TODO: this should be a mechanism to read max RPM from the VFD in order to configure RPM/Hz instead of above define.


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
    uint16_t data = ((uint32_t)(rpm) * 10) / vfd_config.vfd_rpm_hz;

    modbus_message_t rpm_cmd = {
        .context = (void *)VFD_SetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x01,
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
        spindle_data.rpm_low_limit = rpm * (1.0f - (settings.spindle.at_speed_tolerance / 100));
        spindle_data.rpm_high_limit = rpm * (1.0f + (settings.spindle.at_speed_tolerance / 100));
    }

    retries = 0;
}

static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    static uint_fast8_t retries = 0;

    bool ok;
    uint8_t runstop = 0;
    uint8_t direction = 0;

    UNUSED(spindle);

    if(retries)
        return; // block reentry

    if(!state.on || rpm == 0.0f)
        runstop = 0x1;
    else
        runstop = 0x2;

    if(state.ccw)
        direction = 0x20;
    else
        direction = 0x10;

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = direction|runstop,
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
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    UNUSED(spindle);

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x20,
        .adu[3] = 0x0B,
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

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_data.rpm = (float)((msg->adu[3] << 8) | msg->adu[4]) * vfd_config.vfd_rpm_hz / 10;
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
                retry_counter = 0;
                break;

            case VFD_GetMaxRPM:
                rpm_max = (msg->adu[4] << 8) | msg->adu[5];
                retry_counter = 0;
                break;

            case VFD_SetStatus:
                //add check here to ensure command was successful, retry if not.
                retry_counter = 0;
                break;

            case VFD_SetRPM:
                //add check here to ensure command was successful, retry if not.
                retry_counter = 0;
                break;

            default:
                retry_counter = 0;
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

        // when RX exceptions during some of the VFD messages, need to retry.

        switch((vfd_response_t)context) {

            case VFD_SetRPM:
    //          modbus_reset();
                retry_counter++;
                spindleSetRPM(max(spindle_data.rpm_programmed, 0.0f), false);
                break;

            case VFD_GetRPM:
    //          modbus_reset();
//                spindleGetState(); no need to retry?
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

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Yalang VFD YL620A v0.02]" ASCII_EOL);
}

/*
void onDriverReset (void)
{
    driver_reset();
}
*/

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

void vfd_yl620_init (void)
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

    if((spindle_id = vfd_register(&spindle, "Yalang YS620")) != -1) {

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        //driver_reset = hal.driver_reset;
        //hal.driver_reset = onDriverReset;
    }
}

#endif
