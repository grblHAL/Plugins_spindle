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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if VFD_ENABLE == 3

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

#include "modbus.h"

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Huanyang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

/* Read Registers:
	0x2100 = status word 1
	0x2101 = status word 2
	0x2102 = frequency command
	0x2103 = actual frequency
	0x2104 = output current
	0x2105 = DC bus voltage
	0x2106 = actual output voltage
	0x2107 = actual RPM
	0x2108 + 0x2109 = scale freq (not sure what this actually is - it's the same as 0x2103)
	0x210A = power factor.  Not sure of the units (1/10 or 1/100)
	0x210B = load percentage
	0x210C = Firmware revision (never saw anything other than 0 here)
	total of 13 registers		*/
#define START_REGISTER_R	0x2100
#define NUM_REGISTERS_R		13
/* write registers:
	0x91A = Speed reference, in 1/10Hz increments
	0x91B = RUN command, 0=stop, 1=run
	0x91C = direction, 0=forward, 1=reverse
	0x91D = serial fault, 0=no fault, 1=fault (maybe can stop with this?)
	0x91E = serial fault reset, 0=no reset, 1 = reset fault
	total of 5 registers */
#define START_REGISTER_W	0x091A
#define NUM_REGISTERS_W		5


#define GS2_REG_STOP_METHOD                             0x0100
#define GS2_REG_STOP_METHOD__RAMP_TO_STOP               0
#define GS2_REG_STOP_METHOD__COAST_TO_STOP              1

#define GS2_REG_ACCELERATION_TIME_1                     0x0101

#define GS2_REG_DECELERATION_TIME_1                     0x0102

#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION           0x0605
#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__ENABLE   0
#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__DISABLE  1

#define MOTOR_RPM_HZ		60

typedef enum {
    VFD_Idle = 0,
    VFD_GetRPM,
    VFD_SetRPM,
    VFD_GetMaxRPM,
    VFD_GetMaxRPM50,
    VFD_GetStatus,
    VFD_SetStatus,
    VFD_SetAccelTime
} vfd_response_t;

static float rpm_programmed = -1.0f;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static on_report_options_ptr on_report_options;
static on_spindle_select_ptr on_spindle_select;
static driver_reset_ptr driver_reset;
static uint32_t rpm_max = 0;
static uint8_t poles = 2;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

// Read maximum configured RPM from spindle, value is used later for calculating current RPM
// In the case of the original Huanyang protocol, the value is the configured RPM at 50Hz
static void spindleGetMaxRPM (void)
{
    modbus_message_t cmd;
    
    /*modbus_message_t cmd = {
        .context = (void *)VFD_GetPoles,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x01,
        .adu[3] = 0x00,
        .adu[4] = 0x02,
        .adu[5] = 0x57,
        .tx_length = 8,
        .rx_length = 8
    };
    
    modbus_send(&cmd, &callbacks, true);*/

/*        cmd.context = (void *)VFD_GetMaxRPM;
        cmd.adu[0] = VFD_ADDRESS;
        cmd.adu[1] = ModBus_ReadHoldingRegisters;
        cmd.adu[2] = 0x00;
        cmd.adu[3] = 0x04;
        cmd.adu[4] = 0x00;
        cmd.adu[5] = 0x02;
        cmd.tx_length = 8;
        cmd.rx_length = 8;
    modbus_send(&cmd, &callbacks, true);*/
}

static void spindleSetRPM (float rpm, bool block)
{

    if (rpm != rpm_programmed) {

        uint16_t data = ((uint32_t)(rpm)*50) / MOTOR_RPM_HZ;

        //data = data*10;

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = VFD_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x20,
            .adu[3] = 0x01,
            .adu[4] = data >> 8,
            .adu[5] = data & 0xFF,
            .tx_length = 8,
            .rx_length = 8
        };        

        vfd_state.at_speed = false;

        modbus_send(&rpm_cmd, &callbacks, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            spindle_data.rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
            spindle_data.rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
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
    uint8_t runstop = 0;
    uint8_t direction = 0;

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
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = direction|runstop,
        .tx_length = 8,
        .rx_length = 8
    };

    if(vfd_state.ccw != state.ccw)
        rpm_programmed = 0.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        spindleSetRPM(rpm, true);
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x21,
        .adu[3] = 0x03,
        .adu[4] = 0x00,
        .adu[5] = 0x01,
        .tx_length = 8,
        .rx_length = 7
    };


    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?
    

    // Get the actual RPM from spindle encoder input when available.
    if(hal.spindle.get_data && hal.spindle.get_data != spindleGetData) {
        float rpm = hal.spindle.get_data(SpindleData_RPM)->rpm;
        vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
    }

    return vfd_state; // return previous state as we do not want to wait for the response
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_data.rpm = (float)((msg->adu[3] << 8) | msg->adu[4])*MOTOR_RPM_HZ/100;
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
                break;

            case VFD_GetMaxRPM:
                rpm_max = (msg->adu[4] << 8) | msg->adu[5];
                break;

            default:
                break;
        }
    }
}

static void raise_alarm (uint_fast16_t state)
{
    system_raise_alarm(Alarm_Spindle);
}

static void rx_exception (uint8_t code, void *context)
{
    // Alarm needs to be raised directly to correctly handle an error during reset (the rt command queue is
    // emptied on a warm reset). Exception is during cold start, where alarms need to be queued.
    if(sys.cold_start)
        protocol_enqueue_rt_command(raise_alarm);
    else
        system_raise_alarm(Alarm_Spindle);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:Durapulse VFD G20 v0.01]" ASCII_EOL);
    }
}

static void g20_reset (void)
{
    driver_reset();
    //spindleGetMaxRPM();
}

bool g20_spindle_select (uint_fast8_t spindle_id)
{
    static bool init_ok = false, vfd_active = false;
    static driver_cap_t driver_cap;
    static spindle_ptrs_t spindle_org;

    if(vfd_active && spindle_id != 1 && spindle_org.set_state != NULL) {

        vfd_active = false;

        gc_spindle_off();

        hal.driver_cap = driver_cap;
        memcpy(&hal.spindle, &spindle_org, sizeof(spindle_ptrs_t));
    }

    if(on_spindle_select && on_spindle_select(spindle_id))
        return true;

    if(!modbus_isup())
        return false;

    if((vfd_active = spindle_id == 1)) {

        if(hal.spindle.set_state != spindleSetState) {

            if(spindle_org.set_state == NULL) {
                driver_cap = hal.driver_cap;
                memcpy(&spindle_org, &hal.spindle, sizeof(spindle_ptrs_t));
            }

            if(spindle_org.set_state)
                gc_spindle_off();

            hal.spindle.set_state = spindleSetState;
            hal.spindle.get_state = spindleGetState;
            hal.spindle.update_rpm = spindleUpdateRPM;
            hal.spindle.reset_data = NULL;

            hal.driver_cap.variable_spindle = On;
            hal.driver_cap.spindle_at_speed = On;
            hal.driver_cap.spindle_dir = On;
        }

        if(settings.spindle.ppr == 0)
            hal.spindle.get_data = spindleGetData;

        if(!init_ok) {
            init_ok = true;
            spindleGetMaxRPM();
        }
    }

    return true;
}

void vfd_init (void)
{
    if(modbus_enabled()) {

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = g20_spindle_select;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = g20_reset;
    }
}

#endif
