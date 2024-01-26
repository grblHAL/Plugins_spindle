/*

  stepper.c - stepper motor spindle driver

  !!! EXPERIMENTAL !!!

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

*/

#include <math.h>

#include "shared.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_STEPPER)

#if N_AXIS < 4
//#error Stepper spindle can only bind to an axis > Z axis!
#endif

#include "grbl/stepper2.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

static spindle_id_t spindle_id = -1;
static uint8_t axis_idx = N_AXIS - 1;
static st2_motor_t *motor;
static spindle_data_t spindle_data = {0};

static on_spindle_selected_ptr on_spindle_selected;
static on_execute_realtime_ptr on_execute_realtime = NULL, on_execute_delay;

static void onExecuteRealtime (uint_fast16_t state)
{
    st2_motor_run(motor);

    on_execute_realtime(state);
}

static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    spindle_data.rpm = rpm;
    st2_motor_set_speed(motor, rpm);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    if(state.on) {
        if(st2_motor_running(motor)) {
            if(state.ccw != spindle_data.state_programmed.ccw) {
                st2_motor_stop(motor);
                while(st2_motor_running(motor))
                    onExecuteRealtime(state_get());
                st2_motor_move(motor, state.ccw ? -1.0f : 1.0f, rpm, Stepper2_InfiniteSteps);
            } else
                st2_motor_set_speed(motor, rpm);
        } else
            st2_motor_move(motor, state.ccw ? -1.0f : 1.0f, rpm, Stepper2_InfiniteSteps);
    } else
        st2_motor_stop(motor);

    if(settings.spindle.at_speed_tolerance > 0.0f) {
        float tolerance = rpm * settings.spindle.at_speed_tolerance / 100.0f;
        spindle_data.rpm_low_limit = rpm - tolerance;
        spindle_data.rpm_high_limit = rpm + tolerance;
    }
    spindle_data.state_programmed.on = state.on;
    spindle_data.state_programmed.ccw = state.ccw;
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    return true;
}

#if SPINDLE_SYNC_ENABLE

int64_t offset = 0;

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    int64_t position = st2_get_position(motor) - offset;

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = (uint32_t)(position / settings.axis[0].steps_per_mm);
            spindle_data.pulse_count = position;
            break;

        case SpindleData_RPM:
            //if(!stopped)
            //    spindle_data.rpm = ;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)position / (float)settings.axis[0].steps_per_mm;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    offset = st2_get_position(motor);
}

#endif // SPINDLE_SYNC_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    UNUSED(spindle);

    state.on = st2_motor_running(motor);

//    state.value ^= settings.spindle.invert.mask;

    state.ccw = spindle_data.state_programmed.ccw;
    state.at_speed = st2_motor_cruising(motor);

    return state;
}

static void onExecuteDelay (uint_fast16_t state)
{
    st2_motor_run(motor);

    on_execute_delay(state);
}

static void stepper_spindle_selected (spindle_ptrs_t *spindle)
{
    if(on_spindle_selected)
        on_spindle_selected(spindle);

#if SPINDLE_SYNC_ENABLE
    hal.spindle_data.get = spindleGetData;
    hal.spindle_data.reset = spindleDataReset;
#endif
}
/*
static void raise_alarm (sys_state_t state)
{
    system_raise_alarm(Alarm_Spindle);
}
*/

void stepper_spindle_init (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Stepper,
        .cap.variable = On,
        .cap.at_speed = On,
        .cap.direction = On,
        .cap.gpio_controlled = On,
        .config = spindleConfig,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .update_rpm = spindleUpdateRPM
    };

    if(hal.get_micros && hal.stepper.output_step && (motor = st2_motor_init(axis_idx, true)) && (spindle_id = spindle_register(&spindle, "Stepper")) != -1) {

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = stepper_spindle_selected;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = onExecuteRealtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = onExecuteDelay;

    } else
        protocol_enqueue_foreground_task(report_warning, "Stepper spindle has been disabled!");
}

#endif
