/*

  stepper.c - stepper motor spindle driver

  !!! EXPERIMENTAL !!!

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#include "shared.h"

#if VFD_ENABLE == SPINDLE_ALL || VFD_ENABLE == SPINDLE_STEPPER

#include "grbl/stepper2.h"
#include "grbl/protocol.h"

static spindle_id_t spindle_id = -1;
static uint8_t axis_idx = N_AXIS - 1;
static st2_motor_t *motor;
static spindle_data_t spindle_data = {0};

static on_spindle_selected_ptr on_spindle_selected;
static on_execute_realtime_ptr on_execute_realtime = NULL, on_execute_delay;

static void spindleUpdateRPM (float rpm)
{
    st2_motor_set_speed(motor, rpm * 360.0f);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if(state.on) {
        if(st2_motor_running(motor))
            st2_motor_set_speed(motor, rpm * 360.0f);
        else
            st2_motor_move(motor, state.ccw ? -1.0f : 1.0f, rpm * 360.0f, Stepper2_InfiniteSteps);
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

#if xSPINDLE_SYNC_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;

    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = RPM_TIMER_COUNT - spindle_encoder.timer.last_pulse;

    // if 16 bit RPM timer and RPM_TIMER_COUNT < spindle_encoder.timer.last_pulse then what?

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindle_encoder.spin_lock);

    uint32_t timeout = uwTick + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && uwTick <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }

    RPM_TIMER->EGR |= TIM_EGR_UG; // Reload RPM timer
    RPM_COUNTER->CR1 &= ~TIM_CR1_CEN;

#if RPM_COUNTER_N == 2
    rpm_timer_ovf = 0;
#endif

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->EGR |= TIM_EGR_UG;
    RPM_COUNTER->CCR1 = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CR1 |= TIM_CR1_CEN;
}

#endif // SPINDLE_SYNC_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.on = st2_motor_running(motor);

//    state.value ^= settings.spindle.invert.mask;

    state.ccw = spindle_data.state_programmed.ccw;
    state.at_speed = st2_motor_cruising(motor);

    return state;
}

static void onExecuteRealtime (uint_fast16_t state)
{
    st2_motor_run(motor);

    on_execute_realtime(state);
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
}
/*
static void raise_alarm (sys_state_t state)
{
    system_raise_alarm(Alarm_Spindle);
}
*/
static void warn_disabled (sys_state_t state)
{
    report_message("Stepper spindle has been disabled!", Message_Warning);
}

void stepper_spindle_init (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Stepper,
        .cap.variable = On,
        .cap.at_speed = On,
        .cap.direction = On,
        .config = spindleConfig,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .update_rpm = spindleUpdateRPM
    };

    if(hal.get_micros && hal.stepper.output_step && (motor = st2_motor_init(axis_idx)) && (spindle_id = spindle_register(&spindle, "Stepper")) != -1) {

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = stepper_spindle_selected;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = onExecuteRealtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = onExecuteDelay;

    } else
        protocol_enqueue_rt_command(warn_disabled);
}

#endif
