/*

  stepper.c - stepper motor spindle driver

  !!! EXPERIMENTAL !!!

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
*/

#include <math.h>

#include "shared.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_STEPPER)

#if N_AXIS < 4
#error Stepper spindle can only bind to an axis > Z axis!
#endif

#include "grbl/stepper2.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

static spindle_id_t spindle_id = -1;
static const uint8_t axis_idx = N_AXIS - 1, axis_mask = 1 << (N_AXIS - 1);
static int64_t offset = 0;
static bool stopping = false, running = false;
static st2_motor_t *motor;
static spindle_data_t spindle_data = {0};
static axes_signals_t steppers_enabled = {0};

static on_execute_realtime_ptr on_execute_realtime = NULL, on_execute_delay;
static stepper_enable_ptr stepper_enable;
static settings_changed_ptr settings_changed;

static void stepperEnable (axes_signals_t enable, bool hold)
{
    steppers_enabled = enable;

    if(running)
        enable.mask |= axis_mask;

    stepper_enable(enable, hold);
}

static void onSpindleStopped (void *data)
{
    if(stopping) {

        stopping = running = false;
        hal.stepper.enable(steppers_enabled, false);

        if(hal.stepper.claim_motor && settings.stepper_spindle_flags.allow_axis_control) {

            hal.stepper.claim_motor(axis_idx, false);

            if(settings.stepper_spindle_flags.sync_position) {

                spindle_ptrs_t *spindle;

                if((spindle = spindle_get(spindle_id)) && spindle->get_data) {

                    float ftmp;

                    if(modff(settings.axis[axis_idx].steps_per_mm, &ftmp) == 0.0f)
                        sys.position[axis_idx] = (int32_t)((st2_get_position(motor) - offset) % (int64_t)settings.axis[axis_idx].steps_per_mm);

                    else {

                        double pos, tmp;
                        pos = modf((double)(st2_get_position(motor) - offset) / (double)settings.axis[axis_idx].steps_per_mm, &tmp);

                        sys.position[axis_idx] = lround(pos * (double)settings.axis[axis_idx].steps_per_mm);
                    }
                    sync_position();
                }
            }
        }
    }
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

        if(rpm > 0.0f) {
            running = true;
            stopping = false;
        }

        hal.stepper.enable(steppers_enabled, false);

        if(hal.stepper.claim_motor)
            hal.stepper.claim_motor(axis_idx, true);

        if(st2_motor_running(motor)) {
            if(state.ccw != spindle_data.state_programmed.ccw) {
                st2_motor_stop(motor);
                while(st2_motor_running(motor)) {
                	if(on_execute_realtime)
                		onExecuteRealtime(state_get());
                	// else run main loop?
                }
                st2_motor_move(motor, state.ccw ? -1.0f : 1.0f, rpm, Stepper2_InfiniteSteps);
            } else
                st2_motor_set_speed(motor, rpm);
        } else {
            if(settings.stepper_spindle_flags.sync_position)
                st2_set_position(motor, (int64_t)sys.position[axis_idx] + offset);
            st2_motor_move(motor, state.ccw ? -1.0f : 1.0f, rpm, Stepper2_InfiniteSteps);
        }
    } else
        stopping = st2_motor_stop(motor);

    spindle_set_at_speed_range(spindle, &spindle_data, rpm);

    spindle_data.state_programmed.on = state.on;
    spindle_data.state_programmed.ccw = state.ccw;
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    return st2_motor_bind_spindle(axis_idx);
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    uint32_t position = (uint32_t)llabs(st2_get_position(motor) - offset);

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = (uint32_t)floorf((float)position / settings.axis[axis_idx].steps_per_mm);
            spindle_data.pulse_count = position;
            break;

        case SpindleData_RPM:
            spindle_data.rpm = st2_get_speed(motor);
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)position / settings.axis[axis_idx].steps_per_mm;
            break;

        case SpindleData_AtSpeed:
            spindle_data.state_programmed.at_speed = running ? st2_motor_cruising(motor) : !running;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    offset = st2_get_position(motor);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    UNUSED(spindle);

    state.on = spindle_data.state_programmed.on;
    state.ccw = spindle_data.state_programmed.ccw;
    state.at_speed = spindle->get_data(SpindleData_AtSpeed)->state_programmed.at_speed;

    return state;
}

static void settingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

    spindle_ptrs_t *spindle = spindle_get_hal(spindle_id, SpindleHAL_Configured);

    if(changed.spindle || spindle->rpm_max != settings->axis[axis_idx].max_rate) {

        spindle_ptrs_t *spindle_hal;

        spindle->rpm_min = settings->pwm_spindle.rpm_min;
        spindle->rpm_max = min(settings->pwm_spindle.rpm_max, settings->axis[axis_idx].max_rate);
        spindle->at_speed_tolerance = settings->spindle.at_speed_tolerance;
        spindle_data.at_speed_enabled = settings->spindle.at_speed_tolerance >= 0.0f;

        if((spindle_hal = spindle_get_hal(spindle_id, SpindleHAL_Active))) {
            spindle_hal->rpm_min = spindle->rpm_min;
            spindle_hal->rpm_max = spindle->rpm_max;
            spindle_hal->at_speed_tolerance = spindle->at_speed_tolerance;
        }
    }

    if(motor && hal.stepper.claim_motor) {
        if(!settings->stepper_spindle_flags.allow_axis_control)
            hal.stepper.claim_motor(axis_idx, true);
        else if(!running)
            hal.stepper.claim_motor(axis_idx, false);
    }
}

#ifdef GRBL_ESP32

static void esp32_spindle_off (spindle_ptrs_t *spindle)
{
    stopping = st2_motor_stop(motor);
}

#endif

PROGMEM static const setting_detail_t spindle_setting_detail[] = {
    { Setting_StepperSpindle_Options, Group_Spindle, "Stepper spindle options", NULL, Format_Bitfield, "Allow axis control,Sync position", NULL, NULL, Setting_IsExtended, &settings.stepper_spindle_flags.mask, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

PROGMEM static const setting_descr_t spindle_setting_descr[] = {
    { Setting_StepperSpindle_Options, "Allow axis control is for enabling axis motion commands when the spindle is stopped.\\n"
                                      "Sync position syncs the position within one turn of the spindle."
    }
};

#endif

static void _settings_restore (void)
{
//    settings_write_global();
}

static void _settings_load (void)
{
}

void stepper_spindle_init (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Stepper,
        .ref_id = SPINDLE_STEPPER,
        .cap = {
            .variable = On,
            .at_speed = On,
            .direction = On,
            .rpm_range_locked = On,
            .gpio_controlled = On
        },
        .config = spindleConfig,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
#ifdef GRBL_ESP32
        .esp32_off = esp32_spindle_off,
#endif
        .get_data = spindleGetData,
        .reset_data = spindleDataReset,
        .update_rpm = spindleUpdateRPM
    };

    static setting_details_t setting_details = {
        .is_core = true,
        .settings = spindle_setting_detail,
        .n_settings = sizeof(spindle_setting_detail) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = spindle_setting_descr,
        .n_descriptions = sizeof(spindle_setting_descr) / sizeof(setting_descr_t),
#endif
        .save = settings_write_global,
        .load = _settings_load,
        .restore = _settings_restore
    };

    if((motor = st2_motor_init(axis_idx, true)) && (spindle_id = spindle_register(&spindle, "Stepper")) != -1) {

        settings_register(&setting_details);

        if(st2_motor_poll(motor)) {

            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = onExecuteRealtime;

            on_execute_delay = grbl.on_execute_delay;
            grbl.on_execute_delay = onExecuteDelay;
        }

        st2_motor_register_stopped_callback(motor, onSpindleStopped);

        stepper_enable = hal.stepper.enable;
        hal.stepper.enable = stepperEnable;

        settings_changed = hal.settings_changed;
        hal.settings_changed = settingsChanged;

    } else
        task_run_on_startup(report_warning, "Stepper spindle has been disabled!");
}

#endif
