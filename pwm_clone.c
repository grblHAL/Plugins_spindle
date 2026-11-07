/*
  pwm_clone.c - "clone" of the driver PWM spindle that uses the direction signal
                 to switch between two spindles.

  NOTE: this spindle cannot be active at the same time as the driver PWM spindle!

  Part of grblHAL

  Copyright (c) 2023-2026 Terje Io

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

#if (SPINDLE_ENABLE & (1<<SPINDLE_PWM0)) && (SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE))

#if N_SPINDLE < 2
#error "Bad configuration, set N_SPINDLE >= 2 in grbl/config.h"
#endif

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

static spindle_ptrs_t spindle1;
static spindle1_pwm_settings_t *clone_config;
static on_spindle_select_ptr on_spindle_select;
static spindle_set_state_ptr set_state;
static spindle_get_state_ptr get_state;

static void spindle0SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    state.ccw = state.on;
    state.on = Off;

    set_state(spindle, state, rpm);
}

static spindle_state_t spindle0GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = get_state(spindle);

    state.on = spindle->param->state.on;
    state.ccw = spindle->param->state.ccw;

    return state;
}

static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = get_state(spindle);

    state.ccw = spindle->param->state.ccw;

    return state;
}

static void clone_cfg (spindle_ptrs_t *clone, spindle_ptrs_t *spindle)
{
    static spindle_pwm_t pwm_data;

    spindle->context.pwm->flags.cloned = On;
    spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

    clone->context.pwm = &pwm_data;
    clone_config->cfg.pwm_freq = spindle->context.pwm->settings->pwm_freq;
    spindle_precompute_pwm_values(clone, &pwm_data, &clone_config->cfg, spindle->context.pwm->f_clock);
    spindle_update_caps(clone, &pwm_data);
}

static bool Spindle1Configure (spindle_ptrs_t *spindle)
{
    if(spindle_select_available(SPINDLE_PWM0_CLONE)) {

        spindle_ptrs_t *pwm_spindle = spindle_get_hal(0, SpindleHAL_Configured);

        spindle->cap.rpm_range_locked = On;
        spindle->rpm_min = clone_config->cfg.rpm_min;
        spindle->rpm_max = clone_config->cfg.rpm_max;
        spindle->cap.direction = spindle->cap.laser;

        if(pwm_spindle && pwm_spindle->context.pwm)
            clone_cfg(spindle, pwm_spindle);
    }

    return spindle->context.pwm != NULL;
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    if(spindle->id == 0 && spindle->set_state != spindle0SetState) {
        if(spindle_select_available(SPINDLE_PWM0_CLONE)) {
            set_state = spindle->set_state;
            spindle->set_state = spindle0SetState;
            spindle->get_state = spindle0GetState;
            spindle->cap.direction = spindle->cap.laser;
            if(spindle->context.pwm)
                clone_cfg(&spindle1, spindle);
        }
    } else if(spindle->id == spindle1.id) {
        spindle->cap.direction = spindle->cap.laser;
        spindle->get_state = spindle1GetState;
    }

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void spindle_settings_changed (spindle1_pwm_settings_t *settings)
{
    if(spindle1.context.pwm)
        Spindle1Configure(&spindle1);
}

void cloned_spindle_init (void)
{
    spindle_ptrs_t *pwm_spindle;

    if((pwm_spindle = spindle_get_hal(0, SpindleHAL_Raw)) &&
         pwm_spindle->type == SpindleType_PWM &&
          pwm_spindle->cap.direction &&
           pwm_spindle->update_pwm &&
           (clone_config = spindle1_settings_add(false))) {

        get_state = pwm_spindle->get_state;
        set_state = pwm_spindle->set_state;

        memcpy(&spindle1, pwm_spindle, sizeof(spindle_ptrs_t));
        spindle1.ref_id = SPINDLE_PWM0_CLONE;
        spindle1.cap.clone = On;
        spindle1.cap.direction = Off;
        spindle1.context.pwm = NULL;
        spindle1.config = Spindle1Configure;
        spindle1.id = spindle_register(&spindle1, "PWM clone");

        spindle1_settings_register(spindle1.cap, spindle_settings_changed);

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;
    } else
        report_message("PWM spindle clone failed initialization!", Message_Warning);
}

#elif SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE)
#error "Cannot clone when the base spindle is not enabled!"
#endif
