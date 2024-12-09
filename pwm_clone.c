/*
  pwm_clone.c - "clone" of the driver PWM spindle that uses the direction signal
                 to switch between two spindles.

  NOTE: this spindle cannot be active at the same time as the driver PWM spindle!

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io

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

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

static spindle_id_t spindle_id;
static spindle_ptrs_t spindle1;
static spindle1_pwm_settings_t *spindle_config;
static spindle_state_t spindle0_state = {0}, spindle1_state = {0};
static spindle_pwm_t pwm_data;
static on_spindle_selected_ptr on_spindle_selected;
static spindle_set_state_ptr set_state;

static void spindle0SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    spindle0_state = state;

    state.ccw = state.on;
    state.on = Off;

    set_state(spindle, state, rpm);
}

static spindle_state_t spindle0GetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle0_state;
}

static void spindle1SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    spindle1_state = state;

    state.ccw = Off;

    set_state(spindle, state, rpm);
}

static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle1_state;
}

static bool Spindle1Configure (spindle_ptrs_t *spindle)
{
    spindle_ptrs_t *spindle0 = spindle_get_hal(0, SpindleHAL_Configured);

    spindle->cap.rpm_range_locked = On;
    spindle->rpm_min = spindle_config->cfg.rpm_min;
    spindle->rpm_max = spindle_config->cfg.rpm_max;

    if(spindle0 && spindle0->context.pwm) {
        spindle->context.pwm = &pwm_data;
        spindle_config->cfg.pwm_freq = settings.pwm_spindle.pwm_freq;
        spindle_precompute_pwm_values(spindle, &pwm_data, &spindle_config->cfg, spindle0->context.pwm->f_clock);
    }

    return spindle->context.pwm != NULL;
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(on_spindle_selected)
        on_spindle_selected(spindle);

    if(spindle->id == 0 && spindle->set_state != spindle0SetState) {
        set_state = spindle->set_state;
        spindle->set_state = spindle0SetState;
        spindle->get_state = spindle0GetState;
        spindle->cap.direction = settings.mode == Mode_Laser;
        spindle->context.pwm->flags.cloned = On;
        if(spindle->context.pwm) {
            spindle1.context.pwm = &pwm_data;
            spindle_config->cfg.pwm_freq = settings.pwm_spindle.pwm_freq;
            spindle_precompute_pwm_values(&spindle1, &pwm_data, &spindle_config->cfg, spindle->context.pwm->f_clock);
        } else
            spindle1.context.pwm = NULL;
    }
}
/*
static void warn_disabled (sys_state_t state)
{
    report_message("Cloned PWM spindle failed initialization!", Message_Warning);
}
*/

static void spindle_settings_changed (spindle1_pwm_settings_t *settings)
{
    if(spindle1.context.pwm)
        Spindle1Configure(&spindle1);
}

void cloned_spindle_init (void)
{
    spindle_ptrs_t *pwm_spindle = spindle_get_hal(0, SpindleHAL_Raw);
    if(pwm_spindle &&
        pwm_spindle->type == SpindleType_PWM &&
         pwm_spindle->cap.direction &&
          pwm_spindle->update_pwm &&
          (spindle_config = spindle1_settings_add(false))) {

        set_state = pwm_spindle->set_state;
        memcpy(&spindle1, pwm_spindle, sizeof(spindle_ptrs_t));
        spindle1.config = NULL;
        spindle1.update_pwm = NULL;
        spindle1.cap.laser = Off;
        spindle1.cap.direction = Off;
        spindle1.cap.cloned = On;
        spindle1.config = Spindle1Configure;
        spindle1.set_state = spindle1SetState;
        spindle1.get_state = spindle1GetState;
        spindle_id = spindle_register(&spindle1, "Cloned PWM spindle");

        spindle1_settings_register(spindle1.cap, spindle_settings_changed);

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;
    }
}

#elif SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE)
#error "Cannot clone when the base spindle is not enabled!"
#endif
