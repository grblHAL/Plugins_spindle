/*
  pwm.c - additional PWM spindle.

  NOTE: this spindle is not capable of driving a laser mode spindle.

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

#if SPINDLE_ENABLE & ((1<<SPINDLE_PWM2)|(1<<SPINDLE_PWM2_NODIR))

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

static uint8_t port_pwm = 0, port_on = 0, port_dir = IOPORT_UNASSIGNED;
static xbar_t pwm_port;
static spindle_id_t spindle_id = -1;
static spindle1_pwm_settings_t *spindle_config;
static spindle_state_t spindle_state = {0};

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    spindle_state = state;

    if(state.on && port_dir != IOPORT_UNASSIGNED)
        ioport_digital_out(port_dir, state.ccw);

    ioport_digital_out(port_on, state.on);
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle_state;
}

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    ioport_analog_out(port_pwm, rpm);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    spindle_state = state;

    if(state.on && port_dir != IOPORT_UNASSIGNED)
        ioport_digital_out(port_dir, state.ccw);

    ioport_digital_out(port_on, state.on);
    ioport_analog_out(port_pwm, rpm);
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    static bool config_ok = false;

    if(spindle == NULL)
        return false;

    pwm_config_t config;

    config.freq_hz = spindle_config->cfg.pwm_freq;
    config.min = spindle_config->cfg.rpm_min;
    config.max = spindle_config->cfg.rpm_max;
    config.min_value = spindle_config->cfg.pwm_min_value;
    config.max_value = spindle_config->cfg.pwm_max_value;
    config.off_value = spindle_config->cfg.pwm_off_value;
    config.invert = Off; // TODO: add setting

    spindle->cap.direction = port_dir != IOPORT_UNASSIGNED;
    spindle->cap.rpm_range_locked = On;
    spindle->rpm_min = spindle_config->cfg.rpm_min;
    spindle->rpm_max = spindle_config->cfg.rpm_max;

    if(config_ok) {
        spindle->set_state(NULL, (spindle_state_t){0}, 0.0f);
        system_add_rt_report(Report_Spindle);
    } else
        config_ok = true;

    spindle->set_state = pwm_port.config(&pwm_port, &config, false) ? spindleSetStateVariable : spindleSetState;

    return true;
}

static const spindle_ptrs_t spindle = {
    .type = SpindleType_PWM,
    .ref_id = SPINDLE_PWM2,
    .cap = {
#if !(SPINDLE_ENABLE & 1<<SPINDLE_PWM2_NODIR)
        .direction = On,
#endif
        .variable = On,
//          .pwm_invert = On,
        .gpio_controlled = On
    },
    .config = spindleConfig,
    .update_rpm = spindleSetSpeed,
    .set_state = spindleSetStateVariable,
    .get_state = spindleGetState
};

static void pwm_spindle_register (void)
{
    if((spindle_id = spindle_register(&spindle, "PWM2")) != -1)
        spindleSetState(NULL, spindle_state, 0.0f);
    else
        task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");
}

static void spindle_settings_changed (spindle1_pwm_settings_t *settings)
{
    static bool init_ok = false;

    if(!init_ok) {

        bool ok = false;
        xbar_t *port;

        init_ok = true;
        port_pwm = spindle_config->port_pwm;
        port_on = spindle_config->port_on;
#if (SPINDLE_ENABLE & 1<<SPINDLE_PWM2_NODIR)
        if(spindle_config->port_dir != IOPORT_UNASSIGNED)
            spindle_config->port_dir = IOPORT_UNASSIGNED;
#endif
        port_dir = spindle_config->port_dir;

        if((port = ioport_get_info(Port_Analog, Port_Output, port_pwm))) {

            memcpy(&pwm_port, port, sizeof(xbar_t));

            if((ok = pwm_port.cap.pwm && ioport_claim(Port_Analog, Port_Output, &port_pwm, "Spindle PWM"))) {
                ok = ioport_claim(Port_Digital, Port_Output, &port_on, "PWM spindle on");
#if !(SPINDLE_ENABLE & 1<<SPINDLE_PWM2_NODIR)
                ok = ok && (port_dir == IOPORT_UNASSIGNED || ioport_claim(Port_Digital, Port_Output, &port_dir, "PWM spindle dir"));
#endif
            }
        }

        if(ok)
            pwm_spindle_register();
        else
            task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");
    }

    spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
}

void pwm_spindle_init (void)
{
    if((spindle_config = spindle1_settings_add(true)))
        spindle1_settings_register(spindle.cap, spindle_settings_changed);
    else
        task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");
}

#endif
