/*

  pwm_clone.c - "clone" of the driver PWM spindle that uses the direction signal
                 to switch between two spindles.

  NOTE: this spindle cannot be active at the same time as the driver PWM spindle!

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

#if (SPINDLE_ENABLE & (1<<SPINDLE_PWM0)) && (SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE))

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef struct {
    uint8_t port;
    spindle_settings_t clone;
} spindle_driver_settings_t;

//static uint8_t port = 255;
static spindle_id_t spindle_id;
static spindle_ptrs_t spindle1;
static spindle_driver_settings_t spindle_config;
static spindle_state_t spindle0_state = {0}, spindle1_state = {0};
static spindle_pwm_t pwm_data;
static nvs_address_t nvs_address;
//static char max_dport[4];
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
    spindle->rpm_min = spindle_config.clone.rpm_min;
    spindle->rpm_max = spindle_config.clone.rpm_max;

    if(spindle0 && spindle0->context) {
        spindle->context = &pwm_data;
        spindle_config.clone.pwm_freq = settings.spindle.pwm_freq;
        spindle_precompute_pwm_values(spindle, &pwm_data, &spindle_config.clone, ((spindle_pwm_t *)spindle0->context)->f_clock);
    }

    return spindle->context != NULL;
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
        ((spindle_pwm_t *)spindle->context)->cloned = On;
        if(spindle->context) {
            spindle1.context = &pwm_data;
            spindle_config.clone.pwm_freq = settings.spindle.pwm_freq;
            spindle_precompute_pwm_values(&spindle1, &pwm_data, &spindle_config.clone, ((spindle_pwm_t *)spindle->context)->f_clock);
        } else
            spindle1.context = NULL;
    }
}
/*
static void warn_disabled (sys_state_t state)
{
    report_message("Cloned PWM spindle failed initialization!", Message_Warning);
}
*/
#if ENABLE_SPINDLE_LINEARIZATION

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindle1Piece1;
    float rpm, start, end;

    if(*svalue == '\0' || (svalue[0] == '0' && svalue[1] == '\0')) {
        settings.spindle.pwm_piece[idx].rpm = NAN;
        settings.spindle.pwm_piece[idx].start =
        settings.spindle.pwm_piece[idx].end = 0.0f;
    } else if(sscanf(svalue, "%f,%f,%f", &rpm, &start, &end) == 3) {
        settings.spindle.pwm_piece[idx].rpm = rpm;
        settings.spindle.pwm_piece[idx].start = start;
        settings.spindle.pwm_piece[idx].end = end;
//??       if(idx == 0)
//            settings.spindle.rpm_min = rpm;
    } else
        return Status_SettingValueOutOfRange;

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[40];

    uint32_t idx = id - Setting_LinearSpindle1Piece1;

    if(isnan(settings.spindle.pwm_piece[idx].rpm))
        *buf = '\0';
    else
        snprintf(buf, sizeof(buf), "%g,%g,%g", settings.spindle.pwm_piece[idx].rpm, settings.spindle.pwm_piece[idx].start, settings.spindle.pwm_piece[idx].end);

    return buf;
}

#endif

static const setting_detail_t vfd_settings[] = {
//    { Setting_Spindle_OnPort, Group_AuxPorts, "Spindle on port", NULL, Format_Int8, "##0", "0", max_dport, Setting_NonCore, &spindle_config.on_port, NULL, NULL, { .reboot_required = On } },
    { Setting_RpmMax1, Group_Spindle, "Maximum spindle 1 speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &spindle_config.clone.rpm_max, NULL, NULL },
    { Setting_RpmMin1, Group_Spindle, "Minimum spindle 1 speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &spindle_config.clone.rpm_min, NULL, NULL },
//    { Setting_Mode1, Group_General, "Mode of operation", NULL, Format_RadioButtons, "Normal,Laser mode", NULL, NULL, Setting_IsLegacyFn, set_mode, get_int, NULL },
    { Setting_PWMOffValue1, Group_Spindle, "Spindle 1 PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.clone.pwm_off_value, NULL, NULL },
    { Setting_PWMMinValue1, Group_Spindle, "Spindle 1 PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.clone.pwm_min_value, NULL, NULL },
    { Setting_PWMMaxValue1, Group_Spindle, "Spindle 1 PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.clone.pwm_max_value, NULL, NULL }
#if ENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindle1Piece1, Group_Spindle, "Spindle 1 linearisation, 1st point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindle1Piece2, Group_Spindle, "Spindle 1 linearisation, 2nd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindle1Piece3, Group_Spindle, "Spindle 1 linearisation, 3rd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindle1Piece4, Group_Spindle, "Spindle 1 linearisation, 4th point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t spindle_settings_descr[] = {
    { Setting_Spindle_OnPort, "Spindle  0 (default spindle) VFD ModBus address" },
    { Setting_RpmMax1, "Maximum spindle 1 speed, can be overridden by spindle plugins." },
    { Setting_RpmMin1, "Minimum spindle 1 speed, can be overridden by spindle plugins." },
    { Setting_Mode1, "Laser mode: consecutive G1/2/3 commands will not halt when spindle speed is changed." },
    { Setting_PWMOffValue1, "Spindle 1 PWM off value in percent (duty cycle)." },
    { Setting_PWMMinValue1, "Spindle 1 PWM min value in percent (duty cycle)." },
    { Setting_PWMMaxValue1, "Spindle 1 PWM max value in percent (duty cycle)." }
#if ENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindle1Piece1, "Comma separated list of values: RPM_MIN, RPM_LINE_A1, RPM_LINE_B1, set to blank to disable." },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindle1Piece2, "Comma separated list of values: RPM_POINT12, RPM_LINE_A2, RPM_LINE_B2, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindle1Piece3, "Comma separated list of values: RPM_POINT23, RPM_LINE_A3, RPM_LINE_B3, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindle1Piece4, "Comma separated list of values: RPM_POINT34, RPM_LINE_A4, RPM_LINE_B4, set to blank to disable." },
  #endif
#endif
};
#endif

static void spindle_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(spindle1.context)
        Spindle1Configure(&spindle1);
}

static void spindle_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_config, sizeof(spindle_driver_settings_t), true);
}

static void spindle_settings_restore (void)
{
    static const spindle_settings_t defaults = {
        .rpm_max = DEFAULT_SPINDLE_RPM_MAX,
        .rpm_min = DEFAULT_SPINDLE_RPM_MIN,
        .flags.pwm_disable = false,
        .flags.enable_rpm_controlled = DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED,
        .invert.on = DEFAULT_INVERT_SPINDLE_ENABLE_PIN,
        .invert.ccw = DEFAULT_INVERT_SPINDLE_CCW_PIN,
        .invert.pwm = DEFAULT_INVERT_SPINDLE_PWM_PIN,
        .pwm_freq = DEFAULT_SPINDLE_PWM_FREQ,
        .pwm_off_value = DEFAULT_SPINDLE_PWM_OFF_VALUE,
        .pwm_min_value = DEFAULT_SPINDLE_PWM_MIN_VALUE,
        .pwm_max_value = DEFAULT_SPINDLE_PWM_MAX_VALUE,
        .at_speed_tolerance = DEFAULT_SPINDLE_AT_SPEED_TOLERANCE,
        .ppr = DEFAULT_SPINDLE_PPR,
        .pid.p_gain = DEFAULT_SPINDLE_P_GAIN,
        .pid.i_gain = DEFAULT_SPINDLE_I_GAIN,
        .pid.d_gain = DEFAULT_SPINDLE_D_GAIN,
        .pid.i_max_error = DEFAULT_SPINDLE_I_MAX,
#if ENABLE_SPINDLE_LINEARIZATION
  #if SPINDLE_NPWM_PIECES > 0
        .pwm_piece[0] = { .rpm = DEFAULT_RPM_POINT01, .start = DEFAULT_RPM_LINE_A1, .end = DEFAULT_RPM_LINE_B1 },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
        .pwm_piece[1] = { .rpm = DEFAULT_RPM_POINT12, .start = DEFAULT_RPM_LINE_A2, .end = DEFAULT_RPM_LINE_B2 },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
        .pwm_piece[2] = { .rpm = DEFAULT_RPM_POINT23, .start = DEFAULT_RPM_LINE_A3, .end = DEFAULT_RPM_LINE_B3 },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
        .pwm_piece[3] = { .rpm = DEFAULT_RPM_POINT34, .start = DEFAULT_RPM_LINE_A4, .end = DEFAULT_RPM_LINE_B4 },
  #endif
#else
  #if SPINDLE_NPWM_PIECES > 0
        .pwm_piece[0] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
        .pwm_piece[1] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
        .pwm_piece[2] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
        .pwm_piece[3] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
#endif
    };

    spindle_config.port = hal.port.num_digital_out - 1;
    memcpy(&spindle_config.clone, &defaults, sizeof(spindle_settings_t));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_config, sizeof(spindle_driver_settings_t), true);
}

static void spindle_settings_load (void)
{
//    bool ok;
    if((hal.nvs.memcpy_from_nvs((uint8_t *)&spindle_config, nvs_address, sizeof(spindle_driver_settings_t), true) != NVS_TransferResult_OK))
        spindle_settings_restore();


    /*
    port = spindle_config.port;


    strcpy(max_dport, uitoa(ioports_available(Port_Digital, Port_Output) - 1));

    ok = ioport_claim(Port_Digital, Port_Output, &run.on_port, "Spindle on");

    if(ok)
        onoff_spindle_register();
    else
        protocol_enqueue_foreground_task(warn_disabled);*/
}

static setting_details_t vfd_setting_details = {
    .settings = vfd_settings,
    .n_settings = sizeof(vfd_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = spindle_settings_descr,
    .n_descriptions = sizeof(spindle_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = spindle_settings_load,
    .restore = spindle_settings_restore,
    .save = spindle_settings_save,
    .on_changed = spindle_settings_changed
};

void cloned_spindle_init (void)
{
    spindle_ptrs_t *pwm_spindle = spindle_get_hal(0, SpindleHAL_Raw);
    if(pwm_spindle &&
        pwm_spindle->type == SpindleType_PWM &&
         pwm_spindle->cap.direction &&
          pwm_spindle->update_pwm &&
           (nvs_address = nvs_alloc(sizeof(spindle_driver_settings_t)))) {

        settings_register(&vfd_setting_details);

        set_state = pwm_spindle->set_state;
        memcpy(&spindle1, pwm_spindle, sizeof(spindle_ptrs_t));
        spindle1.config = NULL;
        spindle1.update_pwm = NULL;
        spindle1.cap.laser = Off;
        spindle1.cap.direction = Off;
        spindle1.config = Spindle1Configure;
        spindle1.set_state = spindle1SetState;
        spindle1.get_state = spindle1GetState;
        spindle_id = spindle_register(&spindle1, "Cloned PWM spindle");

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;
    }
}

#elif SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE)
#error "Cannot clone when the base spindle is not enabled!"
#endif
