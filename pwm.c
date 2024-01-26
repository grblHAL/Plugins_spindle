/*

  pwm.c - additional PWM spindle.

  NOTE: this spindle is not capable of driving a laser mode spindle.

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

#if SPINDLE_ENABLE & (1<<SPINDLE_PWM2)

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef struct {
    uint8_t port_pwm;
    uint8_t port_on;
    float port_dir; // set to -1 if unassigned
    spindle_settings_t cfg;
} spindle_driver_settings_t;

static uint8_t n_dout = 0, n_pwm_out = 0;
static uint8_t port_pwm = 0, port_on = 0, port_dir = 255;
static xbar_t pwm_port;
static spindle_id_t spindle_id = -1;
static spindle_driver_settings_t spindle_config;
static spindle_state_t spindle_state = {0};
static nvs_address_t nvs_address;
static char max_aport[4], max_dport[4];

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    spindle_state = state;

    if(state.on && port_dir != 255)
        hal.port.digital_out(port_dir, state.ccw);

    hal.port.digital_out(port_on, state.on);
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

    hal.port.analog_out(port_pwm, rpm);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    spindle_state = state;

    if(state.on && port_dir != 255)
        hal.port.digital_out(port_dir, state.ccw);

    hal.port.digital_out(port_on, state.on);
    hal.port.analog_out(port_pwm, rpm);
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    static bool config_ok = false;

    if(spindle == NULL)
        return false;

    pwm_config_t config;

    config.freq_hz = spindle_config.cfg.pwm_freq;
    config.min = spindle_config.cfg.rpm_min;
    config.max = spindle_config.cfg.rpm_max;
    config.min_value = spindle_config.cfg.pwm_min_value;
    config.max_value = spindle_config.cfg.pwm_max_value;
    config.off_value = spindle_config.cfg.pwm_off_value;
    config.invert = Off; // TODO: add setting

    spindle->cap.direction = port_dir != 255;
    spindle->cap.rpm_range_locked = On;
    spindle->rpm_min = spindle_config.cfg.rpm_min;
    spindle->rpm_max = spindle_config.cfg.rpm_max;

    if(config_ok) {
        spindle->set_state(NULL, (spindle_state_t){0}, 0.0f);
        system_add_rt_report(Report_Spindle);
    } else
        config_ok = true;

    spindle->set_state = pwm_port.config(&pwm_port, &config) ? spindleSetStateVariable : spindleSetState;

    return true;
}

static void pwm_spindle_register (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.direction = On,
        .cap.variable = On,
//        .cap.pwm_invert = On,
        .cap.gpio_controlled = On,
        .config = spindleConfig,
        .update_rpm = spindleSetSpeed,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState
    };

    if((spindle_id = spindle_register(&spindle, "PWM2")) != -1)
        spindleSetState(NULL, spindle_state, 0.0f);
    else
        protocol_enqueue_foreground_task(report_warning, "PWM2 spindle failed to initialize!");
}

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

static uint32_t get_pwm_port (setting_id_t id)
{
    return (uint32_t)spindle_config.port_pwm;
}

bool pwm_port_validate (xbar_t *properties, uint8_t port, void *data)
{
    return port == (uint8_t)((uint32_t)data);
}

static status_code_t set_pwm_port (setting_id_t id, uint_fast16_t int_value)
{
    bool ok;

    if((ok = (uint8_t)int_value == spindle_config.port_pwm ||
              ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .pwm = On, .claimable = On }, pwm_port_validate, (void *)((uint32_t)int_value))))
        spindle_config.port_pwm = (uint8_t)int_value;

    return ok ? Status_OK : Status_SettingValueOutOfRange;
}

static const setting_detail_t pwm_settings[] = {
    { Setting_Spindle_OnPort, Group_AuxPorts, "PWM2 spindle on port", NULL, Format_Int8, "##0", "0", max_dport, Setting_NonCore, &spindle_config.port_on, NULL, NULL, { .reboot_required = On } },
    { Setting_Spindle_DirPort, Group_AuxPorts, "PWM2 spindle direction port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCore, &spindle_config.port_dir, NULL, NULL, { .reboot_required = On } },
    { Setting_Spindle_PWMPort, Group_AuxPorts, "PWM2 spindle PWM port", NULL, Format_Int8, "#0", "0", max_aport, Setting_NonCoreFn, set_pwm_port, get_pwm_port, NULL, { .reboot_required = On } },
    { Setting_RpmMax1, Group_Spindle, "PWM2 spindle maximum speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &spindle_config.cfg.rpm_max, NULL, NULL },
    { Setting_RpmMin1, Group_Spindle, "PWM2 spindle minimum speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &spindle_config.cfg.rpm_min, NULL, NULL },
    { Setting_PWMFreq1, Group_Spindle, "PWM2 spindle PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL, Setting_IsExtended, &spindle_config.cfg.pwm_freq, NULL, NULL },
    { Setting_PWMOffValue1, Group_Spindle, "PWM2 spindle PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.cfg.pwm_off_value, NULL, NULL },
    { Setting_PWMMinValue1, Group_Spindle, "PWM2 spindle PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.cfg.pwm_min_value, NULL, NULL },
    { Setting_PWMMaxValue1, Group_Spindle, "PWM2 spindle PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &spindle_config.cfg.pwm_max_value, NULL, NULL }
#if xENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindle1Piece1, Group_Spindle, "PWM2 spindle linearisation, 1st point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindle1Piece2, Group_Spindle, "PWM2 spindle linearisation, 2nd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindle1Piece3, Group_Spindle, "PWM2 spindle linearisation, 3rd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindle1Piece4, Group_Spindle, "PWM2 spindle linearisation, 4th point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t spindle_settings_descr[] = {
    { Setting_Spindle_OnPort, "On/off aux port." },
    { Setting_Spindle_DirPort, "Direction aux port, set to -1 if not used." },
    { Setting_Spindle_PWMPort, "Spindle analog aux port. Must be PWM capable!" },
    { Setting_RpmMax1, "Maximum spindle speed." },
    { Setting_RpmMin1, "Minimum spindle speed." },
    { Setting_PWMFreq1, "PWM frequency." },
    { Setting_PWMOffValue1, "PWM off value in percent (duty cycle)." },
    { Setting_PWMMinValue1, "PWM min value in percent (duty cycle)." },
    { Setting_PWMMaxValue1, "PWM max value in percent (duty cycle)." }
#if xENABLE_SPINDLE_LINEARIZATION
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
    spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
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

    spindle_config.port_pwm = port_pwm;
    spindle_config.port_on = n_dout;
    spindle_config.port_dir = n_dout > 1 ? (float)(hal.port.num_digital_out - 1) : -1.0f;
    memcpy(&spindle_config.cfg, &defaults, sizeof(spindle_settings_t));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_config, sizeof(spindle_driver_settings_t), true);
}

static void spindle_settings_load (void)
{
    bool ok = false;
    xbar_t *port;

    if((hal.nvs.memcpy_from_nvs((uint8_t *)&spindle_config, nvs_address, sizeof(spindle_driver_settings_t), true) != NVS_TransferResult_OK))
        spindle_settings_restore();

    port_pwm = spindle_config.port_pwm;
    port_on = spindle_config.port_on;
    port_dir = spindle_config.port_dir == -1.0f ? 255 : (uint8_t)spindle_config.port_dir;

    if((port = hal.port.get_pin_info(Port_Analog, Port_Output, port_pwm))) {

        memcpy(&pwm_port, port, sizeof(xbar_t));

        if((ok = pwm_port.cap.pwm && ioport_claim(Port_Analog, Port_Output, &port_pwm, "Spindle PWM"))) {

            ok = ioport_claim(Port_Digital, Port_Output, &port_on, "PWM spindle on");
            ok = ok && (port_dir == 255 || ioport_claim(Port_Digital, Port_Output, &port_dir, "PWM spindle dir"));

    //        strcpy(max_dport, uitoa(ioports_available(Port_Digital, Port_Output) - 1));
        }
    }

    if(ok)
        pwm_spindle_register();
    else
        protocol_enqueue_foreground_task(report_warning, "PWM2 spindle failed to initialize!");
}

static setting_details_t pwm_setting_details = {
    .settings = pwm_settings,
    .n_settings = sizeof(pwm_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = spindle_settings_descr,
    .n_descriptions = sizeof(spindle_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = spindle_settings_load,
    .restore = spindle_settings_restore,
    .save = spindle_settings_save,
    .on_changed = spindle_settings_changed
};

bool pwm_claim (xbar_t *properties, uint8_t port, void *data)
{
    n_pwm_out++;
    port_pwm = max(port_pwm, port);

    return false;
}

static bool check_pwm_ports (void)
{
    ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .pwm = On, .claimable = On }, pwm_claim, NULL);

    return n_pwm_out != 0;
}

void pwm_spindle_init (void)
{
    if((n_dout = hal.port.num_digital_out) > 0 &&
         check_pwm_ports() &&
          (nvs_address = nvs_alloc(sizeof(spindle_driver_settings_t)))) {

        strcpy(max_aport, uitoa(port_pwm + 2));
        strcpy(max_dport, uitoa(n_dout - 1));

        settings_register(&pwm_setting_details);
    } else
        protocol_enqueue_foreground_task(report_warning, "PWM2 spindle failed to initialize!");
}

#endif
