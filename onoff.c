/*

  onoff.c - on/off + optional direction spindle driver

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

#if SPINDLE_ENABLE & ((1<<SPINDLE_ONOFF1)|(1<<SPINDLE_ONOFF1_DIR))

#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

#if SPINDLE_ENABLE & (1<<SPINDLE_ONOFF1_DIR)
#define N_PORTS 2
#else
#define N_PORTS 1
#endif

typedef struct {
    uint8_t on_port;
    uint8_t dir_port;
} onoff_spindle_settings_t;

static onoff_spindle_settings_t spindle_config, run;
static spindle_state_t spindle_state = {0};
static nvs_address_t nvs_address;
static uint8_t n_dout;
static char max_dport[4];

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    spindle_state = state;

#if SPINDLE_ENABLE & (1<<SPINDLE_ONOFF1_DIR)
    hal.port.digital_out(run.dir_port, state.ccw);
#endif
    hal.port.digital_out(run.on_port, state.on);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle_state;
}

static void onoff_spindle_register (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
#if N_PORTS == 2
        .cap.direction = On,
#endif
        .cap.gpio_controlled = On,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    if((spindle_register(&spindle, "On/off spindle")) != -1)
        spindleSetState(NULL, spindle_state, 0.0f);
    else
        protocol_enqueue_foreground_task(report_warning, "On/off spindle failed to initialize!");
}

static const setting_detail_t vfd_settings[] = {
    { Setting_Spindle_OnPort, Group_AuxPorts, "Spindle on port", NULL, Format_Int8, "#0", "0", max_dport, Setting_NonCore, &spindle_config.on_port, NULL, NULL, { .reboot_required = On } },
#if N_PORTS == 2
    { Setting_Spindle_DirPort, Group_AuxPorts, "Spindle dir port", NULL, Format_Int8, "#0", "0", max_dport, Setting_NonCore, &spindle_config.dir_port, NULL, NULL, { .reboot_required = On } }
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t spindle_settings_descr[] = {
    { Setting_Spindle_OnPort, "Spindle  0 (default spindle) VFD ModBus address" },
#if N_PORTS == 2
    { Setting_Spindle_DirPort, "Spindle 1 VFD ModBus address" }
#endif
};
#endif

static void spindle_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_config, sizeof(onoff_spindle_settings_t), true);
}

static void spindle_settings_restore (void)
{
    spindle_config.on_port = n_dout - 1;
    spindle_config.dir_port = n_dout > 1 ? n_dout - 2 : 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_config, sizeof(onoff_spindle_settings_t), true);
}

static void spindle_settings_load (void)
{
    bool ok;
    if((hal.nvs.memcpy_from_nvs((uint8_t *)&spindle_config, nvs_address, sizeof(onoff_spindle_settings_t), true) != NVS_TransferResult_OK))
        spindle_settings_restore();

    run.on_port = spindle_config.on_port;
    run.dir_port = spindle_config.dir_port;
    strcpy(max_dport, uitoa(max(n_dout, ioports_available(Port_Digital, Port_Output)) - 1));

    ok = ioport_claim(Port_Digital, Port_Output, &run.on_port, "Spindle on");
#if N_PORTS == 2
    ok = ok && ioport_claim(Port_Digital, Port_Output, &run.dir_port, "Spindle direction");
#endif
    if(ok)
        onoff_spindle_register();
    else
        protocol_enqueue_foreground_task(report_warning, "On/off spindle failed to initialize!");
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
    .save = spindle_settings_save
};

void onoff_spindle_init (void)
{
    if((n_dout = hal.port.num_digital_out) < N_PORTS)
        protocol_enqueue_foreground_task(report_warning, "On/off spindle failed to initialize!");

    else if(!ioport_can_claim_explicit()) {
        run.on_port = --hal.port.num_digital_out;
#if N_PORTS == 2
        run.dir_port = --hal.port.num_digital_out;
#endif
        onoff_spindle_register();
    } else if((nvs_address = nvs_alloc(sizeof(onoff_spindle_settings_t)))) {
        hal.port.num_digital_out -= N_PORTS;
        settings_register(&vfd_setting_details);
    } else
        protocol_enqueue_foreground_task(report_warning, "On/off spindle failed to initialize!");
}

#endif
