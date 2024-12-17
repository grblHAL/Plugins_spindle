/*
  select.c - spindle select plugin

  Part of grblHAL

  Copyright (c) 2022-2024 Terje Io

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
#include <string.h>

#include "driver.h"

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/protocol.h"
#else
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#endif

#if N_SPINDLE > 1

#define N_SPINDLE_SETTINGS 8
#define SETTING_OPTS { .subgroups = Off, .increment = 1, .reboot_required = On }

typedef struct {
    spindle_id_t ref_id; // TODO: change to uint8_t
    tool_id_t min_tool_id;
} spindle_binding_t;

static uint8_t n_spindle;
static spindle_id_t default_spindle_id;
static uint8_t ref_id_map[N_SPINDLE_SETTINGS]; // maps spindle.ref_id to spindle.id, spindle.id is the index
static spindle_binding_t spindle_setting[N_SPINDLE_SETTINGS];
static char format[110] = "";
#if N_SYS_SPINDLE == 1 && N_SPINDLE_SELECTABLE > 1
static char max_tool[] = "65535";
#endif
static nvs_address_t nvs_address;
static driver_setup_ptr driver_setup;

#if N_SYS_SPINDLE == 1

static bool select_by_tool = false;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_tool_selected_ptr on_tool_selected = NULL;

static spindle_id_t get_spindle_id (uint8_t ref_id);

static user_mcode_type_t check (user_mcode_t mcode)
{
    return mcode == Spindle_Select ? UserMCode_Normal : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == Spindle_Select) {

        if(gc_block->words.p) {
            if(isnan(gc_block->values.p))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.p) && gc_block->values.p >= 0.0f && gc_block->values.p <= 1.0f && spindle_setting[(uint32_t)gc_block->values.p].ref_id != SPINDLE_NONE))
                state = Status_GcodeValueOutOfRange;
        } else if(gc_block->words.q) {
            if(isnan(gc_block->values.q))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.q) && gc_block->values.q >= 0.0f && gc_block->values.q < (float)N_SPINDLE_SETTINGS && spindle_setting[(uint32_t)gc_block->values.q].ref_id != SPINDLE_NONE))
                state = Status_GcodeValueOutOfRange;
        } else
            state = Status_GcodeValueWordMissing;

        if(state == Status_OK && gc_block->words.p != gc_block->words.q) {
            gc_block->words.p = gc_block->words.q = Off;
            gc_block->user_mcode_sync = On;
        } else
            state = Status_GcodeValueOutOfRange;

    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == Spindle_Select) {
        if(gc_block->words.p)
            spindle_select((spindle_id_t)(gc_block->values.p == 0.0f ? default_spindle_id : get_spindle_id(spindle_setting[1].ref_id)));
        else
            spindle_select(get_spindle_id(spindle_setting[(uint32_t)gc_block->values.q].ref_id));
    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void tool_selected (tool_data_t *tool)
{
    bool ok = false;
    spindle_num_t idx = N_SPINDLE_SELECTABLE;

    if(select_by_tool) do {
        idx--;
        if(spindle_setting[idx].ref_id != SPINDLE_NONE && (idx == 0 || spindle_setting[idx].min_tool_id > 0) && tool->tool_id >= spindle_setting[idx].min_tool_id)
            ok = spindle_select(idx == 0 ? default_spindle_id : get_spindle_id(spindle_setting[idx].ref_id));
    } while(idx && !ok);

    if(on_tool_selected)
        on_tool_selected(tool);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {

        const char *name;

        if((name = spindle_get_name(spindle_get_default()))) {
            hal.stream.write("[SPINDLE:");
            hal.stream.write(name);
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

static bool is_setting2_available (const setting_detail_t *setting)
{
    return n_spindle && (setting->id == Setting_SpindleToolStart0 || spindle_setting[setting->id - Setting_SpindleToolStart0].ref_id != SPINDLE_NONE);
}

static bool spindle_settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data)
{
    uint_fast16_t idx;

    for(idx = 0; idx < N_SPINDLE_SELECTABLE; idx++) {
        if(idx == 0 || spindle_setting[idx].ref_id != SPINDLE_NONE)
            callback(setting, idx, data);
    }

    return true;
}

#endif // N_SYS_SPINDLE == 1

static spindle_id_t get_spindle_id (uint8_t ref_id)
{
    spindle_id_t spindle_id = -1, idx;

    if(ref_id != SPINDLE_NONE)
      for(idx = 0; idx < N_SPINDLE; idx++) {
        if(ref_id_map[idx] == ref_id) {
            spindle_id = idx;
            break;
        }
    }

    return spindle_id;
}

static bool is_setting1_available (const setting_detail_t *setting)
{
    return (setting->id - Setting_SpindleEnable0) < n_spindle;
}

static status_code_t set_spindle_type (setting_id_t id, uint_fast16_t int_value)
{
    spindle_id_t spindle_id = int_value - 1;

    if(spindle_id >= 0) {
        if(spindle_get_count() < 2)
            return Status_SettingDisabled;
        else if(spindle_id >= spindle_get_count())
            return Status_SettingValueOutOfRange;
        else if(spindle_id == default_spindle_id)
            return Status_InvalidStatement;
// TODO: check for duplicate registration and/or allow multiple instantiations of spindles where possible...
    }

    spindle_setting[id - Setting_SpindleEnable0].ref_id = spindle_id == -1 ? SPINDLE_NONE : ref_id_map[spindle_id];

    return Status_OK;
}

static uint32_t get_int (setting_id_t id)
{
    return get_spindle_id(spindle_setting[id - Setting_SpindleEnable0].ref_id) + 1;
}

static const setting_detail_t spindle_settings[] = {
#if N_SPINDLE_SELECTABLE > 1
    { Setting_SpindleEnable1, Group_Spindle, "Spindle 2", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 2
    { Setting_SpindleEnable2, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 3
    { Setting_SpindleEnable3, Group_Spindle, "Spindle 4", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 4
    { Setting_SpindleEnable4, Group_Spindle, "Spindle 5", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 5
    { Setting_SpindleEnable5, Group_Spindle, "Spindle 6", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 6
    { Setting_SpindleEnable6, Group_Spindle, "Spindle 7", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 7
    { Setting_SpindleEnable7, Group_Spindle, "Spindle 8", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SYS_SPINDLE == 1
    { Setting_SpindleToolStart0, Group_Spindle, "Spindle ? tool number start", NULL, Format_Int16, "####0", "0", max_tool, Setting_NonCore, &spindle_setting[0].min_tool_id, NULL, is_setting2_available, SETTING_OPTS },
#endif // N_SYS_SPINDLE
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t spindle_settings_descr[] = {
#if N_SPINDLE_SELECTABLE > 1
    { Setting_SpindleEnable1, "Spindle to use as spindle 2." },
#endif
#if N_SPINDLE_SELECTABLE > 2
    { Setting_SpindleEnable2, "Spindle to use as spindle 3." },
#endif
#if N_SPINDLE_SELECTABLE > 3
    { Setting_SpindleEnable3, "Spindle to use as spindle 4." },
#endif
#if N_SPINDLE_SELECTABLE > 4
    { Setting_SpindleEnable4, "Spindle to use as spindle 5." },
#endif
#if N_SPINDLE_SELECTABLE > 5
    { Setting_SpindleEnable5, "Spindle to use as spindle 6." },
#endif
#if N_SPINDLE_SELECTABLE > 6
    { Setting_SpindleEnable5, "Spindle to use as spindle 7." },
#endif
#if N_SPINDLE_SELECTABLE > 7
    { Setting_SpindleEnable5, "Spindle to use as spindle 8." },
#endif
#if N_SYS_SPINDLE == 1
    { Setting_SpindleToolStart0, "Start of tool numbers for selecting the spindle.\\n"
                                 "Normally leave this at 0 for spindle 1 (default spindle)."
    }
#endif // N_SYS_SPINDLE
};

#endif // NO_SETTINGS_DESCRIPTIONS

static void activate_spindles (void *data)
{
    spindle_id_t idx;
    const setting_detail_t *spindles;

    if((spindles = setting_get_details(Setting_SpindleType, NULL)))
        strcat(strcpy(format, "Disabled,"), spindles->format);

#if N_SYS_SPINDLE > 1
    for(idx = 1; idx < N_SYS_SPINDLE; idx++) {
#else
    for(idx = 1; idx < N_SPINDLE_SETTINGS; idx++) {
#endif
        if(get_spindle_id(spindle_setting[idx].ref_id) == -1)
            spindle_setting[idx].ref_id = SPINDLE_NONE;
#if N_SYS_SPINDLE > 1
        if(spindle_setting[idx].ref_id != SPINDLE_NONE)
            spindle_enable(get_spindle_id(spindle_setting[idx].ref_id));
#endif
    }
}

// Write settings to non volatile storage (NVS).
static void spindle_settings_save (void)
{
#if N_SYS_SPINDLE == 1

    spindle_num_t idx = N_SPINDLE_SELECTABLE;

    select_by_tool = false;

    do {
        idx--;
        select_by_tool = spindle_setting[idx].ref_id != SPINDLE_NONE && spindle_setting[idx].min_tool_id > 0;
    } while(idx && !select_by_tool);

    if(select_by_tool) {
        if(on_tool_selected == NULL && grbl.on_tool_selected != tool_selected) {
            on_tool_selected = grbl.on_tool_selected;
            grbl.on_tool_selected = tool_selected;
        }
    } else if(grbl.on_tool_selected == tool_selected) {
        grbl.on_tool_selected = on_tool_selected;
        on_tool_selected = NULL;
    }

#endif

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_setting, sizeof(spindle_setting), true);
}

static bool validate_spindle (spindle_info_t *spindle, void *ref_id)
{
    return spindle->ref_id == *((spindle_id_t *)ref_id);
}

// Restore default settings and write to non volatile storage (NVS).
static void spindle_settings_restore (void)
{
    struct {
        uint8_t idx;
        uint8_t ref_id;
    } spd;

    spd.idx = N_SPINDLE_SETTINGS;

    memset(&spindle_setting, 0, sizeof(spindle_setting));
    spindle_setting[6].ref_id = 0xAB;
    spindle_setting[7].ref_id = 0xCD;

    do {
        spd.idx--;
        spd.ref_id = SPINDLE_NONE;
        switch(spd.idx) {
            case 0:
//                spd.ref_id = settings.spindle.flags.type;
                break;
#if N_SPINDLE_SELECTABLE > 1 && defined(DEFAULT_SPINDLE2)
            case 1:
                spd.ref_id = DEFAULT_SPINDLE2;
                break;
#endif
#if N_SPINDLE_SELECTABLE > 2 && defined(DEFAULT_SPINDLE3)
            case 2:
                spd.ref_id = DEFAULT_SPINDLE3;
                break;
#endif
#if N_SPINDLE_SELECTABLE > 3 && defined(DEFAULT_SPINDLE4)
            case 3:
                spd.ref_id = DEFAULT_SPINDLE4;
                break;
#endif
        }
        if(spd.ref_id != SPINDLE_NONE && spindle_enumerate_spindles(validate_spindle, &spd.ref_id))
            spindle_setting[spd.idx].ref_id = spd.ref_id;
        spindle_setting[spd.idx].min_tool_id = 0;
    } while(spd.idx);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_setting, sizeof(spindle_setting), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void spindle_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&spindle_setting, nvs_address, sizeof(spindle_setting), true) != NVS_TransferResult_OK)
        spindle_settings_restore();

#if N_SYS_SPINDLE == 1

    uint_fast8_t idx, j = 1, k;

    for(idx = 2; idx < N_SPINDLE_SETTINGS - 2; idx++) {
        for(k = 0; k < idx; k++) {
            if(k < j && spindle_setting[j].ref_id == spindle_setting[k].ref_id)
                spindle_setting[j].ref_id = SPINDLE_NONE;
            if(spindle_setting[idx].ref_id == spindle_setting[k].ref_id)
                spindle_setting[idx].ref_id = SPINDLE_NONE;
        }
        if(spindle_setting[j].ref_id == SPINDLE_NONE && spindle_setting[idx].ref_id != SPINDLE_NONE) {
            spindle_setting[j].ref_id = spindle_setting[idx].ref_id;
            spindle_setting[j].min_tool_id = spindle_setting[idx].min_tool_id;
            spindle_setting[idx].ref_id = SPINDLE_NONE;
        }
        if(spindle_setting[idx].ref_id == SPINDLE_NONE && spindle_setting[j].ref_id != SPINDLE_NONE)
            j = idx;
    }

    idx = N_SPINDLE_SELECTABLE;
    do {
        idx--;
        select_by_tool = spindle_setting[idx].ref_id != SPINDLE_NONE && spindle_setting[idx].min_tool_id > 0;
    } while(idx && !select_by_tool);

    if(select_by_tool) {
        on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = tool_selected;
    }

  #if N_SPINDLE_SELECTABLE > 1

    if(grbl.tool_table.n_tools) {

        idx = N_SPINDLE_SELECTABLE;
        strcpy(max_tool, uitoa(grbl.tool_table.n_tools));

        do {
            idx--;
            if(spindle_setting[idx].min_tool_id > grbl.tool_table.n_tools)
                spindle_setting[idx].min_tool_id = grbl.tool_table.n_tools;
        } while(idx);
    }
  #endif
#endif // N_SYS_SPINDLE == 1
}

static bool map_spindles (spindle_info_t *spindle, void *data)
{
    ref_id_map[spindle->id] = spindle->ref_id;

    return false;
}

static bool spindle_select_config (settings_t *settings)
{
    bool ok;

    if((ok = driver_setup(settings))) {

        default_spindle_id = setting_get_int_value(setting_get_details(Setting_SpindleType, NULL), 0);

        spindle_enumerate_spindles(map_spindles, NULL);

        spindle_setting[0].ref_id = ref_id_map[default_spindle_id]; // always default spindle!

#if N_SYS_SPINDLE > 1

        n_spindle = spindle_get_count();

#else

        if((n_spindle = spindle_get_count()) > 1) {

            memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

            grbl.user_mcode.check = check;
            grbl.user_mcode.validate = validate;
            grbl.user_mcode.execute = execute;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = report_options;
        }

#endif
    }

    protocol_enqueue_foreground_task(activate_spindles, NULL);

    return ok;
}

int8_t spindle_select_get_binding (spindle_id_t spindle_id)
{
    uint_fast8_t idx = N_SPINDLE;

    if(spindle_id == default_spindle_id)
        return 0;

    if(spindle_id >= 0) do {
        if(get_spindle_id(spindle_setting[--idx].ref_id) == spindle_id)
            return idx;
    } while(idx);

    return -1;
}

void spindle_select_init (void)
{
    static setting_details_t setting_details = {
        .settings = spindle_settings,
        .n_settings = sizeof(spindle_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = spindle_settings_descr,
        .n_descriptions = sizeof(spindle_settings_descr) / sizeof(setting_descr_t),
#endif
        .save = spindle_settings_save,
        .load = spindle_settings_load,
        .restore = spindle_settings_restore,
#if N_SYS_SPINDLE == 1
        .iterator = spindle_settings_iterator
#endif
    };

    if((nvs_address = nvs_alloc(sizeof(spindle_setting)))) {

        settings_register(&setting_details);

        // delay plugin config until all spindles are registered
        driver_setup = hal.driver_setup;
        hal.driver_setup = spindle_select_config;
    } else
        protocol_enqueue_foreground_task(report_warning, "Spindle select plugin failed to initialize!");
}

#endif
