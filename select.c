/*

  select.c - spindle select plugin

  Part of grblHAL

  Copyright (c) 2022-2023 Terje Io

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
#include <string.h>

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

typedef struct {
    spindle_id_t spindle_id;
    tool_id_t min_tool_id;
} spindle_binding_t;

static spindle_binding_t spindle_setting[8];

static uint8_t n_spindle;
static char format[110] = "";
static nvs_address_t nvs_address;

#if N_SYS_SPINDLE == 1

static bool select_by_tool = false;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_tool_selected_ptr on_tool_selected = NULL;

static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == Spindle_Select ? mcode : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == Spindle_Select) {

        if(gc_block->words.p) {
            if(isnanf(gc_block->values.p))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.p) && gc_block->values.p >= 0.0f && gc_block->values.p <= 1.0f))
                state = Status_GcodeValueOutOfRange;
        } else if(gc_block->words.q) {
            if(isnanf(gc_block->values.q))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.q) && gc_block->values.q >= 0.0f && spindle_setting[(uint32_t)gc_block->values.q].spindle_id != -1))
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

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == Spindle_Select) {
        if(gc_block->words.p)
            spindle_select((spindle_id_t)(gc_block->values.p == 0.0f ? 0 : settings.spindle.flags.type));
        else
            spindle_select(spindle_setting[(uint32_t)gc_block->values.q].spindle_id);
    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void tool_selected (tool_data_t *tool)
{
    bool ok = false;
    spindle_num_t idx = N_SPINDLE_SELECTABLE;

    if(select_by_tool) do {
        idx--;
        if(spindle_setting[idx].spindle_id != -1 && (idx == 0 || spindle_setting[idx].min_tool_id > 0) && tool->tool_id >= spindle_setting[idx].min_tool_id)
            ok = spindle_select(idx == 0 ? settings.spindle.flags.type : spindle_setting[idx].spindle_id);
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
    return setting->id == Setting_SpindleToolStart0 || spindle_setting[setting->id - Setting_SpindleToolStart0].spindle_id != -1;
}

#endif

static bool is_setting1_available (const setting_detail_t *setting)
{
    return (setting->id - Setting_SpindleEnable0) < n_spindle;
}

static status_code_t set_spindle_type (setting_id_t id, uint_fast16_t int_value)
{
    spindle_id_t spindle_id = int_value == n_spindle ? -1 : int_value;

    if(spindle_id >= 0) {
        if(spindle_get_count() < 2)
            return Status_SettingDisabled;
        else if(int_value >= spindle_get_count())
            return Status_SettingValueOutOfRange;
        else if(spindle_id == settings.spindle.flags.type)
            return Status_InvalidStatement;
// TODO: check for duplicate registration and/or allow multiple instantiations of spindles where possible...
    }

    spindle_setting[id - Setting_SpindleEnable0].spindle_id = spindle_id;

    return Status_OK;
}

static uint32_t get_int (setting_id_t id)
{
    uint32_t value = spindle_setting[id - Setting_SpindleEnable0].spindle_id;

    return value == -1 ? n_spindle : value;
}

static const setting_detail_t spindle_settings[] = {
#if N_SPINDLE_SELECTABLE > 1
    { Setting_SpindleEnable1, Group_Spindle, "Spindle 1", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 2
    { Setting_SpindleEnable2, Group_Spindle, "Spindle 2", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 3
    { Setting_SpindleEnable3, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 4
    { Setting_SpindleEnable4, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 5
    { Setting_SpindleEnable5, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 6
    { Setting_SpindleEnable6, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SPINDLE_SELECTABLE > 7
    { Setting_SpindleEnable7, Group_Spindle, "Spindle 3", NULL, Format_RadioButtons, format, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting1_available, { .reboot_required = On } },
#endif
#if N_SYS_SPINDLE == 1
  #if N_SPINDLE_SELECTABLE > 1
    { Setting_SpindleToolStart0, Group_Spindle, "Spindle 0 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[0].min_tool_id, NULL, is_setting2_available },
    { Setting_SpindleToolStart1, Group_Spindle, "Spindle 1 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[1].min_tool_id, NULL, is_setting2_available },
  #endif
  #if N_SPINDLE_SELECTABLE > 2
    { Setting_SpindleToolStart2, Group_Spindle, "Spindle 2 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[2].min_tool_id, NULL, is_setting2_available },
  #endif
  #if N_SPINDLE_SELECTABLE > 3
    { Setting_SpindleToolStart3, Group_Spindle, "Spindle 3 tool numbert start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[3].min_tool_id, NULL, is_setting2_available },
  #endif
/*
  #if N_SPINDLE > 4
    { Setting_SpindleToolStart4, Group_Spindle, "Spindle 4 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[4].min_tool_id, NULL, is_setting2_available },
  #endif
  #if N_SPINDLE > 5
    { Setting_SpindleToolStart4, Group_Spindle, "Spindle 5 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[5].min_tool_id, NULL, is_setting2_available },
  #endif
  #if N_SPINDLE > 6
    { Setting_SpindleToolStart6, Group_Spindle, "Spindle 6 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[5].min_tool_id, NULL, is_setting2_available },
  #endif
  #if N_SPINDLE > 7
    { Setting_SpindleToolStart7, Group_Spindle, "Spindle 7 tool number start", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &spindle_setting[5].min_tool_id, NULL, is_setting2_available },
  #endif
*/
#endif // N_SYS_SPINDLE
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t spindle_settings_descr[] = {
#if N_SPINDLE > 1
    { Setting_SpindleEnable1, "Spindle to use as spindle 1." },
#endif
#if N_SPINDLE > 2
    { Setting_SpindleEnable2, "Spindle to use as spindle 2." },
#endif
#if N_SPINDLE > 3
    { Setting_SpindleEnable3, "Spindle to use as spindle 3." },
#endif
#if N_SYS_SPINDLE > 4
    { Setting_SpindleEnable4, "Spindle to use as spindle 4." },
#endif
#if N_SYS_SPINDLE > 5
    { Setting_SpindleEnable5, "Spindle to use as spindle 5." },
#endif
#if N_SYS_SPINDLE > 5
    { Setting_SpindleEnable5, "Spindle to use as spindle 6." },
#endif
#if N_SYS_SPINDLE > 5
    { Setting_SpindleEnable5, "Spindle to use as spindle 7." },
#endif
#if N_SYS_SPINDLE == 1
#if N_SPINDLE > 1
    { Setting_SpindleToolStart0, "Start of tool numbers for selecting spindle 0 (default spindle).\\n"
                                 "Normally leave this at 0."
    },
    { Setting_SpindleToolStart1, "Start of tool numbers for selecting spindle 1." },
  #endif
  #if N_SPINDLE > 2
    { Setting_SpindleToolStart2, "Start of tool numbers for selecting spindle 2." },
  #endif
  #if N_SPINDLE > 3
    { Setting_SpindleToolStart3, "Start of tool numbers for selecting spindle 3." },
  #endif
/*
  #if N_SPINDLE > 4
    { Setting_SpindleToolStart4, "Start of tool numbers for selecting spindle 4." },
  #endif
  #if N_SPINDLE > 5
    { Setting_SpindleToolStart5, "Start of tool numbers for selecting spindle 5." },
  #endif
  #if N_SPINDLE > 5
    { Setting_SpindleToolStart6, "Start of tool numbers for selecting spindle 6." },
  #endif
  #if N_SPINDLE > 5
    { Setting_SpindleToolStart7, "Start of tool numbers for selecting spindle 7." },
  #endif
*/
#endif // N_SYS_SPINDLE
};

static void activate_spindles (uint_fast16_t state)
{
    spindle_id_t idx;
    const setting_detail_t *spindles;

    if((spindles = setting_get_details(Setting_SpindleType, NULL)))
        strcat(strcpy(format, spindles->format), ",Disabled");

#if N_SYS_SPINDLE > 1
    for(idx = 1; idx < N_SYS_SPINDLE; idx++) {
#else
    for(idx = 1; idx < sizeof(spindle_setting) / sizeof(spindle_binding_t); idx++) {
#endif
        if(spindle_setting[idx].spindle_id >= spindle_get_count())
            spindle_setting[idx].spindle_id = -1;
#if N_SYS_SPINDLE > 1
        if(spindle_setting[idx].spindle_id != -1)
            spindle_enable(spindle_setting[idx].spindle_id);
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
        select_by_tool = spindle_setting[idx].spindle_id != -1 && spindle_setting[idx].min_tool_id > 0;
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

// Restore default settings and write to non volatile storage (NVS).
static void spindle_settings_restore (void)
{
    uint_fast8_t idx = sizeof(spindle_setting) / sizeof(spindle_binding_t);

    do {
        idx--;
        spindle_setting[idx].spindle_id = idx == 0 ? 0 : -1;
        spindle_setting[idx].min_tool_id = 0;
    } while(idx);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&spindle_setting, sizeof(spindle_setting), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void spindle_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&spindle_setting, nvs_address, sizeof(spindle_setting), true) != NVS_TransferResult_OK)
        spindle_settings_restore();

    spindle_setting[0].spindle_id = 0; // always default spindle!

#if N_SYS_SPINDLE == 1

    spindle_num_t idx = N_SPINDLE_SELECTABLE;

    do {
        idx--;
        select_by_tool = spindle_setting[idx].spindle_id != -1 && spindle_setting[idx].min_tool_id > 0;
    } while(idx && !select_by_tool);

    if(select_by_tool) {
        on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = tool_selected;
    }

#endif

    protocol_enqueue_rt_command(activate_spindles);
}

static setting_details_t setting_details = {
    .settings = spindle_settings,
    .n_settings = sizeof(spindle_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = spindle_settings_descr,
    .n_descriptions = sizeof(spindle_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = spindle_settings_save,
    .load = spindle_settings_load,
    .restore = spindle_settings_restore
};

#endif

static void warning_msg (uint_fast16_t state)
{
    report_message("Spindle select plugin failed to initialize!", Message_Warning);
}

int8_t spindle_select_get_binding (spindle_id_t spindle_id)
{
    uint_fast8_t idx = N_SPINDLE;

    if(spindle_id == settings.spindle.flags.type)
        return 0;

    if(spindle_id >= 0) do {
        if(spindle_setting[--idx].spindle_id == spindle_id)
            return idx;
    } while(idx);

    return -1;
}

void spindle_select_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(spindle_setting)))) {

        settings_register(&setting_details);

#if N_SYS_SPINDLE > 1

        n_spindle = spindle_get_count();

#else

        if((n_spindle = spindle_get_count()) > 1) {

            memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

            hal.user_mcode.check = check;
            hal.user_mcode.validate = validate;
            hal.user_mcode.execute = execute;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = report_options;
        }

#endif
    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
