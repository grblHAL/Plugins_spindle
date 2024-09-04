/*

  offset.c - plugin for for (laser) spindle offsetting

  Part of grblHAL

  grblHAL (c) 2024 Terje Io

  Grbl is free software: you can redistribute it and/or modify
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

#include "driver.h"

#if SPINDLE_OFFSET == 1

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"

#define N_OFFSETS 1

typedef struct {
    float x;
    float y;
} offset_setting_t;

typedef struct {
    offset_setting_t offset[N_OFFSETS];
} offset_settings_t;

static nvs_address_t nvs_address;
static offset_settings_t plugin_settings;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    static spindle_id_t default_spindle_id = -1, laser_spindle_id = -1;

    if(default_spindle_id == -1)
        default_spindle_id = spindle->id;

    if(sys.driver_started && (plugin_settings.offset[0].x != 0.0f || plugin_settings.offset[0].y != 0.0f)) {

        bool move_offset;
        coord_data_t target;
        plan_line_data_t plan_data;

        plan_data_init(&plan_data);
        plan_data.condition.rapid_motion = On;

        protocol_buffer_synchronize();
        system_convert_array_steps_to_mpos(target.values, sys.position);

        if((move_offset = spindle->id != default_spindle_id && spindle->cap.laser && laser_spindle_id == -1)) {
            laser_spindle_id = spindle->id;
            target.x += plugin_settings.offset[0].x;
            target.y += plugin_settings.offset[0].y;
        } else if((move_offset = laser_spindle_id != -1)) {
            laser_spindle_id = -1;
            target.x -= plugin_settings.offset[0].x;
            target.y -= plugin_settings.offset[0].y;
        }

        if(move_offset) {
            if(mc_line(target.values, &plan_data)) {
                protocol_buffer_synchronize();
                sync_position();
            } // else alarm?
        }
    }

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static bool is_setting_available (const setting_detail_t *setting)
{
    return true; // TODO: check if there is a non-default laser spindle available
}

static const setting_detail_t offset_settings[] = {
    { Setting_SpindleOffsetX, Group_Spindle, "Laser X offset", "mm", Format_Decimal, "-##0.000", "-1000", NULL, Setting_IsExtended, &plugin_settings.offset[0].x, NULL, is_setting_available },
    { Setting_SpindleOffsetY, Group_Spindle, "Laser Y offset", "mm", Format_Decimal, "-##0.000", "-1000", NULL, Setting_IsExtended, &plugin_settings.offset[0].y, NULL, is_setting_available },
};

#ifdef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t offset_settings_descr[] = {
    { Setting_SpindleOffsetX, "X offset from current position for non-default laser spindle." },
    { Setting_SpindleOffsetY, "X offset from current position for non-default laser spindle." }
};

#endif

static void offset_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(offset_settings_t), true);
}

static void offset_settings_restore (void)
{
    memset(&plugin_settings, 0, sizeof(offset_settings_t));

    offset_settings_save();
}

static void offset_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(offset_settings_t), true) != NVS_TransferResult_OK)
        offset_settings_restore();
}

static setting_details_t setting_details = {
    .settings = offset_settings,
    .n_settings = sizeof(offset_settings) / sizeof(setting_detail_t),
#ifdef NO_SETTINGS_DESCRIPTIONS
    .descriptions = offset_settings_descr,
    .n_descriptions = sizeof(offset_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = offset_settings_save,
    .load = offset_settings_load,
    .restore = offset_settings_restore
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Spindle offset", "0.01");
}

void spindle_offset_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(offset_settings_t)))) {

        settings_register(&setting_details);

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;
    }
}

#endif // SPINDLE_OFFSET == 1

