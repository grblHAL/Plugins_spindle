/*

  vfd_spindle.c - Top level functions for VFD spindle selection and control

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if VFD_ENABLE

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#endif

#include "modbus.h"
#include "vfd_spindle.h"

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Huanyang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

static on_spindle_select_ptr on_spindle_select;
static nvs_address_t nvs_address = 0;


// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t vfd_groups [] = {
    {Group_Root, Group_UserSettings, "VFD Config"}
};

static const setting_detail_t vfd_settings[] = {
     { Setting_VFD_TYPE, Group_UserSettings, "VFD Model", NULL, Format_RadioButtons, "Huanyang 1,Huanyang P2A,Durapulse GS20,Yalang YL620A, MODVFD Custom", NULL, NULL, Setting_NonCore, &vfd_config.vfd_type, NULL, NULL },  
     { Setting_VFD_RPM_HZ, Group_UserSettings, "RPM per Hz", "", Format_Integer, "####0", "1", "3000", Setting_NonCore, &vfd_config.vfd_rpm_hz, NULL, NULL },
     { Setting_VFD_PLUGIN_10, Group_UserSettings, "Run/Stop Register (decimal)", NULL, Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.runstop_reg, NULL, NULL },  
     { Setting_VFD_PLUGIN_11, Group_UserSettings, "Set Frequency Register (decimal)", "", Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.set_freq_reg, NULL, NULL },    
     { Setting_VFD_PLUGIN_12, Group_UserSettings, "Get Frequency Register (decimal)", NULL, Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.get_freq_reg, NULL, NULL },  
     { Setting_VFD_PLUGIN_13, Group_UserSettings, "Run CW Command (decimal)", "", Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.run_cw_cmd, NULL, NULL },   
     { Setting_VFD_PLUGIN_14, Group_UserSettings, "Run CCW Command (decimal)", NULL, Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.run_ccw_cmd, NULL, NULL },  
     { Setting_VFD_PLUGIN_15, Group_UserSettings, "Stop Command (decimal)", "", Format_Integer, "########0", NULL, NULL, Setting_NonCore, &vfd_config.stop_cmd, NULL, NULL },  
     { Setting_VFD_PLUGIN_16, Group_UserSettings, "RPM input Multiplier", "", Format_Decimal, "########0", NULL, NULL, Setting_NonCore, &vfd_config.in_multiplier, NULL, NULL }, 
     { Setting_VFD_PLUGIN_17, Group_UserSettings, "RPM input Divider", "", Format_Decimal, "########0", NULL, NULL, Setting_NonCore, &vfd_config.in_divider, NULL, NULL },  
     { Setting_VFD_PLUGIN_18, Group_UserSettings, "RPM output Multiplier", "", Format_Decimal, "########0", NULL, NULL, Setting_NonCore, &vfd_config.out_multiplier, NULL, NULL }, 
     { Setting_VFD_PLUGIN_19, Group_UserSettings, "RPM output Divider", "", Format_Decimal, "########0", NULL, NULL, Setting_NonCore, &vfd_config.out_divider, NULL, NULL },       
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t vfd_settings_descr[] = {
    { Setting_VFD_TYPE, "type of vfd" },
    { Setting_VFD_RPM_HZ, "rpm_hz" },
    { Setting_VFD_PLUGIN_10, "Register for Run/stop" },
    { Setting_VFD_PLUGIN_11, "Set Frequency Register" },
    { Setting_VFD_PLUGIN_12, "Get Frequency Register" },
    { Setting_VFD_PLUGIN_13, "Command word for CW" },
    { Setting_VFD_PLUGIN_14, "Command word for CCW" },
    { Setting_VFD_PLUGIN_15, "Command word for stop" },   
    { Setting_VFD_PLUGIN_16, "RPM value multiplier for programming RPM" },
    { Setting_VFD_PLUGIN_17, "RPM value divider for programming RPM" },
    { Setting_VFD_PLUGIN_18, "RPM value multiplier for reading RPM" },
    { Setting_VFD_PLUGIN_19, "RPM value divider for reading RPM" },               
};
#endif

static bool vfd_spindle_select (spindle_id_t spindle_id)
{
    if(spindle_id == NULL) {

        switch (vfd_config.vfd_type) {
            case MODVFD:
            MODVFD_init();
            break;
            case GS20:
            GS20_init();
            break;
            case YL620A:
            YL620_init();
            break;
            case HUANYANG1:
            case HUANYANG2:
            vfd_huanyang_init();
            break;            
            default:
            break;
        }
    }

    return true;
}

static void vfd_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&vfd_config, sizeof(vfd_settings_t), true);
}

static void vfd_settings_restore (void)
{
    vfd_config.vfd_type = MODVFD; //settings below are defaulted to values for GS20 VFD
    vfd_config.vfd_rpm_hz = 60;
    vfd_config.runstop_reg = 8192; //0x2000
    vfd_config.set_freq_reg = 8193; //0x2001
    vfd_config.get_freq_reg = 8451; //0x2103
    vfd_config.run_cw_cmd = 17; //0x11
    vfd_config.run_ccw_cmd = 33; //0x21
    vfd_config.stop_cmd = 2; //0x02
    vfd_config.in_multiplier = 50;
    vfd_config.in_divider = 60;
    vfd_config.out_multiplier = 60;
    vfd_config.out_divider = 100;    

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&vfd_config, sizeof(vfd_settings_t), true);
}

static void vfd_settings_load (void)
{
    if(nvs_address != 0){
        if((hal.nvs.memcpy_from_nvs((uint8_t *)&vfd_config, nvs_address, sizeof(vfd_settings_t), true) != NVS_TransferResult_OK))
            vfd_settings_restore();
    }
}

static setting_details_t vfd_setting_details = {
    .groups = vfd_groups,
    .n_groups = sizeof(vfd_groups) / sizeof(setting_group_detail_t),
    .settings = vfd_settings,
    .n_settings = sizeof(vfd_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = vfd_settings_descr,
    .n_descriptions = sizeof(vfd_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = vfd_settings_load,
    .restore = vfd_settings_restore,
    .save = vfd_settings_save
};

static on_report_options_ptr on_report_options;

extern modbus_settings_t modbus;

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:VFD SELECTOR v0.02]" ASCII_EOL);
    }
}

void vfd_init (void)
{
    if(modbus_enabled() && (nvs_address = nvs_alloc(sizeof(vfd_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_register(&vfd_setting_details);

        //vfd_huanyang_init();
        //MODVFD_init();

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = vfd_spindle_select;
        
    }
}

#endif
