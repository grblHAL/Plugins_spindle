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

#include "GS20_VFD.h"
#include "MODVFD.h"
#include "YL620_VFD.h"
#include "huanyang.h"

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Huanyang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

static on_spindle_select_ptr on_spindle_select;
static nvs_address_t nvs_address = 0;
static driver_reset_ptr driver_reset;

static on_report_options_ptr on_report_options;
extern modbus_settings_t modbus;

/*static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};*/


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
    { Setting_VFD_TYPE, "Choose from supported VFDs or use custom MODVFD" },
    { Setting_VFD_RPM_HZ, "RPM/Hz value for GS20 and YL620A" },
    { Setting_VFD_PLUGIN_10, "MODVFD Register for Run/stop" },
    { Setting_VFD_PLUGIN_11, "MODVFD Set Frequency Register" },
    { Setting_VFD_PLUGIN_12, "MODVFD Get Frequency Register" },
    { Setting_VFD_PLUGIN_13, "MODVFD Command word for CW" },
    { Setting_VFD_PLUGIN_14, "MODVFD Command word for CCW" },
    { Setting_VFD_PLUGIN_15, "MODVFD Command word for stop" },   
    { Setting_VFD_PLUGIN_16, "MODVFD RPM value multiplier for programming RPM" },
    { Setting_VFD_PLUGIN_17, "MODVFD RPM value divider for programming RPM" },
    { Setting_VFD_PLUGIN_18, "MODVFD RPM value multiplier for reading RPM" },
    { Setting_VFD_PLUGIN_19, "MODVFD RPM value divider for reading RPM" },               
};
#endif

static void vfd_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&vfd_config, sizeof(vfd_settings_t), true);
}

static void vfd_settings_restore (void)
{
    vfd_config.vfd_type = HUANYANG1; //settings below are defaulted to values for GS20 VFD
    vfd_config.vfd_rpm_hz = 60;
    vfd_config.runstop_reg = 8192; //0x2000
    vfd_config.set_freq_reg = 8193; //0x2001
    vfd_config.get_freq_reg = 8451; //0x2103
    vfd_config.run_cw_cmd = 18; //0x12
    vfd_config.run_ccw_cmd = 34; //0x22
    vfd_config.stop_cmd = 1; //0x02
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

void vfd_spindleUpdateRPM (float rpm)
{
    switch (vfd_config.vfd_type) {
        case MODVFD:
            modvfd_spindleUpdateRPM(rpm);
        break;
        case GS20:
            gs20_spindleUpdateRPM(rpm);
        break;
        case YL620A:
            yl620_spindleUpdateRPM(rpm);
        break;
        case HUANYANG1:
            huanyangv1_spindleUpdateRPM(rpm);
            break;
        case HUANYANG2:
            huanyangv2_spindleUpdateRPM(rpm);
        break;            
        default:
        break;
    }    
}

static void vfd_spindleSetState (spindle_state_t state, float rpm){
    switch (vfd_config.vfd_type) {
        case MODVFD:
            modvfd_spindleSetState (state, rpm);
        break;
        case GS20:
            gs20_spindleSetState (state, rpm);
        break;
        case YL620A:
            yl620_spindleSetState (state, rpm);
        break;
        case HUANYANG1:
            return huanyangv1_spindleSetState(state, rpm);
        break;
        case HUANYANG2:
            return huanyangv2_spindleSetState(state, rpm);
        break;            
        default:
        break;
    }
}

static spindle_state_t vfd_spindleGetState (void) {

    spindle_state_t vfd_state = {0};

    switch (vfd_config.vfd_type) {
        case MODVFD:
            vfd_state = modvfd_spindleGetState();
        break;
        case GS20:
            vfd_state = gs20_spindleGetState();
        break;
        case YL620A:
            vfd_state = yl620_spindleGetState();
        break;
        case HUANYANG1:
            vfd_state = huanyangv1_spindleGetState();
        break;
        case HUANYANG2:
            vfd_state = huanyangv2_spindleGetState();
        break;            
        default:
        break;
    }

     return vfd_state;
}

static void vfd_onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:VFD SELECTOR v0.02]" ASCII_EOL);

        switch (vfd_config.vfd_type) {
            case MODVFD:
                modvfd_OnReportOptions(newopt);
            break;
            case GS20:
                gs20_onReportOptions(newopt);
            break;
            case YL620A:
                yl620_onReportOptions(newopt);
            break;
            case HUANYANG1:
            case HUANYANG2:
                huanyang_onReportOptions(newopt);
            break;            
            default:
            break;
        }        
    }
}

static bool vfd_spindle_select (spindle_id_t spindle_id){

    bool select_ok = false;

    switch (vfd_config.vfd_type) {
        case MODVFD:
            select_ok = modvfd_spindle_select(spindle_id);
        break;
        case GS20:
            select_ok = gs20_spindle_select(spindle_id);
        break;
        case YL620A:
            select_ok = yl620_spindle_select(spindle_id);
        break;
        case HUANYANG1:
        case HUANYANG2:
            select_ok = huanyang_spindle_select(spindle_id);
        break;            
        default:
        break;
    }

    if(select_ok && on_spindle_select && on_spindle_select(spindle_id))
        return true;

    return select_ok;
}

static bool vfd_spindle_config (void)
{
    static bool init_ok = false;

    if(!modbus_isup())
        return false;

    switch (vfd_config.vfd_type) {
        case MODVFD:
            init_ok = modvfd_spindle_config();
        break;
        case GS20:
            init_ok = gs20_spindle_config();
        break;
        case YL620A:
            init_ok = yl620_spindle_config();
        break;
        case HUANYANG1:
            init_ok = huanyangv1_spindle_config();
        break;
        case HUANYANG2:
            init_ok = huanyangv2_spindle_config();
        break;            
        default:
        break;
    }  

    return init_ok;        
}

static void vfd_reset (void)
{

    driver_reset();

    switch (vfd_config.vfd_type) {
        case MODVFD:
            modvfd_reset();
        break;
        case GS20:
            gs20_reset();
        break;
        case YL620A:
            yl620_reset();
        break;
        case HUANYANG1:
        case HUANYANG2:
            huanyang_reset();
        break;            
        default:
        break;
    }
}

void vfd_init (void)
{
    vfd_spindle_id = -1;

    static const spindle_ptrs_t vfd_spindle = {
        .cap.variable = On,
        .cap.at_speed = On,
        .cap.direction = On,
        .config = vfd_spindle_config,
        .set_state = vfd_spindleSetState,
        .get_state = vfd_spindleGetState,
        .update_rpm = vfd_spindleUpdateRPM
    };

    if(modbus_enabled() && (nvs_address = nvs_alloc(sizeof(vfd_settings_t)))) {

        vfd_spindle_id = spindle_register(&vfd_spindle, "VFD Spindle");

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = vfd_spindle_select;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = vfd_onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = vfd_reset;

        settings_register(&vfd_setting_details);
        
    }
}

#endif
