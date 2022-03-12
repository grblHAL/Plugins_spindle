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
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
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

static on_report_options_ptr on_report_options;

static settings_changed_ptr settings_changed;

extern modbus_settings_t modbus;

static void _settings_changed (settings_t *settings)
{
    if(settings_changed)
        settings_changed(settings);

    if(hal.spindle.set_state == NULL && modbus_isup()){     
        switch (modbus.vfd_type) {
            case H100:
            H100_init();
            break;
            case GS20:
            GS20_init();
            break;
            case YL620A:
            YL620_init();
            break;
            case HUANYANG1:
            case HUANYANG2:
            HY_VFD_init();
            break;            
            default:
            break;
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:Runtime VFD Selector v0.02]" ASCII_EOL);
    }
}

void vfd_init (void)
{
    if(modbus_enabled()) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_changed = hal.settings_changed;
        hal.settings_changed = _settings_changed;
    }    
}

#endif
