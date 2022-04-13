/*

  modbus.h - a lightweight ModBus implementation

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

#ifndef _VFD_SPINDLE_H_
#define _VFD_SPINDLE_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Huanyang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

#define VFD_RETRIES     25
#define RETRY_DELAY 		25

typedef enum {
    VFD_Idle = 0,
    VFD_GetRPM,
    VFD_SetRPM,
    VFD_GetMaxRPM,
    VFD_GetMaxRPM50,
    VFD_GetStatus,
    VFD_SetStatus
} vfd_response_t;

typedef enum {
    HUANYANG1 = 0,
    HUANYANG2,
    GS20,
    YL620A,
    MODVFD,
} vfd_type_t;

typedef struct {
    vfd_type_t vfd_type;
    uint32_t vfd_rpm_hz;
    uint32_t runstop_reg;
    uint32_t set_freq_reg;
    uint32_t get_freq_reg;
    uint32_t run_cw_cmd;
    uint32_t run_ccw_cmd;
    uint32_t stop_cmd;
    float in_multiplier;
    float in_divider;
    float out_multiplier;
    float out_divider;    
} vfd_settings_t;

vfd_settings_t vfd_config;
spindle_id_t vfd_spindle_id;

void GS20_init (void);
void YL620_init (void);
void vfd_huanyang_init (void);
void MODVFD_init (void);

#endif
