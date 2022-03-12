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
    VFD_SetStatus,
} vfd_response_t;

typedef enum {
    HUANYANG1 = 0,
    HUANYANG2,
    GS20,
    YL620A,
    H100,
} vfd_type_t;

void GS20_init (void);
void YL620_init (void);
void HY_VFD_init (void);
void H100_init (void);

#endif
