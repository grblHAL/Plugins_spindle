/*

  modvfd.h - MODVFD generic Modbus RTU support

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

#ifndef _MODVFD_H_
#define _MODVFD_H_

#if VFD_ENABLE

#ifdef ARDUINO
#include "../grbl/hal.h"
#else
#include "grbl/hal.h"
#endif

void modvfd_spindleUpdateRPM (float rpm);
void modvfd_spindleSetState (spindle_state_t state, float rpm);
spindle_state_t modvfd_spindleGetState (void);
bool modvfd_spindle_config (void);
void modvfd_OnReportOptions (bool newopt);
void modvfd_reset (void);
bool modvfd_spindle_select (spindle_id_t spindle_id);

#endif
#endif
