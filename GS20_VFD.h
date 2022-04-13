/*

  gs20.h - Durapulse GS20 support

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

#ifndef _GS20_VFD_H_
#define _GS20_VFD_H_

#if VFD_ENABLE

#ifdef ARDUINO
#include "../grbl/hal.h"
#else
#include "grbl/hal.h"
#endif

void gs20_spindleUpdateRPM (float rpm);
void gs20_spindleSetState (spindle_state_t state, float rpm);
spindle_state_t gs20_spindleGetState (void);
bool gs20_spindle_config (void);
void gs20_onReportOptions (bool newopt);
void gs20_reset (void);
bool gs20_spindle_select (spindle_id_t spindle_id);

#endif
#endif
