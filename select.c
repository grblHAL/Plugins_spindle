/*

  select.c - spindle select plugin

  Part of grblHAL

  Copyright (c) 2022 Terje Io

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
#else
#include "grbl/hal.h"
#endif

static user_mcode_ptrs_t user_mcode;

static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == Spindle_Select ? mcode : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == Spindle_Select && grbl.on_spindle_select) {

        if(!gc_block->words.p || isnanf(gc_block->values.p))
            state = Status_GcodeValueWordMissing;
        else if(!(isintf(gc_block->values.p) && gc_block->values.p >= 0.0f && gc_block->values.p <= 1.0f))
            state = Status_GcodeValueOutOfRange;
        else {
            gc_block->words.p = Off;
            gc_block->user_mcode_sync = On;
        }
    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == Spindle_Select)
        grbl.on_spindle_select((uint_fast8_t)(gc_block->values.p));
    else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

void spindle_select_init (void)
{
    if(grbl.on_spindle_select) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = check;
        hal.user_mcode.validate = validate;
        hal.user_mcode.execute = execute;
    }
}
