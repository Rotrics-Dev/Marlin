/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"

#include "../../MarlinCore.h"

#if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/stepper.h"
#endif

#include "../../module/dexarm/dexarm.h"

extern xyze_pos_t destination;

void GcodeSuite::G100() {
  #define TEST_STEP_DISTANCE 3
  #define CALC_LIMIT(axis) \
      safe_distance = parser.value_bool() ? TEST_STEP_DISTANCE : -TEST_STEP_DISTANCE; \
      while (dexarm_position_is_reachable(xyz)) { \
        xyz.axis += safe_distance; \
      } \
      xyz.axis -= safe_distance; \
      is_seen = true;

  xyze_pos_t xyz = current_position;
  int8_t safe_distance;  // mm
  bool is_seen = false;
  if (IsRunning()) {
    if (parser.seenval('X')) {
      CALC_LIMIT(x);
    } else if (parser.seenval('Y')) {
      CALC_LIMIT(y);
    }  else if (parser.seenval('Z')) {
      CALC_LIMIT(z);
    }
    if (is_seen) {
      destination = xyz;
      SERIAL_ECHOLNPAIR("MOVE TO LIMIT x:", destination[X_AXIS], 
                          " y:", destination[Y_AXIS], " z:", destination[Z_AXIS]);
      if (parser.linearval('F') > 0)
        feedrate_mm_s = parser.value_feedrate();
      prepare_line_to_destination();
    }
  }
}
