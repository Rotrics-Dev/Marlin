#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../gcode/gcode.h"
#include "../planner.h"
#include "../motion.h"
#include "../../feature/tmc_util.h"
#include "../stepper.h"
#include "../endstops.h"
#include "dexarm.h"

static bool sliding_rail_homed = false;

void sliding_rail_home(feedRate_t home_feedrate_high=30, feedRate_t home_feedrate_low=10, int16_t homing_threshold_first=60, int16_t homing_threshold_second=30) {
  sensorless_t stealth_states;
  feedRate_t old_feedrate;
  
  INVERT_E0_DIR = false;
  planner.settings.axis_steps_per_mm[E0_AXIS] = 170.4;

  SERIAL_ECHOLNPAIR("Home E Axis 1 Fast start");
  stepperE0.homing_threshold(homing_threshold_first);
  stealth_states.e0 = tmc_enable_stallguard(stepperE0);
  endstops.enable_globally(true);
  destination[A_AXIS] = current_position[A_AXIS];
  destination[B_AXIS] = current_position[B_AXIS];
  destination[C_AXIS] = current_position[C_AXIS];
  destination[E0_AXIS] = -1500;
  old_feedrate = feedrate_mm_s;
  feedrate_mm_s = home_feedrate_high;
  prepare_line_to_destination();
  planner.synchronize();
  current_position[E0_AXIS] = 0;
  sync_plan_position();
  endstops.validate_homing_move();
  tmc_disable_stallguard(stepperE0, stealth_states.e0);
  endstops.enable_globally(false);
  SERIAL_ECHOLNPAIR("Home E Axis 1 fast end");

  SERIAL_ECHOLNPAIR("Move Away E Axis 1 start");
  destination[A_AXIS] = current_position[A_AXIS];
  destination[B_AXIS] = current_position[B_AXIS];
  destination[C_AXIS] = current_position[C_AXIS];
  destination[E0_AXIS] = 10;
  feedrate_mm_s = home_feedrate_high;
  prepare_line_to_destination();
  planner.synchronize();
  current_position[E0_AXIS] = 0;
  sync_plan_position();
  SERIAL_ECHOLNPAIR("Move Away E Axis 1 end");

  SERIAL_ECHOLNPAIR("Home E Axis 2 Slow start");
  stepperE0.homing_threshold(homing_threshold_second);
  stealth_states.e0 = tmc_enable_stallguard(stepperE0);
  endstops.enable_globally(true);
  destination[A_AXIS] = current_position[A_AXIS];
  destination[B_AXIS] = current_position[B_AXIS];
  destination[C_AXIS] = current_position[C_AXIS];
  destination[E0_AXIS] = -100;
  feedrate_mm_s = home_feedrate_low;
  prepare_line_to_destination();
  planner.synchronize();
  current_position[E0_AXIS] = 0;
  sync_plan_position();
  endstops.validate_homing_move();
  tmc_disable_stallguard(stepperE0, stealth_states.e0);
  endstops.enable_globally(false);
  SERIAL_ECHOLNPAIR("Home E Axis 2 Slow end");

  SERIAL_ECHOLNPAIR("Move Away E Axis 2 start");
  destination[A_AXIS] = current_position[A_AXIS];
  destination[B_AXIS] = current_position[B_AXIS];
  destination[C_AXIS] = current_position[C_AXIS];
  destination[E0_AXIS] = 2;
  feedrate_mm_s = home_feedrate_low;
  prepare_line_to_destination();
  planner.synchronize();
  current_position[E0_AXIS] = 0;
  sync_plan_position();
  SERIAL_ECHOLNPAIR("Move Away E Axis 2 end");

  feedrate_mm_s = old_feedrate;
  sliding_rail_homed = true;
}

bool is_sliding_rail_homed() {
  return sliding_rail_homed;
}