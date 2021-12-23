#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/dexarm/dexarm_conveyor_belt.h"

// Conveyor belt contorl

void GcodeSuite::M2012() {
	feedRate_t e_feedrate_mm_s = 0;
	bool direction = false;
  if (parser.linearval('F') > 0)
    e_feedrate_mm_s = parser.value_feedrate();
	if (parser.linearval('D') > 0)
    direction =parser.boolval('D', true);

  if (e_feedrate_mm_s) {
	  conveyor_belt.turn_on(e_feedrate_mm_s, direction);
  } else {
    conveyor_belt.turn_off();
  }
  conveyor_belt.report_status();
}

void GcodeSuite::M2013() {
	conveyor_belt.turn_off();
  conveyor_belt.report_status();
}

void GcodeSuite::M2014() {
  conveyor_belt.report_status();
}
