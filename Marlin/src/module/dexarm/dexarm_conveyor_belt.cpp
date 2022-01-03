#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../gcode/gcode.h"
#include "HAL.h"

ConveyorBelt conveyor_belt;

void ConveyorBelt::turn_on(float feedRate_t, bool direction) {
  int rail_feedrate;
  feedrate = feedRate_t;
  dir = direction;
  rail_feedrate = int(65535 / feedRate_t);
  OUT_WRITE(E0_ENABLE_PIN, LOW);
  if (direction == 0) {
    OUT_WRITE(E0_DIR_PIN, LOW);
  } else if (direction == 1) {
    OUT_WRITE(E0_DIR_PIN, HIGH);
  }
  HAL_timer_start(RAIL_TIMER_NUM, 4 * planner.settings.axis_steps_per_mm[E_AXIS]);
  ENABLE_RAIL_INTERRUPT();
  HAL_timer_set_compare(RAIL_TIMER_NUM, rail_feedrate);
}

void ConveyorBelt::turn_off() {
  DISABLE_RAIL_INTERRUPT();
  feedrate = 0;
}

void ConveyorBelt::report_status() {
  feedrate *= 60;
  if (feedrate) {
    if (dir) {
      SERIAL_ECHOLNPAIR("Conveyor belt forward ", feedrate, "mm/min ");
    } else {
      SERIAL_ECHOLNPAIR("Conveyor belt backward ", feedrate, "mm/min ");
    }
  } else {
    SERIAL_ECHOLNPAIR("Conveyor belt stop");
  }
}

HAL_RAIL_TIMER_ISR()
{
  HAL_timer_isr_prologue(RAIL_TIMER_NUM);

  TOGGLE(E0_STEP_PIN);

  HAL_timer_isr_epilogue(RAIL_TIMER_NUM);
}
