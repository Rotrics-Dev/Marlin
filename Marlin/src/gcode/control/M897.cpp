#include "../gcode.h"
#include "../../module/planner.h"

// Movement status
void GcodeSuite::M897()
{
  if (planner.has_blocks_queued()) {
    SERIAL_ECHOLN("Movement");
  } else {
    SERIAL_ECHOLN("Completed");
  }
}
