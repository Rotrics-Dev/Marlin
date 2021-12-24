#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/dexarm/dexarm_front_rotation.h"

// front rotation init
void GcodeSuite::M2100()
{
  dexarm_rotation.init();
  dexarm_rotation.clear_front_val();
  dexarm_rotation.enable(0);
  HAL_Delay(100);
}

void GcodeSuite::M2101()
{
  planner.synchronize();
  static int speed = 0;
  float positon = 0.0f;
  static uint8_t e_flag = 0;

  if (!dexarm_rotation.is_init())
  {
    dexarm_rotation.init();
    MYSERIAL0.println("front rotation init ok......\r\n");
  }
  if (parser.seen('P'))
  {
    positon = parser.floatval('P');
    dexarm_rotation.set_pos(positon);
  }
  else if (parser.seen('R'))
  {
    positon = parser.floatval('R');
    dexarm_rotation.set_relation_pos(positon);
  }
  else if (parser.seen('S'))
  {
    speed = parser.intval('S');
    speed = dexarm_rotation.scope_limit(-100, speed, 100);
    dexarm_rotation.set_rotation_pos((-1) * speed);
  }
  else if (parser.seen('E'))
  {
    e_flag = parser.intval('E');
    dexarm_rotation.enable(e_flag);
  }
  else {
    dexarm_rotation.report_pos();
  }
}

// update front rotation model bin
uint16_t front_rotation_model_bin_size = 0;
void GcodeSuite::M2102()
{
  uint8_t update_step = 0; // defult enter boot
  bool update_flag = parser.seen('U');
  if (update_flag)
  {
    update_step = parser.intval('U');
    if (update_step == REV_SIZE)
    {
      front_rotation_model_bin_size = parser.intval('S');
    }
  }
  dexarm_rotation.update(update_step, front_rotation_model_bin_size);
}

void GcodeSuite::M2103()
{
  uint16_t edition = 0;

  edition = dexarm_rotation.read_edition();
  HAL_Delay(100);

  uint16_t a = edition / 100;
  if (a)
  {
    uint16_t b = (edition - 100) / 10;
    uint16_t c = edition % 10;

    char str[50];
    memset(&str, 0, 50);
    sprintf(str, "Rotary Firmware V%d.%d.%d", a, b, c);
    MYSERIAL0.println(str);
  }
  else
  {
    MYSERIAL0.println("Rotary Firmware Read Failed");
  }
}
