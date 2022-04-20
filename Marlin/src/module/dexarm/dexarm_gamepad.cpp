#include "dexarm_gamepad.h"
#include "../../inc/MarlinConfig.h"
#include "dexarm.h"
#include "../planner.h"

GamepadControl gamepad_control;

bool is_limit_pos(const xyz_pos_t &position)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	int tmp_z = (int(z + 0.5) + abs(ZMINARM)) / 2 + 0.5;
	float tmps = sqrt(x * x + y * y) - dexarm_offset;

	if (z > ZMAXARM || z < ZMINARM)
	{
		return false;
	}

	if (y < 0)
	{
		return false;
	}

	if (tmps < ARMLIMIT[tmp_z * 2 + 1] + 1 || tmps > ARMLIMIT[tmp_z * 2] - 1)
	{
		return false;
	}
	return true;
}

void GamepadControl::line_move_mode() {
  float move_step = 0.15;
  xyze_pos_t xyz = current_position;
  if (planner.movesplanned() <= 2) {
    switch (status_) {
      case GAMEPAD_LINE_MOVE_X_MIN:
        xyz.x -= move_step;
        break;
      case GAMEPAD_LINE_MOVE_X_MAX:
        xyz.x += move_step;
        break;
      case GAMEPAD_LINE_MOVE_Y_MIN:
        xyz.y -= move_step;
        break;
      case GAMEPAD_LINE_MOVE_Y_MAX:
        xyz.y += move_step;
        break;
      case GAMEPAD_LINE_MOVE_Z_MIN:
        xyz.z -= move_step;
        break;
      case GAMEPAD_LINE_MOVE_Z_MAX:
        xyz.z += move_step;
        break;
      case GAMEPAD_RAIL_MOVE_MAX:
        xyz.e += move_step;
        break;
      case GAMEPAD_RAIL_MOVE_MIN:
        xyz.e -= move_step;
        break;
      default:
        return;
    }
    if (!is_limit_pos(xyz)) {
      status_ = GAMEPAD_NO_MOVE;
    } else {
      feedRate_t old_feedrate = feedrate_mm_s;
      float acc = planner.settings.travel_acceleration;
      planner.settings.travel_acceleration = 5000;
      feedrate_mm_s = GAMEPAD_LINE_MOVE_FEEDRATE;
      destination = xyz;
      prepare_fast_move_to_destination();
      feedrate_mm_s = old_feedrate;
      planner.settings.travel_acceleration = acc;
    }
  }
}

void GamepadControl::angle_move_mode() {
  float move_angle = 0.1;
  abc_pos_t abc = delta;
  if (planner.movesplanned() <= 8) {
    switch (status_) {
        case GAMEPAD_ANGLE_MOVE_X_MIN:
          abc.x -= move_angle;
          break;
        case GAMEPAD_ANGLE_MOVE_X_MAX:
          abc.x += move_angle;
          break;
        case GAMEPAD_ANGLE_MOVE_Y_MIN:
          abc.y -= move_angle;
          break;
        case GAMEPAD_ANGLE_MOVE_Y_MAX:
          abc.y += move_angle;
          break;
        case GAMEPAD_ANGLE_MOVE_Z_MIN:
          abc.z -= move_angle;
          break;
        case GAMEPAD_ANGLE_MOVE_Z_MAX:
          abc.z += move_angle;
          break;
      default:
        return;
    }
    forward_kinematics_DEXARM(abc);
    current_position = cartes;
    if (!is_limit_pos(current_position)) {
      status_ = GAMEPAD_NO_MOVE;
      set_current_position_from_position_sensor();
      destination = current_position;
      prepare_fast_move_to_destination();
    } else {
      feedRate_t old_feedrate = feedrate_mm_s;
      float acc = planner.settings.travel_acceleration;
      planner.settings.travel_acceleration = 10000;
      planner.buffer_angle(abc.x, abc.y, abc.z, GAMEPAD_LINE_MOVE_FEEDRATE);
      feedrate_mm_s = old_feedrate;
      planner.settings.travel_acceleration = acc;
    }
  }
}



void GamepadControl::set_status(gamepad_status_e status) {
  status_ = status;
  if (status_ == GAMEPAD_NO_MOVE) {
    set_current_position_from_position_sensor();
    destination = current_position;
    prepare_fast_move_to_destination();
  } else {
    refresh_movement();
  }
}

void GamepadControl::refresh_movement() {
  if (status_ != GAMEPAD_NO_MOVE) {
    if (status_ <= GAMEPAD_RAIL_MOVE_MAX) {
      line_move_mode();
    } else if (status_ <= GAMEPAD_ANGLE_MOVE_Z_MAX) {
      angle_move_mode();
    }
  }
}

void GamepadControl::loop() {
  refresh_movement();
}
