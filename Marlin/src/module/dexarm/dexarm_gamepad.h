#ifndef DEXARM_GAMEPAD_H
#define DEXARM_GAMEPAD_H

#define GAMEPAD_LINE_MOVE_FEEDRATE 18000

typedef enum {
  GAMEPAD_NO_MOVE,
  GAMEPAD_LINE_MOVE_X_MIN,
  GAMEPAD_LINE_MOVE_X_MAX,
  GAMEPAD_LINE_MOVE_Y_MIN,
  GAMEPAD_LINE_MOVE_Y_MAX,
  GAMEPAD_LINE_MOVE_Z_MIN,
  GAMEPAD_LINE_MOVE_Z_MAX,
  GAMEPAD_RAIL_MOVE_MIN,
  GAMEPAD_RAIL_MOVE_MAX,
  GAMEPAD_ANGLE_MOVE_X_MIN,
  GAMEPAD_ANGLE_MOVE_X_MAX,
  GAMEPAD_ANGLE_MOVE_Y_MIN,
  GAMEPAD_ANGLE_MOVE_Y_MAX,
  GAMEPAD_ANGLE_MOVE_Z_MIN,
  GAMEPAD_ANGLE_MOVE_Z_MAX,
} gamepad_status_e;

class GamepadControl {
  public:
    void set_status(gamepad_status_e status);
    void loop();
  private:
    void line_move_mode();
    void angle_move_mode();
    void refresh_movement();
  private:
    gamepad_status_e status_ = GAMEPAD_NO_MOVE;
};

extern GamepadControl gamepad_control;
#endif
