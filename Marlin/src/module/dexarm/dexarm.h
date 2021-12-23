#pragma once

#include "dexarm_position_config.h"
#include "dexarm_position_reachable.h"
#include "dexarm_position_sensor.h"
#include "dexarm_update_manager.h"
#include "dexarm_front_rotation.h"
#include "dexarm_conveyor_belt.h"
#include "dexarm_sliding_rail.h"
#include "dexarm_air_pump.h"

//Dexarm config
extern int calibration_position_sensor_value[3];
extern int door_open_message_counter;
extern float x_axis_scaling_factor;
extern float y_axis_scaling_factor;
extern float front_module_offset;
extern float delta_segments_per_second;

extern bool laser_protection_enable_flag;
extern bool laser_door_open_flag;
extern bool laser_fan_flag;

extern bool position_init_flag;
extern bool INVERT_E0_DIR;

typedef enum {
    FAST_MODE,
    LINE_MODE,
    JUMP_MODE,
}move_mode_t;

extern move_mode_t G0_MOVE_MODE;

void print_current_module_type();
void module_position_init();
void update_dexarm_offset(void);

void get_current_encoder();
void get_current_position_from_position_sensor(xyz_pos_t &position);
void set_current_position_from_position_sensor();
void process_encoder(int x, int y, int z);

int position_M1111();
int m1112_position(xyz_pos_t &position);
int m1113_position(xyz_pos_t &position);

void forward_kinematics_DEXARM(abc_pos_t &angle);
void forward_kinematics_DEXARM_position(abc_pos_t &angle, xyz_pos_t &position);
void rotate_angle_diff(abc_pos_t &angle_abc);
void inverse_kinematics(const xyz_pos_t &raw);

bool dexarm_position_is_reachable(const xyz_pos_t &position);

void dexarm_report_positions();
void dexarm_loop();

