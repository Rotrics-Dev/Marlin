#pragma once

#define MATH_TRANS 57.296
#define ARM_L 150.0

//Front-end module
#define LASER_MODULE_OFFSET 88.217
#define PUMP_MODULE_OFFSET 105
#define PEN_MODULE_OFFSET 101.5
#define _3D_MODULE_OFFSET 92.6
#define ROTARY_MODULE_OFFSET 109.5
#define CAMERA_MODULE_OFFSET 107.7

#define START_A_ANGLE 90.0
#define START_B_ANGLE 105.24
#define START_C_ANGLE 99.60
#define OFFSET_ROT 12.5

#define START_X 0
#define START_Y 109.55
#define START_Z 169.66

#if defined(USING_MT6701_POSITION_SENSOR)
    #define MAX_POSITION_SENSOR_RANGE 16384              // MT6701
    #define MAX_POSITION_SENSOR_DIFF 2570*4
    #define LESS_THAN_VAL <3   
#else
    #define MAX_POSITION_SENSOR_RANGE 4096              // AS5600
    #define MAX_POSITION_SENSOR_DIFF 2570
    #define LESS_THAN_VAL ==0   
#endif

#define NUM_AXIS 4

//Z-axis calibration parameters
#define X_AXIS_SLOPE_MAX 1
#define X_AXIS_SLOPE_MIN -1
#define Y_AXIS_SLOPE_MAX 1
#define Y_AXIS_SLOPE_MIN -1

#define Z_AXIS_TARTGET_POSITION_CALIBRATION_ALGORUTHM target[Z_AXIS] += (X_KTMP * target[X_AXIS] + Y_KTMP * (target[Y_AXIS] - 200));
#define Z_AXIS_CURRENT_POSITION_CALIBRATION_ALGORUTHM current_position[Z_AXIS] += (X_KTMP * current_position[X_AXIS] + Y_KTMP * current_position[Y_AXIS] - Y_KTMP * 200);