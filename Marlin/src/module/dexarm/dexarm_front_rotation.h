/*
 * bsp_serial_servo.h
 *
 *  Created on: Jun 15, 2020
 *      Author: fbc
 */
#pragma once

#include "string.h"
#include "stdio.h"
#include <stdbool.h>


#define CMD_MAX_LEN 50

extern uint8_t cmd_buf[CMD_MAX_LEN];

//指令
#define  PING_CMD		0x01
#define  READ_CMD		0x02
#define  WRITE_CMD		0x03

//REG
#define TORQUE_REG 				0X22	//扭矩限制		读取2字节
#define POS_REG 				0x24	//当前位置		读取2字节
#define ANGLE_SPEED_REG 		0x26	//当前角速度	读取2字节
#define BURDEN_REG 				0x28	//当前负载		读取2字节
#define MIN_ANGLE_REG			0x06	//最小角度		读取2字节
#define MAX_ANGLE_REG			0x08	//最大角度		读取2字节
#define MOTION_SPEED_REG 		0x20	//运动速度		读取2字节
#define VOLTAGE_REG 			0x2A	//当前电压		读取1字节
#define TEMP_REG		 		0x2B	//当前温度		读取1字节
#define EDITION_REG				0x2C	//当前版本		读取1字节

#define BONED_SPEED				0X04	//波特率		读取1字节

#define TORQUE_ENABLE_REG				0x18	//当前扭矩是否使能	读取1字节

#define TARGET_POS_REG			0x1E	//旋转
#define RELATION_POS_REG			0x1F	//旋转
#define ROTATION_REG			0x1D	//旋转

#define	EDITION	0x70		//1.1.2	= 112 = 0x70



//SERO ID

enum sero_x
{
	SERO_1	=	1,
	SERO_2,
	SERO_3,
	SERO_4,
	SERO_5,
};

enum sero_update
{
	ENTER_BOOT	=	1,
	REV_SIZE,
	REV_BIN,
};

void front_rotation_init(void);
bool set_pos(int id,int val);
bool set_relation_pos(int id,int val);
bool set_rotation_pos(int id,int val);
void pos_demo_test();
uint16_t set_motion_speed(uint8_t id,int val);
int scope_limit(int min,int val,int max);
float scope_limit_float(float min,float val,float max);
bool set_enable(int id,int val);
uint16_t set_torque_limt(uint8_t id,int val);
uint16_t read_motion_speed(uint8_t id);
uint16_t read_pos(uint8_t id);
uint16_t read_edition(uint8_t id);
void update_rotation_model(uint8_t flag,uint16_t bin_size,uint8_t bin[]);
char rotation_model_rev_bin(char c);

extern char rev_buffer[500];
extern void clear_front_val(void);

#define KEY_Pin GPIO_PIN_10
#define KEY_GPIO_Port GPIOA