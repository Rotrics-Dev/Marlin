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

//指令
#define PING_CMD 0x01
#define READ_CMD 0x02
#define WRITE_CMD 0x03

// REG
#define TORQUE_REG 0X22				//扭矩限制		读取2字节
#define POS_REG 0x24					//当前位置		读取2字节
#define ANGLE_SPEED_REG 0x26	//当前角速度	读取2字节
#define BURDEN_REG 0x28				//当前负载		读取2字节
#define MIN_ANGLE_REG 0x06		//最小角度		读取2字节
#define MAX_ANGLE_REG 0x08		//最大角度		读取2字节
#define MOTION_SPEED_REG 0x20 //运动速度		读取2字节
#define VOLTAGE_REG 0x2A			//当前电压		读取1字节
#define TEMP_REG 0x2B					//当前温度		读取1字节
#define EDITION_REG 0x2C			//当前版本		读取1字节

#define BONED_SPEED 0X04 //波特率		读取1字节

#define TORQUE_ENABLE_REG 0x18 //当前扭矩是否使能	读取1字节

#define TARGET_POS_REG 0x1E		//旋转
#define RELATION_POS_REG 0x1F //旋转
#define ROTATION_REG 0x1D			//旋转

#define EDITION 0x70 // 1.1.2	= 112 = 0x70

// SERO ID

enum sero_x
{
	SERO_1 = 1,
	SERO_2,
	SERO_3,
	SERO_4,
	SERO_5,
};

enum sero_update
{
	ENTER_BOOT = 1,
	REV_SIZE,
	REV_BIN,
};

#define DATA_BUF_LEN 30
typedef struct _r_data
{
	uint8_t buf[DATA_BUF_LEN];
	uint8_t len;
} data_typedef;

class DexarmRotation
{
public:
	void init(void);
	void init(uint8_t sero_id);
	void loop(void);
	void report_pos(void);
	bool set_pos(float val);
	bool set_relation_pos(float val);
	bool set_rotation_pos(int val);
	uint16_t set_motion_speed(int val);
	int scope_limit(int min, int val, int max);
	float scope_limit_float(float min, float val, float max);
	bool enable(int val);
	uint16_t set_torque_limt(int val);
	uint16_t read_motion_speed();
	uint16_t read_pos();
	uint16_t read_edition();
	void update(uint8_t flag, uint16_t bin_size);
	char recv_bin(char c);
	void clear_front_val(void);
	bool is_init() { return inited; }
	uint16_t read_enable();
	bool read_torque_limt();
	uint16_t read_min_pos();
	uint16_t read_max_pos();
	bool set_min_pos(int val);
	bool set_max_pos(int val);
	uint16_t read_bps();
	uint16_t set_bps(int val);

private:
	uint16_t check_sum(data_typedef data);
	data_typedef w_data_buf(uint8_t cmd, uint8_t addr, int16_t info);
	void usart_get_flag_wait();
	bool write_info(uint8_t reg, int16_t val);
	data_typedef r_data_buf(uint8_t cmd, uint8_t addr, int addr_len);
	uint8_t ret_reg_count(uint8_t reg);
	uint16_t read_info(uint8_t reg);
	void build_bin_pack();
	void build_one_fps(char buf[], uint16_t len);
	void HexStrToByte(char *source, char *dest, int sourceLen);
	void read_dev_answer();
	void usart_send_buf(UART_HandleTypeDef huart, uint8_t *pData, uint16_t len);
	void usart_rev_buf(UART_HandleTypeDef huart, uint8_t *pData, uint16_t len);

private:
	bool inited = false;
	uint8_t id = SERO_1;
	bool is_move = false;
	bool enabled = false;
	int last_positon = -1;
	uint32_t loop_time = 0;
	uint16_t target_pos = 0;
	bool relation = false;
};
extern DexarmRotation dexarm_rotation;
