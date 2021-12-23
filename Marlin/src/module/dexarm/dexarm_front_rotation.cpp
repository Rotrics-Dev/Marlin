/*
 * front_rotation.c
 *
 *  Created on: Jun 15, 2020
 *      Author: fbc
 */
#include "HAL.h"

#include "stm32f4xx_hal.h"
#include "dexarm_front_rotation.h"
#include "interrupt.h"
#include "../../module/stepper/indirection.h"

#define KEY_Pin GPIO_PIN_10
#define KEY_GPIO_Port GPIOA

DexarmRotation dexarm_rotation;

//协议头
uint8_t cmd_head[2] = {0xFF, 0xFF};

UART_HandleTypeDef huart1;

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */
	// 1000000
	// 115200 117647
	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 117647;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
	{
		// Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

static uint8_t last_value = 0;
void front_button_callback()
{
	if (dexarm_rotation.is_init())
	{
		// MYSERIAL0.println("Front button is pressed!\r\n");
		if (last_value == 1)
		{
			if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
			{
				SERIAL_ECHOLNPAIR("Button Pressed");
				DISABLE_AXIS_X();
				DISABLE_AXIS_Y();
				DISABLE_AXIS_Z();
				last_value = 0;
			}
		}
		if (last_value == 0)
		{
			if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
			{
				SERIAL_ECHOLNPAIR("Button Released");
				ENABLE_AXIS_X();
				ENABLE_AXIS_Y();
				ENABLE_AXIS_Z();
				xyz_pos_t position;
				get_current_position_from_position_sensor(position);
				SERIAL_ECHOLNPAIR("X:", position.x, " Y:", position.y, " Z:", position.z);
				last_value = 1;
			}
		}
	}
}

void DexarmRotation::init(void)
{
	init(SERO_1);
}
void DexarmRotation::init(uint8_t sero_id)
{
	id = sero_id;
	pinMode(PA10, INPUT_PULLUP);
	stm32_interrupt_enable(KEY_GPIO_Port, KEY_Pin, front_button_callback, GPIO_MODE_IT_RISING_FALLING);
	last_value = 1;
	MX_USART1_UART_Init();
	HAL_Delay(300);
	inited = true;
}

//除去协议头计算 ~校检和
uint16_t DexarmRotation::check_sum(data_typedef data)
{
	uint16_t check_val = 0;
	int offset = 2;
	for (int i = offset; i < data.len + offset + 1; i++)
	{
		check_val += data.buf[i];
	}

	check_val = ~check_val & 0x00FF;

	return check_val;
}

data_typedef DexarmRotation::w_data_buf(uint8_t cmd, uint8_t addr, int16_t info)
{
	data_typedef data;
	memset(&data, 0, sizeof(data_typedef));
	uint8_t index = 4;
	data.buf[0] = cmd_head[0];
	data.buf[1] = cmd_head[1];
	data.buf[2] = id;
	data.len++;
	data.buf[index++] = cmd;
	data.len++;
	data.buf[index++] = addr;
	data.len++;
	data.buf[index++] = info & 0xFF;
	data.len++;
	data.buf[index++] = info >> 8;
	data.len++;

	data.buf[3] = data.len;
	data.len++;
	data.buf[index++] = check_sum(data);

	return data;
}

void DexarmRotation::usart_get_flag_wait()
{
	int outtime = 1000 * 30;
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET)
	{
		outtime--;
		if (outtime < 0)
		{
			MYSERIAL0.println("receive time out......\r\n");
			return;
		}
	}
}

bool DexarmRotation::write_info(uint8_t reg, int val)
{
	uint8_t send_num = 0;
	data_typedef temp;
	uint8_t rev_buf[DATA_BUF_LEN];
	memset(&temp, 0, sizeof(data_typedef));
	memset(rev_buf, 0, sizeof(uint8_t) * DATA_BUF_LEN);
	temp = w_data_buf(WRITE_CMD, reg, val);
	rev_buf[4] = 1; // status flag
	do
	{
		HAL_HalfDuplex_EnableTransmitter(&huart1);
		// HAL_UART_Transmit(&huart1, temp.buf, temp.len+3, 1000);
		HAL_UART_Transmit(&huart1, temp.buf, 10, 1000);

		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET); //等待发送结束

		uint8_t rev_len = 0;

		//先接收4 个字节
		//两个协议头 一个ID 一个长度
		HAL_HalfDuplex_EnableReceiver(&huart1);
		//		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
		usart_get_flag_wait();
		HAL_UART_Receive(&huart1, rev_buf, 4, 1000);

		if (rev_buf[0] == 0xFF && rev_buf[1] == 0xFF)
		{
			rev_len = rev_buf[3]; //获取长度
		}

		HAL_HalfDuplex_EnableReceiver(&huart1);
		//		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
		usart_get_flag_wait();
		HAL_UART_Receive(&huart1, rev_buf + 4, rev_len, 1000);
		send_num++;

		if (send_num > 1)
		{
			rev_buf[4] = 0;
			MYSERIAL0.println("ERROR:send font model cmd fail!!!\r\n");
		}

	} while (rev_buf[4]); //发送失败,重发
	return true;
}

data_typedef DexarmRotation::r_data_buf(uint8_t cmd, uint8_t addr, int addr_len)
{
	data_typedef data;
	memset(&data, 0, sizeof(data_typedef));
	data.buf[0] = cmd_head[0];
	data.buf[1] = cmd_head[1];
	data.buf[2] = id;
	data.len++;
	data.buf[4] = cmd;
	data.len++;
	data.buf[5] = addr;
	data.len++;
	data.buf[6] = addr_len;
	data.len++;
	data.buf[3] = data.len;

	data.buf[7] = check_sum(data);

	return data;
}

uint8_t DexarmRotation::ret_reg_count(uint8_t reg)
{
	uint8_t ret_val = 0;
	switch (reg)
	{
	case TORQUE_REG:
		ret_val = 2;
		break;
	case POS_REG:
		ret_val = 2;
		break;
	case ANGLE_SPEED_REG:
		ret_val = 2;
		break;
	case BURDEN_REG:
		ret_val = 2;
		break;
	case MOTION_SPEED_REG:
		ret_val = 2;
		break;
	case MIN_ANGLE_REG:
		ret_val = 2;
		break;
	case MAX_ANGLE_REG:
		ret_val = 2;
		break;
	case VOLTAGE_REG:
		ret_val = 1;
		break;
	case TEMP_REG:
		ret_val = 1;
		break;
	case TORQUE_ENABLE_REG:
		ret_val = 1;
		break;
	case BONED_SPEED:
		ret_val = 1;
		break;
	case EDITION_REG:
		ret_val = 1;
		break;

	default:
		break;
	}
	return ret_val;
}

uint16_t DexarmRotation::read_info(uint8_t reg)
{
	uint8_t rev_buf[DATA_BUF_LEN] = {0};
	data_typedef temp;
	temp = r_data_buf(READ_CMD, reg, ret_reg_count(reg));

	HAL_HalfDuplex_EnableTransmitter(&huart1);
	HAL_UART_Transmit(&huart1, temp.buf, temp.len + 4, 1000);

	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET); //等待发送结束

	uint8_t rev_len = 0;

	//先接收4 个字节
	//两个协议头 一个ID 一个长度
	HAL_HalfDuplex_EnableReceiver(&huart1);
	//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
	usart_get_flag_wait();
	HAL_UART_Receive(&huart1, rev_buf, 4, 1000);

	if (rev_buf[0] == 0xFF && rev_buf[1] == 0xFF)
	{
		rev_len = rev_buf[3]; //获取长度
	}

	//状态
	// rev_buf[4]
	//根据长度接收剩下的字符

	HAL_HalfDuplex_EnableReceiver(&huart1);
	//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
	usart_get_flag_wait();
	HAL_UART_Receive(&huart1, rev_buf + 4, rev_len, 1000);

	//结果

	volatile uint16_t info_val = 0;
	if (ret_reg_count(reg) == 2)
		info_val = rev_buf[6] << 8 | rev_buf[5];
	else if (ret_reg_count(reg) == 1)
		info_val = rev_buf[5];

	return info_val;
}

int DexarmRotation::scope_limit(int min, int val, int max)
{
	if (val <= min)
	{
		val = min;
	}
	else if (val >= max)
	{
		val = max;
	}
	return val;
}

float DexarmRotation::scope_limit_float(float min, float val, float max)
{
	if (val <= min)
	{
		val = min;
	}
	else if (val >= max)
	{
		val = max;
	}
	return val;
}

//同步写入
void sync_write()
{
}

// 读取位置
uint16_t DexarmRotation::read_pos()
{
	return read_info(POS_REG);
}

// 读取版本号
uint16_t DexarmRotation::read_edition()
{
	return read_info(EDITION_REG);
}

//读取扭矩是否使能
uint16_t DexarmRotation::read_enable()
{
	return read_info(TORQUE_ENABLE_REG);
}

//设置扭矩限制
uint16_t DexarmRotation::set_torque_limt(int val)
{
	return write_info(TORQUE_REG, val);
}

//读取扭矩限制
bool DexarmRotation::read_torque_limt()
{
	return read_info(TORQUE_REG);
}

// 读取最小位置
uint16_t DexarmRotation::read_min_pos()
{
	return read_info(MIN_ANGLE_REG);
}

// 读取最大位置
uint16_t DexarmRotation::read_max_pos()
{
	return read_info(MAX_ANGLE_REG);
}

// 设置最小位置
bool DexarmRotation::set_min_pos(int val)
{
	return write_info(MIN_ANGLE_REG, val);
}

// 设置最大位置
bool DexarmRotation::set_max_pos(int val)
{
	return write_info(MAX_ANGLE_REG, val);
}
//读速度
uint16_t DexarmRotation::read_motion_speed()
{
	return read_info(MOTION_SPEED_REG);
}
//设置速度
uint16_t DexarmRotation::set_motion_speed(int val)
{
	return write_info(MOTION_SPEED_REG, val);
}

//读速度
uint16_t DexarmRotation::read_bps()
{
	return read_info(BONED_SPEED);
}
//设置速度
uint16_t DexarmRotation::set_bps(int val)
{
	return write_info(BONED_SPEED, val);
}

// 设置位置
bool DexarmRotation::set_pos(int val)
{
	is_move = true;
	return write_info(TARGET_POS_REG, val);
}

// 设置相对位置
bool DexarmRotation::set_relation_pos(int val)
{
	is_move = true;
	return write_info(RELATION_POS_REG, val);
}

// 设置旋转
bool DexarmRotation::set_rotation_pos(int val)
{
	is_move = true;
	return write_info(ROTATION_REG, val);
}

// 设置使能
bool DexarmRotation::enable(int val)
{
	enabled = val;
	return write_info(TORQUE_ENABLE_REG, val);
}

// //设置最大位置最小位置
// //0-1023
// void max_min_pos_demo_test()
// {
// 	int min_val =0;
// 	int max_val =0;
// 	// 读取
// 	min_val = read_max_pos(SERO_1);
// 	max_val = read_min_pos(SERO_1);
// 	HAL_Delay(200);
// 	// 设置
// 	set_min_pos(SERO_1,0);
// 	HAL_Delay(200);//延时不可省略
// 	set_max_pos(SERO_1,1000);
// 	// 读取
// 	HAL_Delay(200);
// 	min_val = read_max_pos(SERO_1);
// 	max_val = read_min_pos(SERO_1);

// }

void DexarmRotation::usart_send_buf(UART_HandleTypeDef huart, uint8_t *pData, uint16_t len)
{
	HAL_HalfDuplex_EnableTransmitter(&huart);
	HAL_UART_Transmit(&huart, pData, len, 1000);
	while (__HAL_UART_GET_FLAG(&huart, UART_FLAG_TC) == RESET)
	{
	};
	memset(&pData, 0, len * sizeof(uint8_t));
}

void DexarmRotation::usart_rev_buf(UART_HandleTypeDef huart, uint8_t *pData, uint16_t len)
{
	HAL_HalfDuplex_EnableReceiver(&huart);
	while (__HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE) == RESET)
		;
	// usart_get_flag_wait();
	HAL_UART_Receive(&huart, pData, len, 1000);
	memset(&pData, 0, len * sizeof(uint8_t));
}

void DexarmRotation::read_dev_answer()
{
	uint8_t pData[10] = {0};
	usart_rev_buf(huart1, pData, 4);

	if (strstr((const char *)pData, "ok") == NULL)
	{
		MYSERIAL0.println("answer fail!!!");
	}
	else
	{

		MYSERIAL0.println("found,rotation model answer ok !!!");
	}
}

uint8_t start_rev_bin = 0;
int rev_bin_byte_num = 0;
int rev_one_fps_dat_flag = 0;
#define PACK_SIZE 500
char recv_buffer[500];
char bin_buf[PACK_SIZE];
int bin_size = 0;

void DexarmRotation::clear_front_val()
{
	start_rev_bin = 0;
	rev_bin_byte_num = 0;
	rev_one_fps_dat_flag = 0;
	memset(recv_buffer, 0, 500);
	memset(bin_buf, 0, PACK_SIZE);
	bin_size = 0;
}

void DexarmRotation::HexStrToByte(char *source, char *dest, int sourceLen)
{
	short i;
	unsigned char highByte, lowByte;

	for (i = 0; i < sourceLen; i += 2)
	{
		highByte = toupper(source[i]);
		lowByte = toupper(source[i + 1]);

		if (highByte > 0x39)
			highByte -= 0x37;
		else
			highByte -= 0x30;

		if (lowByte > 0x39)
			lowByte -= 0x37;
		else
			lowByte -= 0x30;

		dest[i / 2] = (highByte << 4) | lowByte;
	}
	return;
}

void DexarmRotation::build_one_fps(char buf[], uint16_t len)
{
	char str[30] = {0};
	static uint16_t offset = 0;

	static int pack_num = 0;
	static int last_pack_size = 0;
	static int pack_index = 0;

	pack_num = bin_size / PACK_SIZE;
	last_pack_size = bin_size % PACK_SIZE;

	memcpy(bin_buf + offset, buf, len);

	offset = offset + len;
	memset(str, 0, 30);
	sprintf(str, "bin size: %d", offset);
	MYSERIAL0.println(str);
	// clear flag
	rev_one_fps_dat_flag = 0;
	if (pack_index < pack_num)
	{
		if (offset >= PACK_SIZE)
		{
			usart_send_buf(huart1, (uint8_t *)bin_buf, PACK_SIZE);
			read_dev_answer();

			MYSERIAL0.println("pack_index ,500 char");
			memset(bin_buf, 0, 500);
			pack_index++;
			offset = 0;
		}
	}
	else
	{ // last pack

		if (offset >= last_pack_size)
		{
			usart_send_buf(huart1, (uint8_t *)bin_buf, last_pack_size);
			read_dev_answer();

			MYSERIAL0.println("last pack ,< 500 char");
			memset(bin_buf, 0, 500);
			pack_index++;
			offset = 0;
			start_rev_bin = 0;
		}
	}

	/*info print*/
	memset(str, 0, 30);
	sprintf(str, "pack_index: %d", pack_index);
	MYSERIAL0.println(str);

	memset(str, 0, 30);
	sprintf(str, "pack_num: %d", pack_num);
	MYSERIAL0.println(str);

	memset(str, 0, 30);
	sprintf(str, "last_pack_size: %d", last_pack_size);
	MYSERIAL0.println(str);
}

void DexarmRotation::build_bin_pack()
{
	char str[30] = {0};
	char data[100];
	uint16_t len = 0;

	HexStrToByte(recv_buffer, data, rev_bin_byte_num + 1);

	len = rev_bin_byte_num / 2 - 1;
	memset(str, 0, 30);
	sprintf(str, "bin size: %d", len);
	MYSERIAL0.println(str);

	build_one_fps(data, len);
}

void DexarmRotation::update(uint8_t flag, uint16_t len)
{
	char str[30];
	uint8_t enter_boot_cmd[4] = {0xFF, 0xFF, 0xEF, 0xFF};
	switch (flag)
	{
	// enter boot
	case ENTER_BOOT:
		usart_send_buf(huart1, enter_boot_cmd, 4);
		delay(500);
		MYSERIAL0.println("enter boot cmd");
		break;
	// receive bin size
	case REV_SIZE:

		bin_size = len;
		sprintf(str, "<%d>", bin_size);
		MYSERIAL0.println(str);

		usart_send_buf(huart1, (uint8_t *)str, strlen(str));
		read_dev_answer();

		start_rev_bin = 1;

		memset(recv_buffer, 0, 500);
		break;
	// receive bin
	case REV_BIN:
		if (start_rev_bin)
		{
			// wait one fps data ok
			while (!rev_one_fps_dat_flag);

			memset(str, 0, 30);
			sprintf(str, "<rev:%d>", rev_bin_byte_num);
			MYSERIAL0.println(str);

			build_bin_pack();

			memset(recv_buffer, 0, 500);
			rev_bin_byte_num = 0;
		}
		break;
	}
}

char DexarmRotation::recv_bin(char c)
{
	if (start_rev_bin)
	{
		recv_buffer[rev_bin_byte_num++] = c;
		// start
		if (c == '<')
		{
			rev_bin_byte_num = 0;
		}
		// end
		if (c == '>')
		{
			rev_one_fps_dat_flag = 1;
		}

		if (rev_bin_byte_num > 500)
		{
			rev_bin_byte_num = 0;
		}
	}
	return c;
}

void DexarmRotation::report_pos(void) {
	int positon = dexarm_rotation.read_pos();
	SERIAL_ECHOLNPAIR("current positon = ", positon);
}

void DexarmRotation::loop(void) {
	if (loop_time > millis()) {
		return;
	}
	loop_time = millis() + 200;
	if (is_move) {
		int positon = dexarm_rotation.read_pos();
  	positon = (positon * 100) / 284;
		if (last_positon == positon) {
			is_move = false;
			SERIAL_ECHOLNPAIR("Movement: Completed");
			report_pos();
		}
		last_positon = positon;
	}
}
