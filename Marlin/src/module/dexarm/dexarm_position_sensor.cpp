#include "../../inc/MarlinConfig.h"
#include "../../core/language.h"

#include <SlowSoftI2CMaster.h>  //https://github.com/stawel/SlowSoftI2CMaster

#define POSITION_SENSOR_NUM 3

const uint8_t sda_pins[POSITION_SENSOR_NUM] = {
  POSITION_SENSOR_I2C_SDA_X
    , POSITION_SENSOR_I2C_SDA_Y
      , POSITION_SENSOR_I2C_SDA_Z
};

const uint8_t sck_pins[POSITION_SENSOR_NUM] = {
  POSITION_SENSOR_I2C_SCK_X
    , POSITION_SENSOR_I2C_SCK_Y
      , POSITION_SENSOR_I2C_SCK_Z
};

static SlowSoftI2CMaster pots[POSITION_SENSOR_NUM] = {
  SlowSoftI2CMaster { sda_pins[X_AXIS], sck_pins[X_AXIS] }
    , SlowSoftI2CMaster { sda_pins[Y_AXIS], sck_pins[Y_AXIS] }
      , SlowSoftI2CMaster { sda_pins[Z_AXIS], sck_pins[Z_AXIS] }
};

static void position_sensor_error(const uint8_t position_sensor_state) {

  SERIAL_ERROR_START();
  serialprintPGM(PSTR(STR_POSITION_SENSOR_READ_ERROR));
  SERIAL_ECHOPGM(STR_POSITION_SENSOR_AXIS);

  if((position_sensor_state&(0x01<<0)) > 0)
    SERIAL_ECHO("A ");
  if((position_sensor_state&(0x01<<1)) > 0)
    SERIAL_ECHO("B ");
  if((position_sensor_state&(0x01<<2)) > 0)
    SERIAL_ECHO("C ");

  SERIAL_EOL();
}

void check_position_sensor(const int sensor_value[]) {
  uint8_t position_sensor_state = 0;
  for(int i=0; i<3; i++){
    if(sensor_value[i] > 4096){
      position_sensor_state |= 0x01<<i;
    }
  }

  if(position_sensor_state > 0){
    position_sensor_error(position_sensor_state);
    SERIAL_ECHO("Position Sensor A:");
	  SERIAL_ECHO(sensor_value[0]);

	  SERIAL_ECHO(" B: ");
	  SERIAL_ECHO(sensor_value[1]);

	  SERIAL_ECHO(" C: ");
	  SERIAL_ECHO(sensor_value[2]);
    SERIAL_EOL();
  }
}

static uint8_t i2c_read(const uint8_t channel, bool last, const uint8_t reg_address) {
  uint8_t value = 0;
  if (WITHIN(channel, 0, POSITION_SENSOR_NUM - 1)) {
    pots[channel].i2c_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_WRITE);
    pots[channel].i2c_write(reg_address);
    pots[channel].i2c_rep_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_READ);
    value = pots[channel].i2c_read(last);
    pots[channel].i2c_stop();
  }
  return value;
}

static void i2c_send(const uint8_t channel, const byte v) {
  if (WITHIN(channel, 0, DIGIPOT_I2C_NUM_CHANNELS - 1)) {
    pots[channel].i2c_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_WRITE);
    pots[channel].i2c_write(v);
    pots[channel].i2c_stop();
  }
}

word position_sensor_value_read(const uint8_t channel) {
#if 1
	word value_hight = i2c_read(channel, false, 0x03);
	word value_low = i2c_read(channel, true, 0x04); 
	word value = (((value_hight&0x00ff)<<8)|(value_low&0x00fc))>>2;
	// value = 4096.0 - value * 4096.0 / 16384.0;
	// value = 4096 - value * 4096 / 16384;
	value = (16384 - value)/4;
#else
	word value = i2c_read(channel, false, 0x0e);
	value <<= 8;
	value |= i2c_read(channel, true, 0x0f); 
#endif
	return value;
}

void position_sensor_init() {
  LOOP_L_N(i, POSITION_SENSOR_NUM)
    pots[i].i2c_init();
}