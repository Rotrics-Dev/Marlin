/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#ifndef TARGET_STM32F4
  #error "Oops! Select an STM32F4 board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "DexArm V3_1" // redefined?
#define HARDWARE_VERSION "Hardware V3.1\r\n"
#define FIRMWARE_VERSION "Firmware V2.2.5\r\n"

// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION   // Use BackSRAM-based EEPROM emulation
#define FLASH_EEPROM_EMULATION  // Use Flash-based EEPROM emulation

#define LED_PIN            PB14
#define UART1_TX_PIN       PA9
#define UART1_RX_PIN       PA10
#define UART3_TX_PIN       PD8
#define UART3_RX_PIN       PD9
#define LASER_PWM_PIN      PD14
#define MS1_PIN            PB9
#define MS2_PIN            PB8

// Use sensor MT6701 or AS5600, default AS5600
// #define USING_MT6701_POSITION_SENSOR

//Position Sensor
// DIGIPOT slave addresses
#ifndef POSITION_SENSOR_I2C_ADDRESS

  #if defined(USING_MT6701_POSITION_SENSOR)
    #define POSITION_SENSOR_I2C_ADDRESS 0x06              // MT6701
  #else
    #define POSITION_SENSOR_I2C_ADDRESS 0x36              // AS5600
  #endif

#endif
#define POSITION_SENSOR_I2C_SCK_X PF4
#define POSITION_SENSOR_I2C_SDA_X PF5
#define POSITION_SENSOR_I2C_SCK_Y PF6
#define POSITION_SENSOR_I2C_SDA_Y PF7
#define POSITION_SENSOR_I2C_SCK_Z PF8
#define POSITION_SENSOR_I2C_SDA_Z PF9

//
// Limit Switches
//
#define X_MIN_PIN          PB10
//#define X_MAX_PIN        
#define Y_MIN_PIN          PE12
//#define Y_MAX_PIN        
#define Z_MIN_PIN          PG8
#define Z_MAX_PIN          PG8
#define E0_MIN_PIN         PE15

//
// Steppers
//
#define X_STEP_PIN         PE9
#define X_DIR_PIN          PF1
#define X_ENABLE_PIN       PF2
#ifndef X_CS_PIN
  #define X_CS_PIN         PA15
#endif

#define Y_STEP_PIN         PE11
#define Y_DIR_PIN          PE8
#define Y_ENABLE_PIN       PD7
 #ifndef Y_CS_PIN
  #define Y_CS_PIN         PF8
#endif

#define Z_STEP_PIN         PE13
#define Z_DIR_PIN          PC2
#define Z_ENABLE_PIN       PC0
#ifndef Z_CS_PIN
  #define Z_CS_PIN         PF10
#endif

#define E0_STEP_PIN        PE14
#define E0_DIR_PIN         PA0
#define E0_ENABLE_PIN      PC3
#ifndef E0_CS_PIN
  #define E0_CS_PIN        PB3
#endif

#define E1_STEP_PIN        PD15
#define E1_DIR_PIN         PE7
#define E1_ENABLE_PIN      PA3
#ifndef E1_CS_PIN
  #define E1_CS_PIN        PG15
#endif

#define E2_STEP_PIN        PD13
#define E2_DIR_PIN         PG9
#define E2_ENABLE_PIN      PF0
#ifndef E2_CS_PIN
  #define E2_CS_PIN        PG12
#endif

#if HAS_TMC220x
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  //
  // Software serial
  //
  #define X_SERIAL_TX_PIN  PE4
  #define X_SERIAL_RX_PIN  PC13

  #define Y_SERIAL_TX_PIN  PE2
  #define Y_SERIAL_RX_PIN  PE3

  #define Z_SERIAL_TX_PIN  PE0
  #define Z_SERIAL_RX_PIN  PE1

  #define E0_SERIAL_TX_PIN PD2
  #define E0_SERIAL_RX_PIN PD4

  #define E1_SERIAL_TX_PIN PD1
  #define E1_SERIAL_RX_PIN PD1

  #define E2_SERIAL_TX_PIN PD6
  #define E2_SERIAL_RX_PIN PD6

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE 19200
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN         PF3  // T1 <-> E0

//
// Heaters / Fans
//
#define HEATER_0_PIN       PB0  // Heater0
#define FAN_PIN            PD14 // Fan0