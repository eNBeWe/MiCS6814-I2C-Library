/*
 * MIT License
 * 
 * Copyright (c) 2018 Nis Wechselberg
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Part of the library is based on works by Seeed Technology Inc., published under MIT license:
 *
 * Copyright (c) 2015 Seeed Technology Inc.  All right reserved.
 * Version 1 - Author: Jacky Zhang, 2015-03-17, http://www.seeed.cc/
 *             Modified by: Jack, 2015-08
 * Version 2 - Author: Loovee, 2016-11-11
 */

#ifndef MICS6814_I2C_H
#define MICS6814_I2C_H

#include <math.h>
#include <Arduino.h>
#include <Wire.h>

/*
 * I2C commands
 * 
 * Taken from Grove Multichannel Gas Sensor Library, just some renaming done here
 */
#define CMD_GET_NH3             0x01      // Retrieve current resistance for NH3 sensor channel
#define CMD_GET_RED             0x02      // Retrieve current resistance for RED sensor channel
#define CMD_GET_OX              0x03      // Retrieve current resistance for OX sensor channel
#define CMD_V2_GET_ALL          0x04      // Retrieve all sensor resistance channels

#define CMD_V1_GET_R0_NH3       0x11      // Returns 4 bytes, 0: 0x11, 1: MSB, 2: LSB, 3: Checksum (0+1+2) 
#define CMD_V1_GET_R0_RED       0x12      // Returns 4 bytes, 0: 0x12, 1: MSB, 2: LSB, 3: Checksum (0+1+2)
#define CMD_V1_GET_R0_OX        0x13      // Returns 4 bytes, 0: 0x13, 1: MSB, 2: LSB, 3: Checksum (0+1+2)
#define CMD_V2_GET_R0           0x08      // Retrieve current R0 values
#define CMD_V2_GET_R0_DEFAULT   0x09      // Retrieve default R0 values

#define CMD_V1_CALIBRATE        0x22      // Automatically calibrates the sensor
#define CMD_V2_SET_R0           0x07      // Set custom R0 values

#define CMD_V1_CHANGE_I2C       0x23      // Change I2C address of the sensor (V1)
#define CMD_V2_CHANGE_I2C       0x05      // Change I2C address of the sensor (V2)

#define CMD_V2_READ_EEPROM      0x06      // Read stored uint16_t data from EEPROM (MSB is transmitted first)

#define CMD_V2_CONTROL_LED      0x0A      // Control status LED (no LED on my board, but Grove supports is, so ... meh!)

#define CMD_V2_CONTROL_PWR      0x0B      // Heater control (V2) - Takes an additional byte to turn on or off
#define CMD_V1_PWR_OFF          0x20      // Turns off sensor heater (V1)
#define CMD_V1_PWR_ON           0x21      // Turns on sensor heater (V1)


/* EEPROM Addresses
 * 
 * Taken from V2 Seeed library. 
 * V1 used different mappings but didn't offer a direct EEPROM read access
 * Only used in version identification and EEPROM output
 */
#define EEPROM_VERSION_ID       0x00      // Identifier for the current version, Grove uses value 1126 to identify version 2 of their sensor.
#define EEPROM_R0_DEFAULT_NH3   0x02      // Default calibration base value for NH3 sensor
#define EEPROM_R0_DEFAULT_RED   0x04      // Default calibration base value for RED sensor
#define EEPROM_R0_DEFAULT_OX    0x06      // Default calibration base value for OX sensor
#define EEPROM_R0_NH3           0x08      // User calibration base value for NH3 sensor
#define EEPROM_R0_RED           0x0A      // User calibration base value for RED sensor
#define EEPROM_R0_OX            0x0C      // User calibration base value for OX sensor
#define EEPROM_I2C_ADDR         0x14      // Configured I2C address

// Default values 
#define DATA_I2C_ADDR           0x04      // Default I2C address
#define DATA_VERSION_ID_V2      1126

// Enum for the sensor channels
enum channel {
  CH_NH3, CH_RED, CH_OX
};
typedef enum channel channel_t;

// Enum for proper gas declaration
enum gas {
  CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH
};
typedef enum gas gas_t;

class MiCS6814 {

public:
  uint8_t begin();
  uint8_t begin(uint8_t address);

  uint8_t getVersion();

  void changeI2CAddr(uint8_t newAddr);

  void calibrate();

  void powerOn();
  void powerOff();
  void ledOn();
  void ledOff();

  // Low level value access, unit: analog voltage 0..1024
  uint16_t getResistance(channel_t channel);
  uint16_t getResistanceNH3() {
    return getResistance(CH_NH3);
  }
  uint16_t getResistanceRED() {
    return getResistance(CH_RED);
  }
  uint16_t getResistanceOX() {
    return getResistance(CH_OX);
  }

  uint16_t getBaseResistance(channel_t channel);
  uint16_t getBaseResistanceNH3() {
    return getBaseResistance(CH_NH3);
  }
  uint16_t getBaseResistanceRED() {
    return getBaseResistance(CH_RED);
  }
  uint16_t getBaseResistanceOX() {
    return getBaseResistance(CH_OX);
  }


  //High level gas concentration, unit: ppm
  float measure(gas_t gas);
  float measureCO() {
    return measure(CO);
  }
  float measureNO2() {
    return measure(NO2);
  }
  float measureNH3() {
    return measure(NH3);
  }
  float measureC3H8() {
    return measure(C3H8);
  }
  float measureC4H10() {
    return measure(C4H10);
  }
  float measureCH4() {
    return measure(CH4);
  }
  float measureH2() {
    return measure(H2);
  }
  float measureC2H5OH() {
    return measure(C2H5OH);
  }

  // Show the (known) data in the EEPROM
  void display_eeprom();

private:
  uint8_t __version;
  uint8_t __i2CAddress;

  uint16_t getEEPROMData(uint8_t eeprom_address);
  uint16_t getRuntimeData(uint8_t cmd, uint8_t responseLength, uint8_t responseOffset);
  float getCurrentRatio(channel_t channel);
};


#endif
