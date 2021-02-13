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

#include "MiCS6814-I2C.h"

/**
 * Starts the connection to the gas sensor,
 * using the default I2C address.
 * @return {@code 1} if the connection could be
 *         established, {@code 0} otherwise
 */
uint8_t MiCS6814::begin() {
  return begin(DATA_I2C_ADDR);
}


/**
 * Starts the connection to the gas sensor,
 * using the given address.
 * @param address
 *        The I2C address to use for the connection.
 * @return {@code 1} if the connection could be
 *         established, {@code 0} otherwise
 */
uint8_t MiCS6814::begin(uint8_t address) {
  // Store the address for later use
  __i2CAddress = address;

  // Start the Wire library
  Wire.begin();

  // Make sure there is a device at the other end of the wire
  Wire.beginTransmission(__i2CAddress);
  if (Wire.endTransmission() != 0) {
    // There is nobody at the other end that wants to talk to us.
    return 0;
  }

  uint16_t versionData = getEEPROMData(EEPROM_VERSION_ID);
  // Version 2 is identified by the unique data
  if (DATA_VERSION_ID_V2 == versionData) {
    __version = 2;
  } else {
    // Version 1 sensors don't reply to the given request at startup.
    // However, if the sensor has already been running some time,
    // we might receive any old measurement point (0 ... 1024)
    __version = 1;
  }

  return true;
}


/**
 * Returns the (probable) version of the connected sensor.
 * @return The version of the sensor.
 */
uint8_t MiCS6814::getVersion() {
  return __version;
}


/**
 * Sets a new I2C address in the slave EEPROM.
 */
void MiCS6814::changeI2CAddr(uint8_t newAddr) {
  // Start the transmission to the old address
  Wire.beginTransmission(__i2CAddress);
  // The used command differs between V1 an V2
  if (1 == __version) {
    Wire.write(CMD_V1_CHANGE_I2C);
  }
  if (2 == __version) {
    Wire.write(CMD_V2_CHANGE_I2C);
  }
  // Send new address
  Wire.write(newAddr);
  Wire.endTransmission();

  // Store the new address for later use
  __i2CAddress = newAddr;
}


/**
 * Starts a calibration of the sensor. The behaviour of this
 * method differs, depending on the sensor firmware version.
 * Version 1: The calibration is done in the sensor firmware.
 *            This method returns (almost) immediately,
 *            but reported sensor values will be stale for a few
 *            moments, until calibration is done.
 * Version 2: The calibration is done in this code.
 *            The method blocks until the calibration is done
 */
void MiCS6814::calibrate() {
  if (1 == __version) {
    // Start the "onboard" calibration
    Wire.beginTransmission(__i2CAddress);
    Wire.write(CMD_V1_CALIBRATE);
    Wire.endTransmission();

    return;
  }
  if (2 == __version) {
    // Continuously measure the resistance,
    // storing the last N measurements in a circular buffer.
    // Calculate the floating average of the last seconds.
    // If the current measurement is close to the average stop.

    // Seconds to keep stable for successful calibration
    // (Keeps smaller than 64 to prevent overflows)
    uint8_t seconds = 30;
    // Allowed delta for the average from the current value
    uint8_t delta = 2;

    // Circular buffer for the measurements
    uint16_t bufferNH3[seconds];
    uint16_t bufferRED[seconds];
    uint16_t bufferOX[seconds];
    // Pointers for the next element in the buffer
    uint8_t pntrNH3 = 0;
    uint8_t pntrRED = 0;
    uint8_t pntrOX = 0;
    // Current floating sum in the buffer
    uint16_t fltSumNH3 = 0;
    uint16_t fltSumRED = 0;
    uint16_t fltSumOX = 0;

    // Current measurements;
    uint16_t curNH3;
    uint16_t curRED;
    uint16_t curOX;

    // Flag to see if the channels are stable
    bool NH3stable = false;
    bool REDstable = false;
    bool OXstable = false;

    // Initialize buffer
    for (int i = 0; i < seconds; ++i) {
      bufferNH3[i] = 0;
      bufferRED[i] = 0;
      bufferOX[i] = 0;
    }

    do {
      // Wait a second
      delay(1000);

      // Read new resistances
      curNH3 = getResistanceNH3();
      curRED = getResistanceRED();
      curOX = getResistanceOX();

      // Update floating sum by subtracting value
      // about to be overwritten and adding the new value.
      fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
      fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
      fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

      // Store new measurement in buffer
      bufferNH3[pntrNH3] = curNH3;
      bufferRED[pntrRED] = curRED;
      bufferOX[pntrOX] = curOX;

      // Determine new state of flags
      NH3stable = abs(fltSumNH3 / seconds - curNH3) < delta;
      REDstable = abs(fltSumRED / seconds - curRED) < delta;
      OXstable = abs(fltSumOX / seconds - curOX) < delta;

      // Advance buffer pointer
      pntrNH3 = (pntrNH3 + 1) % seconds ;
      pntrRED = (pntrRED + 1) % seconds;
      pntrOX = (pntrOX + 1) % seconds;
    } while (!NH3stable || !REDstable || !OXstable);

    // Store new base resistance values in EEPROM
    Wire.beginTransmission(__i2CAddress);
    Wire.write(CMD_V2_SET_R0);
    Wire.write((fltSumNH3 / seconds) >> 8);
    Wire.write((fltSumNH3 / seconds) & 0xFF);
    Wire.write((fltSumRED / seconds) >> 8);
    Wire.write((fltSumRED / seconds) & 0xFF);
    Wire.write((fltSumOX / seconds) >> 8);
    Wire.write((fltSumOX / seconds) & 0xFF);
    Wire.endTransmission();
  }
}


/**
 * Activates the heater element in the sensor.
 * For sensors with firmware version 1,
 * the LED is also activated.
 */
void MiCS6814::powerOn() {
  // Send the data to the known I2C address
  Wire.beginTransmission(__i2CAddress);
  if (1 == __version) {
    // Version 1 has a dedicated command to turn the heater on
    Wire.write(CMD_V1_PWR_ON);
  }
  if (2 == __version) {
    // Version 2 has a control command with a second byte for the new state.
    Wire.write(CMD_V2_CONTROL_PWR);
    Wire.write(1);
  }
  Wire.endTransmission();
}


/**
 * Deactivates the heater element in the sensor.
 * For sensors with firmware version 1,
 * the LED is also deactivated.
 */
void MiCS6814::powerOff(){
  // Send the data to the known I2C address
  Wire.beginTransmission(__i2CAddress);
  if (1 == __version) {
    // Version 1 has a dedicated command to turn the heater off
    Wire.write(CMD_V1_PWR_OFF);
  }
  if (2 == __version) {
    // Version 2 has a control command with a second byte for the new state.
    Wire.write(CMD_V2_CONTROL_PWR);
    Wire.write(0);
  }
  Wire.endTransmission();
}


/**
 * Activates the LED on the sensor board.
 * This is only supported by firmware version 2.
 */
void MiCS6814::ledOn() {
  if (1 == __version) {
    return;
  }

  Wire.beginTransmission(__i2CAddress);
  Wire.write(CMD_V2_CONTROL_LED);
  Wire.write(1);
  Wire.endTransmission();
}


/**
 * Deactivates the LED on the sensor board.
 * This is only supported by firmware version 2.
 */
void MiCS6814::ledOff(){
  if (1 == __version) {
    return;
  }

  Wire.beginTransmission(__i2CAddress);
  Wire.write(CMD_V2_CONTROL_LED);
  Wire.write(0);
  Wire.endTransmission();

}

/**
 * Requests the current resistance for a given channel
 * from the sensor. The value is an ADC value between
 * 0 and 1024.
 *
 * @param channel
 *        The channel to read the base resistance from.
 * @return The unsigned 16-bit base resistance 
 *         of the selected channel.
 */
uint16_t MiCS6814::getResistance(channel_t channel) {
  // The versions use the same command but different response formats.
  if (1 == __version) {
    switch (channel) {
      case CH_NH3:
        return getRuntimeData(CMD_GET_NH3, 4, 1);
      case CH_RED:
        return getRuntimeData(CMD_GET_RED, 4, 1);
      case CH_OX:
        return getRuntimeData(CMD_GET_OX, 4, 1);
    }
  }
  if (2 == __version) {
    switch (channel) {
      case CH_NH3:
        return getRuntimeData(CMD_GET_NH3, 2, 0);
      case CH_RED:
        return getRuntimeData(CMD_GET_RED, 2, 0);
      case CH_OX:
        return getRuntimeData(CMD_GET_OX, 2, 0);
    }
  }
  
  return 0;
}

/**
 * Requests the base resistance for a given channel
 * from the sensor. The value is an ADC value between
 * 0 and 1024.
 *
 * @param channel
 *        The channel to read the base resistance from.
 * @return The unsigned 16-bit base resistance 
 *         of the selected channel.
 */
uint16_t MiCS6814::getBaseResistance(channel_t channel) {
  if (1 == __version) {
    // Version 1 can query every channel independently
    // Reply is 4 bytes long with relevant data in second and third byte
    switch (channel) {
      case CH_NH3:
        return getRuntimeData(CMD_V1_GET_R0_NH3, 4, 1);
      case CH_RED:
        return getRuntimeData(CMD_V1_GET_R0_RED, 4, 1);
      case CH_OX:
        return getRuntimeData(CMD_V1_GET_R0_OX, 4, 1);
    }
  }
  if (2 == __version) {
    // Version 2 uses the same command every time, but different offsets
    switch (channel) {
      case CH_NH3:
        return getRuntimeData(CMD_V2_GET_R0, 6, 0);
      case CH_RED:
        return getRuntimeData(CMD_V2_GET_R0, 6, 2);
      case CH_OX:
        return getRuntimeData(CMD_V2_GET_R0, 6, 4);
    }
  }

  return 0;
}


/**
 * Calculates the current resistance ratio for the given channel.
 *
 * @param channel
 *        The channel to request resistance values from.
 * @return The floating-point resistance ratio for the given channel.
 */
float MiCS6814::getCurrentRatio(channel_t channel) {
  float baseResistance = (float) getBaseResistance(channel);
  float resistance = (float) getResistance(channel);

  if (1 == __version) {
    return resistance / baseResistance;
  }
  if (2 == __version) {
    return resistance / baseResistance * (1023.0 - baseResistance) / (1023.0 - resistance);
  }

  return -1.0;
}


/**
 * Measures the gas concentration in ppm for the specified gas.
 *
 * @param gas
 *        The gas to calculate the concentration for.
 * @return The current concentration of the gas 
 *         in parts per million (ppm).
 */
float MiCS6814::measure(gas_t gas) {
  float ratio;
  float c = 0;

  switch (gas) {
    case CO: //carbon monoxide
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.177) * 4.4638; //paulvha
      break;
    case NO2: //nitrogen dioxide
      ratio = getCurrentRatio(CH_OX);
      c = pow(ratio, 0.9979) * 0.1516; //paulvha
      break;
    case NH3: //ammonia
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -1.903) * 0.6151; //paulvha
      break;
    case C3H8: //propane
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -2.492) * 569.56; //paulvha
      break;
    case C4H10: //iso-butane
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -1.888) * 503.2; //paulvha
      break;
    case CH4: //methane
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -4.093) * 837.38; //paulvha
      break;
    case H2: //hydrogen
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.781) * 0.828; //paulvha
      break;
    case C2H5OH: //ethanol
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.58) * 1.363; //paulvha
      break;
  }

  return isnan(c) ? -1 : c;
}


/**
 * Print the (known) data in the EEPROM to Serial.
 * This is only supported by firmware version 2.
 */
void MiCS6814::display_eeprom() {
  if (1 == __version) {
    return;
  }

  // Print the data to the serial console by reading
  // it through the 
  Serial.print("VERSION_ID = ");      Serial.println(getEEPROMData(EEPROM_VERSION_ID));
  Serial.print("R0_DEFAULT_NH3 = ");  Serial.println(getEEPROMData(EEPROM_R0_DEFAULT_NH3));
  Serial.print("R0_DEFAULT_RED = ");  Serial.println(getEEPROMData(EEPROM_R0_DEFAULT_RED));
  Serial.print("R0_DEFAULT_OX = ");   Serial.println(getEEPROMData(EEPROM_R0_DEFAULT_OX));
  Serial.print("R0_NH3 = ");          Serial.println(getEEPROMData(EEPROM_R0_NH3));
  Serial.print("R0_RED = ");          Serial.println(getEEPROMData(EEPROM_R0_RED));
  Serial.print("R0_OX = ");           Serial.println(getEEPROMData(EEPROM_R0_OX));
  Serial.print("I2C_ADDR = ");        Serial.println(getEEPROMData(EEPROM_I2C_ADDR));
}


/**
 * Reads a single unsigned 16-bit value from the slave EEPROM.
 *
 * This is not properly supported with a V1 sensor. At startup
 * we won't be getting any data from the sensor, later during
 * runtime we will be getting random data.
 * 
 * @param eeprom_address
 *        The address in the slave eeprom to read.
 * @return The unsigned 16-bit value received from the slave.
 */
uint16_t MiCS6814::getEEPROMData(uint8_t eeprom_address) {
  // Send a request for EEPROM data
  Wire.beginTransmission(__i2CAddress);
  Wire.write(CMD_V2_READ_EEPROM);
  Wire.write(eeprom_address);
  Wire.endTransmission();

  // Use a very short delay here, because the Wire library had
  // a small problem when talking i.e. between multiple ATmega328
  // or with NodeMCU.
  delayMicroseconds(50);

  // Request two bytes of data from the slave.
  uint8_t received = Wire.requestFrom(__i2CAddress,(uint8_t) 2);

  // Make sure we received the right amount of data
  if (2 == received) {
    // The slave sends the MSB first, the LSB afterwards
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();

    // Combine and return the received data
    return (msb << 8) | lsb;
  }

  // If we received no data just return 0 (i.e. when connected to V1 sensor).
  return 0;
}


/**
 * Requests an unsigned 16-bit data point from the sensor.
 * @param cmd
 *        The command to request from the sensor.
 * @param responseLength
 *        The length of the response from the slave.
 * @param responseOffset
 *        The first byte to use from the response.
 * @return The unsigned 16-bit value returned from the sensor.
 */
uint16_t MiCS6814::getRuntimeData(uint8_t cmd, uint8_t responseLength, uint8_t responseOffset) {
  Wire.beginTransmission(__i2CAddress);
  Wire.write(cmd);
  Wire.endTransmission();

  // Use a very short delay here, because the Wire library had
  // a small problem when talking i.e. between multiple ATmega328
  // or with NodeMCU.
  delayMicroseconds(50);

  // Request the given number of bytes from the slave.
  uint8_t received = Wire.requestFrom(__i2CAddress, responseLength);

  // Make sure we received the right amount of data
  if (received == responseLength) {
    // Ignore the first `reponseOffset` bytes
    for (uint8_t i = 0; i < responseOffset; ++i) {
      Wire.read();
    }

    // The slave sends the MSB first, the LSB afterwards
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    // Ignore stuff after our data
    // The Wire library cleans up anyway

    // Combine and return the received data
    return (msb << 8) | lsb;
  }

  return 0;
}
