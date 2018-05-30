/**
 * Calibrate
 * 
 * Connects to a compatible MiCS-6814 sensor through I2C
 * and performs a calibration of the sensor.
 * 
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
 */

#include <MiCS6814-I2C.h>

MiCS6814 sensor;

// Minutes to preheat
uint8_t preheatTime = 10;

void setup() {
  // Initialize serial connection
  Serial.begin(115200);

  // Connect to sensor using default I2C address (0x04)
  // Alternatively the address can be passed to begin(addr)
  bool sensorConnected = sensor.begin();
  
  // Check if the sensor is connected
  if (sensorConnected == true) {
    // Print status message
    Serial.println("Connected to MiCS-6814 sensor");

    // Activate heater element
    sensor.powerOn();
    
    // Print preheat timer
    Serial.print("Preheating sensor ... ");

    // Wait the required amount of time, printing
    // status every minute
    while (preheatTime > 0) {
        Serial.print(preheatTime);
        Serial.print(" ... ");
        
        // Wait a minute
        delay(60000);

        preheatTime = preheatTime - 1;
    }
    Serial.println("done!");

    // Print "old" base resistance
    Serial.println("Current base resistances:");
    Serial.print("NH3: ");
    Serial.print(sensor.getBaseResistance(CH_NH3));
    Serial.print(" ; RED: ");
    Serial.print(sensor.getBaseResistance(CH_RED));
    Serial.print(" ; OX: ");
    Serial.println(sensor.getBaseResistance(CH_OX));
    

    Serial.println("---");
    Serial.println("Starting calibration. This might take a while.");

    // Perform calibration
    sensor.calibrate();
    
    // Print new base resistances stored in the sensor
    Serial.println("Calibration done");
    Serial.println("New base resistances:");
    Serial.print("NH3: ");
    Serial.print(sensor.getBaseResistance(CH_NH3));
    Serial.print("\tRED: ");
    Serial.print(sensor.getBaseResistance(CH_RED));
    Serial.print("\tOX: ");
    Serial.println(sensor.getBaseResistance(CH_OX));
  } else {
    // Print error message on connection failure
    Serial.println("Couldn't connect to MiCS-6814 sensor");
  }
}

void loop() {
  // Nothing to do here
}
