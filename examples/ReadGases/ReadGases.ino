/**
 * ReadGases
 * 
 * Continuously reads the gas measurements in ppm from
 * the attached MiCS-6814 sensor through I2C.
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
bool sensorConnected;

void setup() {
  // Initialize serial connection
  Serial.begin(115200);

  // Connect to sensor using default I2C address (0x04)
  // Alternatively the address can be passed to begin(addr)
  sensorConnected = sensor.begin();

  if (sensorConnected == true) {
    // Print status message
    Serial.println("Connected to MiCS-6814 sensor");

    // Turn heater element on
    sensor.powerOn();
    
    // Print header for live values
    Serial.println("Current concentrations:");
    Serial.println("CO\tNO2\tNH3\tC3H8\tC4H10\tCH4\tH2\tC2H5OH");
  } else {
    // Print error message on failed connection
    Serial.println("Couldn't connect to MiCS-6814 sensor");
  }
}

void loop() {
  if (sensorConnected) {
    // Print live values
    Serial.print(sensor.measureCO());
    Serial.print("\t");
    Serial.print(sensor.measureNO2());
    Serial.print("\t");
    Serial.print(sensor.measureNH3());
    Serial.print("\t");
    Serial.print(sensor.measureC3H8());
    Serial.print("\t");
    Serial.print(sensor.measureC4H10());
    Serial.print("\t");
    Serial.print(sensor.measureCH4());
    Serial.print("\t");
    Serial.print(sensor.measureH2());
    Serial.print("\t");
    Serial.println(sensor.measureC2H5OH());
  }

  // Wait a small amount of time
  delay(1000);
}

