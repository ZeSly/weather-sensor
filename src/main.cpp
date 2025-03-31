// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee temperature sensor.
 *
 * The example demonstrates how to use Zigbee library to create a end device temperature sensor.
 * The temperature sensor is a Zigbee end device, which is controlled by a Zigbee coordinator.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

#include <Adafruit_BME280.h>

/* Zigbee flow + pressure sensor configuration */
#define FLOW_SENSOR_ENDPOINT_NUMBER 10
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 11

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 15           /* Sleep for 55s will + 5s delay for establishing connection => data reported every 1 minute */

uint8_t button = BOOT_PIN;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(FLOW_SENSOR_ENDPOINT_NUMBER);
ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);

Adafruit_BME280 bme;

uint8_t getBatteryCapacity();

void readSensors()
{ // delaying for 100ms x 20 = 2s
  float temperature_value = bme.readTemperature();
  float humidity_value = bme.readHumidity();
  float pressure_value = bme.readPressure() / 100.0;
  uint8_t battery = getBatteryCapacity();

  Serial.printf("Temperature %.2f °C\r\n", temperature_value);
  zbTempSensor.setTemperature(temperature_value);

  Serial.printf("Humidity    %.2f %%\r\n", humidity_value);
  zbTempSensor.setHumidity(humidity_value);

  Serial.printf("Pressure    %.1f hPa\r\n", pressure_value);
  zbPressureSensor.setPressure(pressure_value);

  Serial.printf("Battery     %u %%\r\n", battery);
  zbTempSensor.setBatteryPercentage(battery);

  zbTempSensor.reportBatteryPercentage();
  zbTempSensor.report();
  zbPressureSensor.report();
  Serial.println();
}

void CheckButton()
{
  // Checking button for factory reset and reporting
  if (digitalRead(button) == LOW)
  { // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW)
    {
      delay(50);
      if ((millis() - startTime) > 3000)
      {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
  }
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000); //Take some time to open up the Serial Monitor

  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Init battery ADC
  pinMode(A0, INPUT);

  if (!bme.begin(0x76))
  {
    Serial.println("BME280 not found !");
  }

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("ZeSly", "WeatherSensor");

  // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(-40, 60);

  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(1);

  // Set power source to battery and set battery percentage to measured value
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) anytime
  uint8_t battery = getBatteryCapacity();
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, battery);

  // Set minimum and maximum pressure measurement value in hPa
  zbPressureSensor.setMinMaxValue(0, 10000);

  // Optional: Set tolerance for pressure measurement in hPa
  zbPressureSensor.setTolerance(1);

  // Add humidity cluster to the temperature sensor device with min, max and tolerance values
  zbTempSensor.addHumiditySensor(0, 100, 1);

  // Add endpoints to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);
  Zigbee.addEndpoint(&zbPressureSensor);

  Serial.println("Starting Zigbee...");
  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false))
  {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    delay(1000);
    ESP.restart();
  }
  else
  {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected())
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.println("Successfully connected to Zigbee network");

  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);

  CheckButton();
  readSensors();
  delay(1000);

  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();

  // Set reporting interval for flow and pressure measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (pressure change in hPa, flow change in 0,1 m3/h)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of delta change
  // zbTempSensor.setReporting(0, 30, 1.0);
  // zbPressureSensor.setReporting(0, 30, 1);
}

void loop()
{
  // This is not going to be called
}
