// Copyright Sylvain Girard 2025
// www.zesly.net
//
// This software is a computer program whose purpose is to sedn temperature,
// humidity and pressure measurment to a zigbee network.
//
// This software is governed by the CeCILL-B license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-B
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-B license and that you accept its terms.
//

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

#include <Adafruit_BME280.h>

/* Zigbee flow + pressure sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 11

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 58          /* Sleep for 58 will + 2s delay for establishing connection => data reported every 1 minute */

uint8_t button = BOOT_PIN;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);

Adafruit_BME280 bme;

uint8_t getBatteryCapacity();

struct SensorsValues
{
  float temperature;
  float humidity;
  float pressure;
  uint8_t battery;
};

RTC_DATA_ATTR SensorsValues LatestSensorsValues{-500.0, -1.0, 0.0, 0};

void SendSensorsValues(SensorsValues &sensorsValues)
{
  bool report = false;

  if (LatestSensorsValues.temperature != sensorsValues.temperature ||
      LatestSensorsValues.humidity != sensorsValues.humidity)
  {
    Serial.printf("Temperature %.2f °C\r\n", sensorsValues.temperature);
    zbTempSensor.setTemperature(sensorsValues.temperature);
    LatestSensorsValues.temperature = sensorsValues.temperature;

    Serial.printf("Humidity    %.2f %%\r\n", sensorsValues.humidity);
    zbTempSensor.setHumidity(sensorsValues.humidity);
    LatestSensorsValues.humidity = sensorsValues.humidity;

    zbTempSensor.report();
    report = true;
  }

  if (LatestSensorsValues.pressure != sensorsValues.pressure)
  {
    Serial.printf("Pressure    %.1f hPa\r\n", sensorsValues.pressure);
    zbPressureSensor.setPressure(sensorsValues.pressure);
    zbPressureSensor.report();
    LatestSensorsValues.pressure = sensorsValues.pressure;
    report = true;
  }

  if (LatestSensorsValues.battery != sensorsValues.battery)
  {
    Serial.printf("Battery     %u %%\r\n", sensorsValues.battery);
    zbTempSensor.setBatteryPercentage(sensorsValues.battery);
    zbTempSensor.reportBatteryPercentage();
    LatestSensorsValues.battery = sensorsValues.battery;
    report = true;
  }

  if (!report)
  {
    Serial.print("Nothing to report");
  }
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

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Init battery ADC
  pinMode(A0, INPUT);

  digitalWrite(LED_BUILTIN, HIGH);

  if (!bme.begin(0x76))
  {
    Serial.println("BME280 not found !");
  }

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  SensorsValues sensorsValues;
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure();
  sensorsValues.temperature = temperature < -274.0 ? LatestSensorsValues.temperature : std::roundf(temperature * 10.0) / 10.0;
  sensorsValues.humidity = humidity > 100.0 ? LatestSensorsValues.humidity : std::roundf(bme.readHumidity() * 10.0) / 10.0;
  sensorsValues.pressure = pressure < 500.0 ? LatestSensorsValues.pressure : std::roundf(bme.readPressure() / 10.0) / 10.0;
  sensorsValues.battery = getBatteryCapacity();
  Serial.printf("Sensors: %.2f°C %.2f%% %.2fhPa batt:%u%%\r\n",
                temperature,
                humidity,
                pressure,
                sensorsValues.battery);

  if (sensorsValues.temperature != LatestSensorsValues.temperature ||
      sensorsValues.humidity != LatestSensorsValues.humidity ||
      sensorsValues.pressure != LatestSensorsValues.pressure ||
      sensorsValues.battery != LatestSensorsValues.battery ||
      digitalRead(button) == LOW)
  {

    // Activate RF switch control
    pinMode(WIFI_ENABLE, OUTPUT);
    digitalWrite(WIFI_ENABLE, LOW);

    delay(100);

    // Use external antenna
    pinMode(WIFI_ANT_CONFIG, OUTPUT);
    digitalWrite(WIFI_ANT_CONFIG, HIGH);

    // Optional: set Zigbee device name and model
    zbTempSensor.setManufacturerAndModel("ZeSly", "WeatherSensor");

    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor.setMinMaxValue(-40, 60);

    // Set tolerance for temperature measurement in °C (lowest possible value is 0.1°C)
    zbTempSensor.setTolerance(10);

    // Set power source to battery and set battery percentage to measured value
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, sensorsValues.battery);

    // Set minimum and maximum pressure measurement value in hPa
    zbPressureSensor.setMinMaxValue(0, 10000);

    // Optional: Set tolerance for pressure measurement in hPa
    zbPressureSensor.setTolerance(10);

    // Add humidity cluster to the temperature sensor device with min, max and tolerance values
    zbTempSensor.addHumiditySensor(0, 100, 10);

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
      digitalWrite(LED_BUILTIN, LOW);
      ESP.restart();
    }
    else
    {
      Serial.println("Zigbee started successfully!");
    }
    Serial.print("Connecting to network");
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
    SendSensorsValues(sensorsValues);
    delay(1000);
  }

  Serial.println("Going to sleep now");
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
  esp_deep_sleep_start();
}

void loop()
{
  // This is not going to be called
}
