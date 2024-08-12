#include <Arduino.h>
#include "airvue_config.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

TaskHandle_t task1Handle;

WiFiManager my_wifi;

void task1(void *pvParameters)
{
  while (true)
  {
    button_sleep_handle();
  }
}

int co2 = 0; // Buffer for CO2
float ch2o = 0;
float temp = 0;
float humidity = 0;
int pm1_0 = 0;
int pm2_5 = 0;
int pm10 = 0;
float co = 0;
double aqi = 0;
unsigned long getDataTimer = 0;

float pressure = 0;
float altitude = 0;
float bme_temp = 0;
float bme_humid = 0;

float lux = 0;
float als = 0;
float white = 0;

float eto = 0;
float h2s = 0;
float nh3 = 0;
float no2 = 0;
float o2 = 0;
float so2 = 0;
float tvoc = 0;

void setup()
{
  sys_startup();
  Serial.println("Initializing...");
  delay(7000);
  Serial.println("READY");
  start_BME();
  start_hs3003();
  start_VEML7700();
  xTaskCreate(task1, "Task 1", 5000, NULL, 1, &task1Handle);
}

void loop()
{
  switch_sensor(PM_PORT);
  delay(2000);
  clear_serial();
  read_ps_PM(&pm1_0, &pm2_5, &pm10);

  switch_sensor(CH2O_PORT);
  delay(2000);
  clear_serial();
  read_ch2o(&ch2o);

  // co sensor code
  switch_sensor(CO_PORT);
  delay(2000);
  clear_serial();
  read_co(&co);

  switch_sensor(CO2_PORT);
  delay(2000);
  clear_serial();
  read_co2(&co2);

  switch_sensor(EX_TERMINAL_5V);
  delay(2000);
  clear_serial();
  read_TVOC(&tvoc);

  read_hs3003(HS_TEMP, &temp);
  read_hs3003(HS_HUMID, &humidity);

  read_bme(PRESSURE, &pressure);
  read_bme(ALTITUDE, &altitude);
  read_bme(TEMPERATURE, &bme_temp);
  read_bme(HUMIDITY, &bme_humid);

  read_VEML7700(VEML_ALS, &als);
  read_VEML7700(VEML_WHITE, &white);
  read_VEML7700(VEML_LUX, &lux);

  get_stm_data(&eto, &h2s, &nh3, &no2, &o2, &so2);

  Serial.printf("ETO\t :\t%f \nH2S\t :\t%f \nNH3\t :\t%f \nNO2\t :\t%f \nO2\t :\t%f \nSO2\t :\t%f \n", eto, h2s, nh3, no2, o2, so2);
  Serial.printf("pm1\t :\t%d\npm25\t :\t%d\npm10\t :\t%d \n", pm1_0, pm2_5, pm10);
  Serial.printf("AQI\t :\t%d \n", aqi);
  Serial.printf("ch2o\t :\t%.0f\n", ch2o);
  Serial.printf("CO\t :\t%.3f \n", co);
  Serial.printf("co2\t :\t%d\n", co2);
  Serial.printf("temp\t :\t%.2f \nhumid\t :\t%.2f\n", temp, humidity);
  Serial.printf("ALS:\t :\t%.3f\n", als);
  Serial.printf("White:\t :\t%.3f\n", white);
  Serial.printf("Pressure:\t :\t%.4f\n", pressure);
  Serial.printf("Altitude:\t :\t%.4f\n", altitude);
  Serial.printf("Lux:\t :\t%.3f\n------------------------------\n", lux);

  // to set the notification on buzzer for any parameter with threshold
  NOTIFY_BUZZER(HS_TEMP, 28); // buzzer will be trigger if temperature exceeds 28 degree

  // Follow the below code sequence if you need to enable the modbus
  // creat the structure for modbus parameters the struct is in library by default
  struct modbus_parameter Modbus_DB;

  // Pass the sensor datas to the structure members
  Modbus_DB.CO2 = co2;
  Modbus_DB.CH2O = ch2o;
  Modbus_DB.TEMPERATURE = temp;
  Modbus_DB.HUMIDITY = humidity;
  Modbus_DB.PM1 = pm1_0;
  Modbus_DB.PM2_5 = pm2_5;
  Modbus_DB.PM10 = pm10;
  Modbus_DB.CO = co;
  Modbus_DB.AQI = aqi;
  Modbus_DB.LUX = lux;
  Modbus_DB.PRESSURE = pressure;
  Modbus_DB.ALTITUDE = altitude;
  Modbus_DB.ETO = eto;
  Modbus_DB.H2S = h2s;
  Modbus_DB.NH3 = nh3;
  Modbus_DB.NO2 = no2;
  Modbus_DB.O2 = o2;
  Modbus_DB.SO2 = so2;
  Modbus_DB.TVOC = tvoc;

  // switch the multiplexer to modbus port
  switch_sensor(MODBUS_PORT);
  delay(1000);
  clear_serial();
  delay(3000);

  // initialize modbus before using
  MODBUS_init();
  delay(1500);

  // pass the structure to the function which pushes the data through modbus as Read holding register (20) -> slave ID 77
  MODBUS_push(Modbus_DB);
  delay(1000);
  MODBUS_push(Modbus_DB);
  delay(1000);
  MODBUS_push(Modbus_DB);
  delay(1000);

  //to control the relay terminal connected through stm32 (1st Relay, 2nd Relay)
  stm_relay(1,0);
}

/**
 * platformio.ini
 *
 * lib_deps =
       emelianov/modbus-esp8266 @ ^4.1.0
       plerup/EspSoftwareSerial @ ^8.2.0
       bblanchon/ArduinoJson @ ^7.0.3
       plerup/EspSoftwareSerial @ ^8.2.0
       adafruit/Adafruit BME280 Library @ ^2.2.4
       arduino-libraries/Arduino_HS300x @ ^1.0.0
       adafruit/Adafruit VEML7700 Library @ ^2.1.6
       wnatth3/WiFiManager @ 2.0.16-rc.2
       arduino-libraries/NTPClient @ ^3.2.1
       sstaub/NTP @ ^1.6
       weedmanu/ToneESP32 @ ^1.0.0
       https://github.com/jeromemoses/airvue_config.git#v1.2.7
 *
 *
*/