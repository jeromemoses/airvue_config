#include <Arduino.h>
#include "airvue_config.h"

int co2 = 0; // Buffer for CO2
float ch2o = 0;
float temp = 0;
float humidity = 0;
int pm1_0 = 0;
int pm2_5 = 0;
int pm10 = 0;
float co = 0;
double AQI = 0;
unsigned long getDataTimer = 0;

float pressure = 0;
float altitude = 0;
float bme_temp = 0;
float bme_humid = 0;

float lux = 0;
float als = 0;
float white = 0; 

void setup()
{
  sys_startup();
  Power_switch(1);
  BUT_LED(1);
  Serial.println("Initializing...");
  delay(7000);
  Serial.println("READY");
  start_BME();
  start_hs3003();
  start_VEML7700();
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

    read_hs3003(HS_TEMP, &temp);
    read_hs3003(HS_HUMID, &humidity);

    read_bme(PRESSURE, &pressure);
    read_bme(ALTITUDE, &altitude);
    read_bme(TEMPERATURE, &bme_temp);
    read_bme(HUMIDITY, &bme_humid);

    read_VEML7700(VEML_ALS, &als);
    read_VEML7700(VEML_WHITE, &white);
    read_VEML7700(VEML_LUX, &lux);

    get_stm_data(&ETO, &H2S, &NH3, &NO2, &O2, &SO2);

    Serial.printf("ETO\t :\t%f \nH2S\t :\t%f \nNH3\t :\t%f \nNO2\t :\t%f \nO2\t :\t%f \nSO2\t :\t%f \n", ETO, H2S, NH3, NO2, O2, SO2);
    Serial.printf("pm1\t :\t%d\npm25\t :\t%d\npm10\t :\t%d \n", pm1_0, pm2_5, pm10);
    Serial.printf("AQI\t :\t%d \n", AQI);
    Serial.printf("ch2o\t :\t%.0f\n", ch2o);
    Serial.printf("CO\t :\t%.3f \n", co);
    Serial.printf("co2\t :\t%d\n", co2);
    Serial.printf("temp\t :\t%.2f \nhumid\t :\t%.2f\n", temp, humidity);
    Serial.printf("ALS:\t :\t%.3f\n", als);
    Serial.printf("White:\t :\t%.3f\n", white);
    Serial.printf("Lux:\t :\t%.3f\n------------------------------\n", lux);
}