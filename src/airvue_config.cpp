#include <stdio.h>
#include <stdbool.h>
#include <Arduino.h>
#include "airvue_config.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_HS300x.h>
#include <Adafruit_VEML7700.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <ModbusRTU.h>
#include <ToneESP32.h>

ToneESP32 buzzer(BUZZER_PIN, BUZZER_CHANNEL);

ModbusRTU mb;

Adafruit_VEML7700 veml = Adafruit_VEML7700();

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

#ifdef SENSOR_SERIAL_ENABLE
SoftwareSerial STM_serial(26, 32);
SoftwareSerial MUX_SERIAL(MUX_TX, MUX_RX); // multiplexer serial port
#endif

byte ch2o_received_bytes[9];
byte ps_received_byte[24];
byte co2_received_bytes[9];
byte TVOC_received_bytes[9];
uint8_t getppm[REQUEST_CNT] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
float measurement = 0; // co
byte buf[RESPONSE_CNT - 1];
byte cheksum = 0;

int PM_time_out = 0;
int co_time_out = 0;
int ch2o_time_out = 0;
int co2_time_out = 0;
int tvoc_time_out = 0;

bool is_ok_pm = 0;
bool is_ok_co = 0;
bool is_ok_ch2o = 0;
bool is_ok_co2 = 0;
bool is_ok_tvoc = 0;

int PUSH_BUT_timer = 0;
bool sleep_permission = 0;

int read_co_flag = 0;

int Power_switch(bool is_on)
{
  if (is_on == 1)
  {
    digitalWrite(PWR_SWITCH, HIGH);
    return 1;
  }
  else
  {
    digitalWrite(PWR_SWITCH, LOW);
    return 1;
  }
}

int BUT_LED(bool is_on)
{
  if (is_on == 1)
  {
    analogWrite(BUT_LED_PIN, 25);
    return 1;
  }
  else
  {
    analogWrite(BUT_LED_PIN, 0);
    return 1;
  }
}

void sys_startup()
{
  // Debugging serial initialization
  Serial.begin(115200);
  STM_serial.begin(115200);

#ifdef SENSOR_SERIAL_ENABLE
  // Multiplexer Serial initialization
  MUX_SERIAL.begin(SENSOR_BAUDRATE);
#endif

  // power button LED pin declaration
  pinMode(BUT_LED_PIN, OUTPUT);
  BUT_LED(1);

  // input button pin initialization
  pinMode(BUT_PIN, INPUT);

  // digital power switch pin declaration
  pinMode(PWR_SWITCH, OUTPUT);
  Power_switch(1);
  delay(500);

  // MUTIPLEXER TRUETH TABLE PINS DECLARATION
  pinMode(MUX_SS0, OUTPUT);
  pinMode(MUX_SS1, OUTPUT);
  pinMode(MUX_SS2, OUTPUT);
  pinMode(MUX_SS3, OUTPUT);

  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);

#ifdef SENSOR_SERIAL_ENABLE
  switch_sensor(PM_PORT);
  delay(1500);
  // pollution sesnor startup code for Q&A mode
  for (int i = 0; i < 9; i++)
  {
    MUX_SERIAL.write(ps_start_QAmode_cmd[i]);
  }

  switch_sensor(EX_TERMINAL_5V);
  delay(2000);
  // formaldehyde sensor startup code for Q&A mode
  for (int i = 0; i < 9; i++)
  {
    MUX_SERIAL.write(TVOC_start_QAmode_cmd[i]);
  }

#endif
  return;
}

int switch_sensor(int port)
{
  switch (port)
  {
  case CO2_PORT:
    MULTIPLEX_SWITCH(0, 0, 0, 0); // 0 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case CO_PORT:
    MULTIPLEX_SWITCH(1, 0, 0, 0); // 1 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case CH2O_PORT:
    MULTIPLEX_SWITCH(0, 1, 0, 0); // 2 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case PM_PORT:
    MULTIPLEX_SWITCH(1, 1, 0, 0); // 3 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case MODBUS_PORT:
    MULTIPLEX_SWITCH(0, 0, 1, 0); // 4 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case MIC_PORT:
    MULTIPLEX_SWITCH(1, 0, 1, 0); // 5 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case EX_TERMINAL_5V:
    MULTIPLEX_SWITCH(0, 1, 1, 0); // 6 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  case EX_TERMINAL_3V:
    MULTIPLEX_SWITCH(1, 1, 1, 0); // 7 PORT ON MULTIPLEXER TRUTH TABLE(CD74HC4067)
    return port;
    break;
  default:
    return WRONG_PORT;
    break;
  }
}

void MULTIPLEX_SWITCH(int s0, int s1, int s2, int s3)
{
  if (s0 == 1)
    digitalWrite(MUX_SS0, HIGH);
  else
    digitalWrite(MUX_SS0, LOW);

  if (s1 == 1)
    digitalWrite(MUX_SS1, HIGH);
  else
    digitalWrite(MUX_SS1, LOW);

  if (s2 == 1)
    digitalWrite(MUX_SS2, HIGH);
  else
    digitalWrite(MUX_SS2, LOW);

  if (s3 == 1)
    digitalWrite(MUX_SS3, HIGH);
  else
    digitalWrite(MUX_SS3, LOW);
}

#ifdef SENSOR_SERIAL_ENABLE

void writeCommand(uint8_t cmd[], uint8_t *response)
{
  MUX_SERIAL.write(cmd, REQUEST_CNT);

  if (response != NULL)
  {
    int i = 0;
    while (MUX_SERIAL.available() <= 0)
    {
      if (++i > WAIT_READ_TIMES)
      {
        Serial.println("can't get ZE15CO response.");
        read_co_flag = 1;
        is_ok_co = 1;
        return;
      }
      read_co_flag = 0;
      delay(WAIT_READ_DELAY);
    }
    MUX_SERIAL.readBytes(response, RESPONSE_CNT);
  }
}

void read_ch2o(float *CH2O)
{
read_again:
{
  // clearing serial buffers
  while (MUX_SERIAL.available() > 0)
  {
    for (int i = 0; i < 5; i++)
    {
      for (int j = 0; j < 5; j++)
      {
        char t = Serial.read();
        delay(1);
      }
      delay(10);
    }
    break;
  }
  delay(500);

  MUX_SERIAL.write(ch2o_read_cmd, sizeof(ch2o_read_cmd));
  delay(500);
  if (MUX_SERIAL.write(ch2o_return_cmd, sizeof(ch2o_return_cmd)) == 9)
  {
    for (byte i = 0; i < 9; i++)
    {
      ch2o_received_bytes[i] = MUX_SERIAL.read();
    }

    // debug code
    Serial.print("ch2o RC_BYTES <\t");
    for (int j = 0; j < 9; j++)
    {
      Serial.print(ch2o_received_bytes[j]);
      Serial.print("\t");
    }
    Serial.println(">");

    // Serial.print("ch2o RC_HEX <\t");
    // for (int j = 0; j < 9; j++)
    // {
    //   Serial.print(ch2o_received_bytes[j], HEX);
    //   Serial.print("\t");
    // }
    // Serial.println(">");

    // Gas concentration value=High byte of concentration *256+ Low byte of concentration
    *CH2O = (ch2o_received_bytes[6] * 256) + ch2o_received_bytes[7];
  }
}
  delay(1000);
  while (ch2o_received_bytes[8] == 255)
  {
    goto read_again;
  }
}

void read_ps_PM(int *res1, int *res2, int *res3)
{
read_again:
{
  MUX_SERIAL.flush();
  delay(1000);

  if (MUX_SERIAL.write(ps_read_cmd, sizeof(ps_read_cmd)) == 9)
  {
    for (byte i = 0; i < 9; i++)
    {
      ps_received_byte[i] = MUX_SERIAL.read();
    }

    //    debug code
    Serial.print("PM RC_BYTES <\t");
    for (byte j = 0; j < 9; j++)
    {
      Serial.print(ps_received_byte[j]);
      Serial.print("\t");
    }
    Serial.println(">");

    // pm1_0
    *res1 = (0x00 * 256) + ps_received_byte[7];
    //*res1 = *res1/1000;  //converts ug/m3 to ppm

    // pm2_5
    *res2 = (0x00 * 256) + ps_received_byte[3];
    //*res2 = *res2/1000;  //converts ug/m3 to ppm

    // pm10
    *res3 = (0x00 * 256) + ps_received_byte[5];
    //*res3 = *res3/1000;  //converts ug/m3 to ppm

    if (*res1 == 0 && *res2 > 240 && *res3 == 0)
    {
      *res1 = 0;
      *res2 = 0;
      *res3 = 0;
    }

    if (*res1 == 255 && *res2 == 255 && *res3 == 255)
    {
      *res1 = 0;
      *res2 = 0;
      *res3 = 0;
    }
  }
}
  delay(1000);
  while (ps_received_byte[8] == 255)
  {
    goto read_again;
  }
}

void read_co2(int *CO2)
{
read_again:
{
  MUX_SERIAL.flush();
  delay(1000);

  if (MUX_SERIAL.write(co2_start_cmd, sizeof(co2_start_cmd)) == 9)
  {

    for (byte i = 0; i < 9; i++)
    {
      co2_received_bytes[i] = MUX_SERIAL.read();
    }

    // debug code
    Serial.print("co2 RC_BYTES <\t");
    for (int j = 0; j < 9; j++)
    {
      Serial.print(co2_received_bytes[j]);
      Serial.print("\t");
    }
    Serial.println(">");

    *CO2 = (co2_received_bytes[2] * 255) + co2_received_bytes[3];

    if (*CO2 == 65280)
    {
      *CO2 = 0;
    }
  }
}
  delay(1000);
  while (co2_received_bytes[8] == 255)
  {
    goto read_again;
  }
}

void clear_serial()
{
  MUX_SERIAL.end();
  delay(250);
  MUX_SERIAL.begin(SENSOR_BAUDRATE);
  delay(750);
}

void read_co(float *CO)
{
co_read_again:
{
  // MUX_SERIAL.flush();
  delay(1000);

  writeCommand(getppm, buf);

  // for (int i = 0; i < 8; i++)
  // {
  //   buf[i] = 0x00;
  // }

  // writeCommand(getppm, buf);
  // parse
  cheksum = (buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);
  cheksum = (~cheksum) + 1;

  if (buf[0] == 0xff && buf[1] == 0x86 && buf[8] == cheksum)
  {
    measurement = (buf[2] * 256 + buf[3]) * 0.1;
  }
  else
  {
    measurement = -1;
  }
  *CO = measurement;
  if (*CO > 500)
  {
    *CO = 0;
  }

  if (*CO < 0)
  {
    *CO = 0.5;
  }
}
  if (read_co_flag)
  {
    goto co_read_again;
  }
}
#endif

void start_BME()
{
  unsigned status;
  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    // Serial.print("SensorID was: 0x");
    // Serial.println(bme.sensorID(), 16);
    // Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    // Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    // Serial.print("        ID of 0x60 represents a BME 280.\n");
    // Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  // Serial.println("-- Default Test --");
}

void read_bme(int bme_channel, float *result)
{
  switch (bme_channel)
  {
  case PRESSURE:
    *result = bme.readPressure() / 100.0F;
    break;

  case ALTITUDE:
    *result = bme.readAltitude(SEALEVELPRESSURE_HPA);
    break;

  case TEMPERATURE:
    *result = bme.readTemperature();
    break;

  case HUMIDITY:
    *result = bme.readHumidity();
    break;

  default:
    break;
  }
}

void start_hs3003()
{
  if (!HS300x.begin())
  {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
    {
      Serial.println("Re-trying :)");
      if (HS300x.begin())
      {
        break;
      }
    }
  }
}

void read_hs3003(int HS_CAHNNEL, float *result)
{
  switch (HS_CAHNNEL)
  {
  case HS_TEMP:
    *result = HS300x.readTemperature();
    break;

  case HS_HUMID:
    *result = HS300x.readHumidity();
    break;

  default:
    break;
  }
}

void start_VEML7700()
{
  // Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin())
  {
    Serial.println("Sensor not VEML7700");
    while (1)
    {
      Serial.println("Retrying to connect with VEML7700");
      if (veml.begin())
      {
        break;
      }
    }
  }
  // Serial.println("Sensor found");

  // == OPTIONAL =====
  // Can set non-default gain and integration time to
  // adjust for different lighting conditions.
  // =================
  // veml.setGain(VEML7700_GAIN_1_8);
  // veml.setIntegrationTime(VEML7700_IT_100MS);

  // Serial.print(F("Gain: "));
  switch (veml.getGain())
  {
  case VEML7700_GAIN_1:
    // Serial.println("1");
    break;
  case VEML7700_GAIN_2:
    // Serial.println("2");
    break;
  case VEML7700_GAIN_1_4:
    // Serial.println("1/4");
    break;
  case VEML7700_GAIN_1_8:
    // Serial.println("1/8");
    break;
  }

  // Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime())
  {
  case VEML7700_IT_25MS:
    // Serial.println("25");
    break;
  case VEML7700_IT_50MS:
    // Serial.println("50");
    break;
  case VEML7700_IT_100MS:
    // Serial.println("100");
    break;
  case VEML7700_IT_200MS:
    // Serial.println("200");
    break;
  case VEML7700_IT_400MS:
    // Serial.println("400");
    break;
  case VEML7700_IT_800MS:
    // Serial.println("800");
    break;
  }

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
}

void read_VEML7700(int LUX_CHANNEL, float *result)
{
  switch (LUX_CHANNEL)
  {
  case VEML_ALS:
    *result = veml.readALS();
    break;

  case VEML_WHITE:
    *result = veml.readWhite();
    break;

  case VEML_LUX:
    *result = veml.readLux();
    break;

  default:
    break;
  }

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW)
  {
    // Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH)
  {
    // Serial.println("** High threshold");
  }
}

void goToPowerOff()
{
  // Serial.println("Sleep triggered : )");
  esp_deep_sleep_start();
}

void button_sleep_handle()
{
  while (digitalRead(BUT_PIN))
  {
    PUSH_BUT_timer++;
    vTaskDelay(250);
    if (PUSH_BUT_timer >= 4)
    {
      sleep_permission = 1;
      BUT_LED(0);
      break;
    }
  }
  PUSH_BUT_timer = 0;

  if (sleep_permission)
  {
    Power_switch(0);
    delay(1000);
    // Trigger System OFF after 5 interrupts
    goToPowerOff();
  }
  return;
}

void stm_relay(bool rly1, bool rly2)
{
  if (rly1)
  {
    STM_serial.print(3); // relay 1 off on
  }
  else
  {
    STM_serial.print(4);
  }

  if (rly1)
  {
    STM_serial.print(5);
  }
  else
  {
    STM_serial.print(6);
  }
}

void get_stm_data(float *eto, float *h2s, float *nh3, float *no2, float *o2, float *so2)
{
  STM_serial.print(1);
  String json;
  json = STM_serial.readStringUntil('\n');
  json.trim();
  // Serial.printf("\n\n %s \n\n", json.c_str());
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str()); // failed heer
    return;
  }

  *eto = doc["ETO"];
  *h2s = doc["H2S"];
  *nh3 = doc["NH3"];
  *no2 = doc["NO2"];
  *o2 = doc["O2"];
  *so2 = doc["SO2"];
  // Serial.printf("Extracted data: \nETO :%f\nH2S :%f\nNH3 :%f\nNO2 :%f\nO2 :%f\nSO2 :%f\n", *eto, *h2s, *nh3, *no2, *o2, *so2);
  json.clear();
}

void MODBUS_init()
{
  mb.begin(&Serial);
  mb.slave(SLAVE_ID);

  mb.addHreg(REGN_co2);
  mb.addHreg(REGN_ch2o);
  mb.addHreg(REGN_temp);
  mb.addHreg(REGN_humid);
  mb.addHreg(REGN_pm1);
  mb.addHreg(REGN_pm2_5);
  mb.addHreg(REGN_pm10);
  mb.addHreg(REGN_co);
  mb.addHreg(REGN_aqi);
  mb.addHreg(REGN_lux);
  mb.addHreg(REGN_pressure);
  mb.addHreg(REGN_altitude);
  mb.addHreg(REGN_ETO);
  mb.addHreg(REGN_H2S);
  mb.addHreg(REGN_NH3);
  mb.addHreg(REGN_NO2);
  mb.addHreg(REGN_O2);
  mb.addHreg(REGN_SO2);
  mb.addHreg(REGN_TVOC);
}

void MODBUS_push(struct modbus_parameter MD_data)
{
  mb.Hreg(REGN_co2, MD_data.CO2);
  mb.Hreg(REGN_ch2o, MD_data.CH2O);
  mb.Hreg(REGN_temp, MD_data.TEMPERATURE);
  mb.Hreg(REGN_humid, MD_data.HUMIDITY);
  mb.Hreg(REGN_pm1, MD_data.PM1);
  mb.Hreg(REGN_pm2_5, MD_data.PM2_5);
  mb.Hreg(REGN_pm10, MD_data.PM10);
  mb.Hreg(REGN_co, MD_data.CO);
  mb.Hreg(REGN_aqi, MD_data.AQI);
  mb.Hreg(REGN_lux, MD_data.LUX);
  mb.Hreg(REGN_pressure, MD_data.PRESSURE);
  mb.Hreg(REGN_altitude, MD_data.ALTITUDE);
  mb.Hreg(REGN_ETO, MD_data.ETO);
  mb.Hreg(REGN_H2S, MD_data.H2S);
  mb.Hreg(REGN_NH3, MD_data.NH3);
  mb.Hreg(REGN_NO2, MD_data.NO2);
  mb.Hreg(REGN_O2, MD_data.O2);
  mb.Hreg(REGN_SO2, MD_data.SO2);
  mb.Hreg(REGN_TVOC, MD_data.TVOC);
  mb.task();
}

void NOTIFY_BUZZER(int data, int threshold)
{
  if (data > threshold)
  {
    buzzer.tone(NOTE_C4, 250);
    buzzer.tone(NOTE_D4, 250);
    buzzer.tone(NOTE_E4, 250);
    buzzer.tone(NOTE_F4, 250);
    buzzer.tone(NOTE_G4, 250);
    buzzer.tone(NOTE_A4, 250);
    buzzer.tone(NOTE_B4, 250);
    buzzer.tone(NOTE_C5, 250);
    delay(250);
    buzzer.tone(NOTE_C5, 250);
    buzzer.tone(NOTE_B4, 250);
    buzzer.tone(NOTE_A4, 250);
    buzzer.tone(NOTE_G4, 250);
    buzzer.tone(NOTE_F4, 250);
    buzzer.tone(NOTE_E4, 250);
    buzzer.tone(NOTE_D4, 250);
    buzzer.tone(NOTE_C4, 250);
    buzzer.noTone();
  }
}

void read_TVOC(float *tvoc)
{
read_again:
{
  if (MUX_SERIAL.write(TVOC_read_cmd, sizeof(TVOC_read_cmd)) == 9)
  {
    for (byte i = 0; i < 9; i++)
    {
      TVOC_received_bytes[i] = MUX_SERIAL.read();
    }

    // debug code
    Serial.print("TVOC RC_BYTES <\t");
    for (int j = 0; j < 9; j++)
    {
      Serial.print(TVOC_received_bytes[j]);
      Serial.print("\t");
    }
    Serial.println(">");

    // Gas concentration value=High byte of concentration *256+ Low byte of concentration
    *tvoc = (TVOC_received_bytes[6] * 256) + TVOC_received_bytes[7]; // ug/m³ or ppb
    //*tvoc = *tvoc/1000; // mg/m³ or ppm
  }
}
  delay(1000);
  while (TVOC_received_bytes[8] == 255)
  {
    goto read_again;
  }
}

int Sensor_test_handle()
{
  if (Serial.read() == '7')
  {
    ///// PM sensor testing block /////
    switch_sensor(PM_PORT);
    vTaskDelay(500);
    clear_serial();

  PM_read_again:
  {
    PM_time_out += 1;
    MUX_SERIAL.flush();
    vTaskDelay(500);

    if (MUX_SERIAL.write(ps_read_cmd, sizeof(ps_read_cmd)) == 9)
    {
      for (byte i = 0; i < 9; i++)
      {
        ps_received_byte[i] = MUX_SERIAL.read();
      }
    }
  }
    if (ps_received_byte[8] != 255)
    {
      is_ok_pm = 1;
    }

    vTaskDelay(500);
    while ((ps_received_byte[8] == 255) && (PM_time_out <= 4) && !(is_ok_pm))
    {
      goto PM_read_again;
    }

    ///// ch2o sensor testing block /////
    switch_sensor(CH2O_PORT);
    vTaskDelay(500);
    clear_serial();
  ch2o_read_again:
  {
    ch2o_time_out += 1;
    // clearing serial buffers
    while (MUX_SERIAL.available() > 0)
    {
      for (int i = 0; i < 5; i++)
      {
        for (int j = 0; j < 5; j++)
        {
          char t = Serial.read();
          vTaskDelay(500);
        }
        vTaskDelay(500);
      }
      break;
    }
    vTaskDelay(500);

    MUX_SERIAL.write(ch2o_read_cmd, sizeof(ch2o_read_cmd));
    vTaskDelay(500);
    if (MUX_SERIAL.write(ch2o_return_cmd, sizeof(ch2o_return_cmd)) == 9)
    {
      for (byte i = 0; i < 9; i++)
      {
        ch2o_received_bytes[i] = MUX_SERIAL.read();
      }
    }
  }
    if (ch2o_received_bytes[8] != 255)
    {
      is_ok_ch2o = 1;
    }
    vTaskDelay(500);
    while ((ch2o_received_bytes[8] == 255) && (ch2o_time_out <= 4) && !(is_ok_ch2o))
    {
      goto ch2o_read_again;
    }

    ///// Co sensor testing block /////
    switch_sensor(CO_PORT);
    vTaskDelay(500);
    clear_serial();
  co_read_again:
  {
    co_time_out += 1;
    // MUX_SERIAL.flush();
    vTaskDelay(500);

    writeCommand(getppm, buf);
    cheksum = (buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);
    cheksum = (~cheksum) + 1;
  }
    if ((read_co_flag) && !(is_ok_co) && (co_time_out <= 4))
    {
      goto co_read_again;
    }

    ///// Co2 sensor testng block /////
    switch_sensor(CO2_PORT);
    vTaskDelay(500);
    clear_serial();
  co2_read_again:
  {
    co2_time_out += 1;
    MUX_SERIAL.flush();
    vTaskDelay(500);

    if (MUX_SERIAL.write(co2_start_cmd, sizeof(co2_start_cmd)) == 9)
    {
      for (byte i = 0; i < 9; i++)
      {
        co2_received_bytes[i] = MUX_SERIAL.read();
      }
    }
  }
    if (co2_received_bytes[8] != 255)
    {
      is_ok_co2 = 1;
    }
    vTaskDelay(500);
    while ((co2_received_bytes[8] == 255) && (co2_time_out <= 4) && !(is_ok_co2))
    {
      goto co2_read_again;
    }

    ///// Tvoc sensor testing block /////
    switch_sensor(EX_TERMINAL_5V);
    vTaskDelay(500);
    clear_serial();
  tvoc_read_again:
  {
    tvoc_time_out += 1;
    if (MUX_SERIAL.write(TVOC_read_cmd, sizeof(TVOC_read_cmd)) == 9)
    {
      for (byte i = 0; i < 9; i++)
      {
        TVOC_received_bytes[i] = MUX_SERIAL.read();
      }
    }
  }
    vTaskDelay(500);
    while ((TVOC_received_bytes[8] == 255) && (tvoc_time_out <= 4) && !(is_ok_tvoc))
    {
      goto tvoc_read_again;
    }
  }
  return 1;
}

int check_sensor(int test_at)
{
  switch (test_at)
  {
  case PM:
    return is_ok_pm;
    break;

  case CO:
    return is_ok_co;
    break;

  case CH2O:
    return is_ok_ch2o;
    break;

  case CO2:
    return is_ok_co2;
    break;

  case TVOC:
    return is_ok_tvoc;
    break;
  
  default:
    break;
  }
  return 0;
}