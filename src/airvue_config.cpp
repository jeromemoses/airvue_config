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

Adafruit_VEML7700 veml = Adafruit_VEML7700();

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

#ifdef SENSOR_SERIAL_ENABLE
SoftwareSerial MUX_SERIAL(MUX_TX, MUX_RX); // multiplexer serial port
#endif

byte ch2o_received_bytes[9];
byte ps_received_byte[24];
byte co2_received_bytes[9];
uint8_t getppm[REQUEST_CNT] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
float measurement = 0;
byte buf[RESPONSE_CNT - 1];
byte cheksum = 0;

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

#ifdef SENSOR_SERIAL_ENABLE
  // Multiplexer Serial initialization
  MUX_SERIAL.begin(SENSOR_BAUDRATE);
#endif

  // power button LED pin declaration
  pinMode(BUT_LED_PIN, OUTPUT);
  BUT_LED(1);

  //input button pin initialization
  pinMode(BUT_PIN, INPUT);

  // digital power switch pin declaration
  pinMode(PWR_SWITCH, OUTPUT);

  // MUTIPLEXER TRUETH TABLE PINS DECLARATION
  pinMode(MUX_SS0, OUTPUT);
  pinMode(MUX_SS1, OUTPUT);
  pinMode(MUX_SS2, OUTPUT);
  pinMode(MUX_SS3, OUTPUT);

#ifdef SENSOR_SERIAL_ENABLE
  switch_sensor(PM_PORT);
  delay(1500);
  // pollution sesnor startup code for Q&A mode
  for (int i = 0; i < 9; i++)
  {
    MUX_SERIAL.write(ps_start_QAmode_cmd[i]);
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
  MUX_SERIAL.flush();

  if (response != NULL)
  {
    int i = 0;
    while (MUX_SERIAL.available() <= 0)
    {
      if (++i > WAIT_READ_TIMES)
      {
        Serial.println("can't get ZE15CO response.");
        return;
      }
      delay(WAIT_READ_DELAY);
    }
    MUX_SERIAL.readBytes(response, RESPONSE_CNT);
  }
}

void read_ch2o(float *CH2O)
{
read_again:
{
  MUX_SERIAL.write(ch2o_read_cmd, sizeof(ch2o_read_cmd));
  delay(1000);
  if (MUX_SERIAL.write(ch2o_return_cmd, sizeof(ch2o_return_cmd)) == 9)
  {
    for (byte i = 0; i < 9; i++)
    {
      ch2o_received_bytes[i] = MUX_SERIAL.read();
    }

    // debug code
    // Serial.print("ch2o RC_BYTES <\t");
    // for (int j = 0; j < 9; j++)
    // {
    //   Serial.print(ch2o_received_bytes[j]);
    //   Serial.print("\t");
    // }
    // Serial.println(">");

    // Gas concentration value=High byte of concentration *256+ Low byte of concentration
    *CH2O = (ch2o_received_bytes[2] * 256) + ch2o_received_bytes[3];
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
  if (MUX_SERIAL.write(ps_read_cmd, sizeof(ps_read_cmd)) == 9)
  {
    for (byte i = 0; i < 9; i++)
    {
      ps_received_byte[i] = MUX_SERIAL.read();
    }

    //    debug code
    // Serial.print("PM RC_BYTES <\t");
    // for (byte j = 0; j < 9; j++)
    // {
    //   Serial.print(ps_received_byte[j]);
    //   Serial.print("\t");
    // }
    // Serial.println(">");

    // pm1_0
    *res1 = (0x00 * 256) + ps_received_byte[7];
    //*res1 = *res1/1000;  //converts ug/m3 to ppm

    // pm2_5
    *res2 = (0x00 * 256) + ps_received_byte[3];
    //*res2 = *res2/1000;  //converts ug/m3 to ppm

    // pm10
    *res3 = (0x00 * 256) + ps_received_byte[5];
    //*res3 = *res3/1000;  //converts ug/m3 to ppm
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
  if (MUX_SERIAL.write(co2_start_cmd, sizeof(co2_start_cmd)) == 9)
  {

    for (byte i = 0; i < 9; i++)
    {
      co2_received_bytes[i] = MUX_SERIAL.read();
    }

    // debug code
    // Serial.print("co2 RC_BYTES <\t");
    // for (int j = 0; j < 9; j++)
    // {
    //   Serial.print(co2_received_bytes[j]);
    //   Serial.print("\t");
    // }
    // Serial.println(">");

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

  // clearing serial buffers
  // while (MUX_SERIAL.available() > 0)
  // {
  //   for (int i = 0; i < 5; i++)
  //   {
  //     for (int j = 0; j < 5; j++)
  //     {
  //       char t = Serial.read();
  //       delay(1);
  //     }
  //     delay(10);
  //   }
  //   break;
  // }
  delay(250);
}

void read_co(float *CO)
{
  writeCommand(getppm, buf);

  for (int i = 0; i < 8; i++)
  {
    buf[i] = 0x00;
  }

  writeCommand(getppm, buf);
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
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  Serial.println("-- Default Test --");
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
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin())
  {
    Serial.println("Sensor not found");
    while (1)
      ;
  }
  Serial.println("Sensor found");

  // == OPTIONAL =====
  // Can set non-default gain and integration time to
  // adjust for different lighting conditions.
  // =================
  // veml.setGain(VEML7700_GAIN_1_8);
  // veml.setIntegrationTime(VEML7700_IT_100MS);

  Serial.print(F("Gain: "));
  switch (veml.getGain())
  {
  case VEML7700_GAIN_1:
    Serial.println("1");
    break;
  case VEML7700_GAIN_2:
    Serial.println("2");
    break;
  case VEML7700_GAIN_1_4:
    Serial.println("1/4");
    break;
  case VEML7700_GAIN_1_8:
    Serial.println("1/8");
    break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime())
  {
  case VEML7700_IT_25MS:
    Serial.println("25");
    break;
  case VEML7700_IT_50MS:
    Serial.println("50");
    break;
  case VEML7700_IT_100MS:
    Serial.println("100");
    break;
  case VEML7700_IT_200MS:
    Serial.println("200");
    break;
  case VEML7700_IT_400MS:
    Serial.println("400");
    break;
  case VEML7700_IT_800MS:
    Serial.println("800");
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
  if (irq & VEML7700_INTERRUPT_LOW) {
    //Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    //Serial.println("** High threshold");
  }
}

void goToPowerOff() 
{
  Serial.println("Sleep triggered : )");
  esp_deep_sleep_start();
}