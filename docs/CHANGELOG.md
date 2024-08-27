# ChangeLog for Airvue_config LIB

## [1.3.9] - 2024-8-27
#### -> Added Sensor state checking functionality -> ensor_test_handle();
#### -> Added a function to get sensor test flag check_sensor(int);
#### -> Updated example code with all the new features.

## [1.2.9] - 2024-8-12
#### -> Added TVOC Sensor read functionality -> read_TVOC(*float);
#### -> Added CO sensor read Error handling functionality -> function will retry to fetch data if its not correct
#### -> Added relay terminal control functionality via ESP32 -> STM32 

## [1.2.8] - 2024-8-9
#### -> Fixed Sensor uart - changed HardWareSerial to SoftWareSerial
#### -> Added sensor data Serial printing for debugging
#### -> Fixed - push button sleep mode handler been triggering watchdog timer

## [1.2.7] - 2024-8-5
#### -> Added MODBUS functionalities -> MODBUS_init(); , MODBUS_push();
#### -> Added BUZZER functionalities -> NOTIFY_BUZZER(data, threshold);
#### -> Added sleep mode functionality with push button -> button_sleep_handle();

## [1.1.7] - 2024-6-22
#### ->Added stm32 data fetch functionality -> get_stm_data(...);
#### ->Fixed json fetch issue on reading sub mcu ESP32 <- STM32
#### ->Fixed Sensor data exception errors(CO, PM)

## [1.1.6] - 2024-6-18
### Serial>error fixes with Multiplexed uart pin
#### ->solved ch2o expection values
#### ->added ch2o default return commands
#### ->Automated power switching to be called inside sys_startup() instead on void setup(); --> Power_switch(1);
#### ->Changed read method of ch2o sensor

## [1.1.5] - 2024-6-16
### Serial.flush() fix with co sensor
#### ->removed unwanted serial flushes
#### ->removed MUX_SERIAL.end() on clear_serial() and fixes sensor value errors 

## [1.1.4] - 2024-6-16
### Fixed serial terminating issue with PM sensor
#### Replaced MUX_SERIAL.flush() instead of serial.end()

## [1.1.3] - 2024-6-14
### fixed But led to turns-ON on sys_startup() itself

## [1.1.2] - 2024-6-14
### defined button input

## [1.1.1] - 2024-6-12
### Define ON-board BUT_LED pin in header and Fixed button's LED shutdown error  

## [1.1.0] - 2024-6-12
### Added sleep mode functionality with on board button

## [1.0.0] - 2024-6-12
### Initial commit