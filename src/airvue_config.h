#define BUT_LED_PIN 27
#define PWR_SWITCH 13
#define MUX_RX 25
#define MUX_TX 23
#define MUX_SS0 18
#define MUX_SS1 17
#define MUX_SS2 15
#define MUX_SS3 19
#define SENSOR_BAUDRATE 9600 //sensors baud rate

#define SENSOR_SERIAL_ENABLE //define SENSOR_SERIAL_ENABLE when using sensor on UART on multiplexer
//#undef SENSOR_SERIAL_ENABLE  //command this when using sensor UART on multiplexer

//functions declarations
//////////////// high priority to call on void setup() //////////////////
void sys_startup(); //you must call this funtion at the begining in the void setup();
int BUT_LED(bool); //to switch ON/OFF the led on the button --> example: BUT_LED(1) to ON , BUT_LED(0) to OFF 
int Power_switch(bool); // to switch 3.3V and 5V sensors connection --> example: Power_switch(1) -> ON , Power_switch(1) -> OFF 
//write the below line at the end of the void setup to initialize all the i2c sensors
void start_BME();
void start_hs3003();
void start_VEML7700();
////////////////////////////////////////////////////////////////////////

//Inloop functions  
enum MULTIPLEX{WRONG_PORT, CO2_PORT, CO_PORT, CH2O_PORT, PM_PORT, MODBUS_PORT, MIC_PORT, EX_TERMINAL_5V, EX_TERMINAL_3V }; //mux port numbers
int switch_sensor(int); //you must call this function and pass the argument of the sensor channel you want to use. -->example switch_sensor(CH2O_PORT);

void MULTIPLEX_SWITCH(int, int, int, int); //Automated in library

void read_ps_PM(int *, int *, int *); //to read the pm sensor value you have to pass the address of the variable you want to store the value to. -->example: read_ps_PM(&pm1_0, &pm2_5, &pm10);
void read_ch2o(float *); //To read you have to pass the address of the variable that you want to store the data at. example --> read_ch2o(&ch2o);
void writeCommand(uint8_t cmd[], uint8_t *response);//Automated in library
void read_co2(int*); //pass the address of the variable that you want to store the data at--> example: read_co2(&co2);
void clear_serial(); //have to clear the serial monitor every time when the sensor multiplexer channel is changed
void read_co(float*); //read_co(&co);

enum BME_channels{PRESSURE = 1, ALTITUDE, TEMPERATURE, HUMIDITY}; //BME sensor function arguments to switch between parameters (Example: read_bme(PRESSURE, &pressure);)
void read_bme(int,float*); // you have to pass what parameter you want to measure and address of the variable you want to store the data at. --> example: read_bme(PRESSURE, &pressure);

enum HS3003_parameter{HS_TEMP=1, HS_HUMID};
void read_hs3003(int,float*); //function is same ass the bme sensor. --> example: read_hs3003(HS_TEMP, &temp); 

enum VEML7700_REG{VEML_ALS=1, VEML_WHITE, VEML_LUX};
void read_VEML7700(int, float*); // example: read_VEML7700(VEML_LUX, &lux);

// Pollution sensor commands
const byte ps_start_QAmode_cmd[] = {0XFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
const byte ps_read_cmd[] = {0XFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

//CO sensor dependencies
static const int REQUEST_CNT = 9;
static const int RESPONSE_CNT = 9;
#define WAIT_READ_TIMES 100
#define WAIT_READ_DELAY 10


//Co2
const byte co2_start_cmd[] = {0XFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

// ch2o commands
const byte ch2o_start_QAmode_cmd[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
const byte ch2o_activeUpload_mode[] = {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};
const byte ch2o_read_cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const byte ch2o_return_cmd[] = {0xFF, 0x86, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x20, 0x30};