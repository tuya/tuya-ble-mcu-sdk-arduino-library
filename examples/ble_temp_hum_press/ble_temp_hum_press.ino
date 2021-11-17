/*
 * @FileName: BLE_TEMP_HUM_PRESS.ino
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-10-22 10:41:27
 * @LastEditTime: 2021-10-27 19:48:31
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: This demo is based on the Arduino UNO, and the LEDs on the UNO board are controlled by the Tuya Smart App. 
 *               Enter network connection mode when Pin6 to GND.
 * @Github:
 */
#include <TuyaBLE.h>
#include <Seeed_BMP280.h>
#include <Wire.h>
#include <DHT.h>

/* object statement */
#define WIRE Wire
TuyaBLE my_device;
BMP280 bmp280;

#define LED_PIN  4

/* button pin */
#define KEY_PIN  6

/* what pin we're connected to */
#define DHTPIN 3 

/* air pressure sensor choose and init*/
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/* set led blink interval (500ms) */
#define LED_BLINK_INTERVAL  500     

/* upload data interval(5000ms) */
#define UPLOAD_INTERVAL     500

/* Current LED status */
unsigned char led_state = 0;

/* have three temperature warning types */
typedef enum {
  TEMP_LOWER,
  TEMP_UPPER,
  TEMP_CANCEL,
  TEMP_NOT_EXIT,
}TEMP_WARN_TYPE_E;

/* have three humitudy warning types */
typedef enum {
  HUM_LOWER,
  HUM_UPPER,
  HUM_CANCEL,
  HUM_NOT_EXIT,
}HUM_WARN_TYPE_E;

/* five dp points*/
typedef enum {
  DPID_TEMP_CURRENT = 1,
  DPID_HUMIDITY_CURRENT = 2,
  DPID_TEMP_WARN = 14,
  DPID_HUMIDITY_WARN = 15,
  DPID_AIR_PRESSURE = 16,
}DPID_E;

/* current device dp values */
short temperature = 0;
short humidity = 0;
short air_pressure = 0;
TEMP_WARN_TYPE_E temp_warn_type;
HUM_WARN_TYPE_E hum_warn_type;

/* temp critial */
#define UPPER_TEMP  60
#define LOWER_TEMP  0

/* humidity critial */
#define UPPER_HUM   70
#define LOWER_HUM   20

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 * dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] =
{
  {DPID_TEMP_CURRENT, DP_TYPE_VALUE},
  {DPID_HUMIDITY_CURRENT, DP_TYPE_VALUE},
  {DPID_TEMP_WARN, DP_TYPE_ENUM},
  {DPID_HUMIDITY_WARN, DP_TYPE_ENUM},
  {DPID_AIR_PRESSURE, DP_TYPE_VALUE},
};

/* product ID */
unsigned char pid[] = {"djx1qnsr"};
/* mcu version */
unsigned char mcu_ver[] = {"1.0.0"};

/* last time */
unsigned long last_time = 0;

/* last time data */
unsigned long last_upload_time = 0;

/* last temp */
unsigned short last_temp = 0;

/* last hum */
unsigned short last_hum = 0;

/* last air press */
unsigned short last_air_press = 0; 

/* last temp warn type */
TEMP_WARN_TYPE_E last_temp_warn_type = TEMP_NOT_EXIT;

/* last hum warn type */
HUM_WARN_TYPE_E last_hum_warn_type = HUM_NOT_EXIT;

void setup() 
{
  Serial.begin(9600);

  /* iic init */
  Wire.begin();
  /* nitialize the air pressure sensor */
  bmp280.init(); 
  /* Temperature and humidity sensor initialization */
  dht.begin();
  
  //Initialize led port, turn off led.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* pin mode set */
  pinMode(KEY_PIN, INPUT_PULLUP);

  //Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  //incoming all DPs and their types array, DP numbers
  my_device.set_dp_cmd_total(dp_array, 5);
  //register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  //register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);
  /* Record start time */
  last_time = millis();
}

void loop() 
{
  /* init uart */
  my_device.uart_service();
  /* long press time */
  if (digitalRead(KEY_PIN) == HIGH) {
      delay(20);
    if (digitalRead(KEY_PIN) == HIGH) {
      /* ble work states reset */
      my_device.mcu_reset_ble();
    }
  }
  
  /* LED blinks when network is being connected */
  if ((BLE_CONNECTED != my_device.mcu_get_ble_work_state()) && (BLE_SATE_UNKNOW != my_device.mcu_get_ble_work_state())) {
    if (LED_BLINK_INTERVAL <= (millis()- last_time)) {
      last_time = millis();
      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_PIN, led_state);
    }
  }else if(BLE_CONNECTED == my_device.mcu_get_ble_work_state()){
    digitalWrite(LED_PIN, HIGH);
  }
  
  /* get the temperature、humidity and air pressure value */
  get_temp_humidity_air();

  /* get temperature warn state */
  get_temp_warn_states();
  
  /* get humidity warn state */
  get_hum_warn_states();

  /* report all dp data */
  if (BLE_CONNECTED == my_device.mcu_get_ble_work_state() && UPLOAD_INTERVAL <  (millis() - last_upload_time)) {
    last_upload_time = millis();
    dp_update_all();
  }

  delay(10);

}

/**
* @Function: get_temp_humidity_air
* @Description: Obtain temperature, humidity, and air pressure values
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void get_temp_humidity_air(void)
{
  float temp_hum_val[2] = {0};
  if(!dht.readTempAndHumidity(temp_hum_val)) {
    humidity = temp_hum_val[0];
    temperature = temp_hum_val[1];
  }
  
  /* Pressure units convert to thousands of psi */
  air_pressure = bmp280.getPressure()/1000;
}

/**
* @Function: get_temp_warn_states
* @Description: Obtain temperature warning DP data
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void get_temp_warn_states(void)
{
  if(UPPER_TEMP < temperature){
    temp_warn_type = TEMP_UPPER;
  } else if(LOWER_TEMP > temperature){
    temp_warn_type = TEMP_LOWER;
  } else {
    temp_warn_type = TEMP_CANCEL;
  }
}

/**
* @Function: get_hum_warn_states
* @Description: Obtain humidity warning DP data
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void get_hum_warn_states(void)
{
  if(UPPER_HUM < humidity){
    hum_warn_type = HUM_UPPER;
  } else if(LOWER_HUM > humidity){
    hum_warn_type = HUM_LOWER;
  } else {
    hum_warn_type = HUM_CANCEL;
  }
}

/**
* @Function: dp_process
* @Description: 
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void dp_process(void)
{
  return;
}

/**
* @Function: dp_update_all
* @Description: Upload all DP status of the current device
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void dp_update_all(void)
{
  if(temperature != last_temp){ 
    my_device.mcu_dp_update(DPID_TEMP_CURRENT, temperature*10, 1);
    last_temp = temperature;
  }
  
  if(humidity != last_hum){
    my_device.mcu_dp_update(DPID_HUMIDITY_CURRENT, humidity, 1);
    last_hum = humidity;
  }
  
  if(temp_warn_type != last_temp_warn_type){
    my_device.mcu_dp_update(DPID_TEMP_WARN, temp_warn_type, 1);
    last_temp_warn_type = temp_warn_type;
  }
  
  if(hum_warn_type != last_hum_warn_type){
    my_device.mcu_dp_update(DPID_HUMIDITY_WARN, hum_warn_type, 1);
    last_hum_warn_type = hum_warn_type;
  }

  if(air_pressure != last_air_press){
    my_device.mcu_dp_update(DPID_AIR_PRESSURE, air_pressure, 1);
    last_air_press = air_pressure;
  }
}
