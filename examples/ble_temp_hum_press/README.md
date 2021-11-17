# BLE_TEMP_HUM_PRESS
[English](./README.md) | [中文](./README_zh.md) 


## Function Description

| Function          | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| Button            | After the network configuration is successful, short press the button to reset the Bluetooth working state (seeed D6 pin of one board) |
| Indicator light   | After the Bluetooth device is powered on, it flashes for 500ms by default. After the pairing is successful, the indicator light is always on (seeed more and one board D4 pin) |
| Temperature       | The panel can display the current temperature                |
| Humidity          | The panel can display the current humidity                   |
| Air pressure      | The panel can display the current air pressure               |
| Temperature alarm | High temperature alarm is triggered when the temperature is higher than 60 degrees, low temperature alarm is triggered when the temperature is lower than 0 degrees |
| Humidity alarm    | High humidity alarm is triggered when humidity is higher than 70, low humidity alarm is triggered when temperature is lower than 20 |

Notice：

- Waiting for pairing after power-on by default。

- The default Serial port in Arduino has been taken over by Tuya mcu sdk, please do not do any operation on the default Serial (pin 0, 1).

  

## Create a product

enter [Tuya IoT platform](https://iot.tuya.com/?_source=97c44038fafc20e9c8dd5fdb508cc9c2) Create a product：

![](https://images.tuyacn.com/smart/shiliu_zone/Tuya_Arduino_library/creat_produce1.png)



Choose category, plan

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/1.png)



Complete product information, click to create a product：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/3.png)



Select the relevant function according to your own needs, click to confirm：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/dp_point_choose.png)



After selecting the device panel, enter the hardware development, select Tuya standard module MCU SDK development method, and select the corresponding module：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/hardware.png)



## Hardware preparation

### MCU burning tool

- Micro-USB cable supporting data transmission



### BLE MCU communication board (BT3L)

![](https://images.tuyacn.com/goat/20200225/ba81c4a030ae485c8bb4d24097302cc9.jpg)



### Grove Beginner Kit For Arduino Multi-One Development Board

This routine uses the temperature and humidity sensor (DHT11) and air pressure sensor (BMP280) of the board

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/seeed_board.png)

## Software environment preparation



### Dependent library installation![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/soft1.png)



Install temperature and humidity sensor library (DHT)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/lib1.png)



Install air pressure sensor library (BMP280)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/lib2.png)



## software design



### Structure description

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



/* High temperature and low temperature threshold (adjustable) */
#define UPPER_TEMP  60
#define LOWER_TEMP  0

/* High humidity and low humidity threshold (adjustable) */
#define UPPER_HUM   70
#define LOWER_HUM   20



### Function module description

#### Serial port

Initialize the serial port, the general firmware of Tuya WIFi module supports 115200, 9600 baud rate self-adaption, so the serial port baud rate should be initialized to 115200 or 9600 when initializing the serial port. In `setup()`:void setup()

{

...

Serial.begin(9600);

...

}



Set up serial communication service



void loop()

{

  ...

  my_device.uart_service();

  ...

}



#### BLE MCU SDK 

The mcu needs to send the created PID and the software version number of the mcu to the Tuya Cloud module.



Create BLE MCU object

TuyaBLE my_device;

/* product ID */

unsigned char pid[] = {"tbvsjw1f"};

/* mcu version */

unsigned char mcu_ver[] = {"1.0.0"};



void setup()

{ 

...

/* Enter the PID and MCU software version */

my_device.init(pid, mcu_ver);

...

}



#### dp data related

/* five dp points*/
typedef enum {
  DPID_TEMP_CURRENT = 1,
  DPID_HUMIDITY_CURRENT = 2,
  DPID_TEMP_WARN = 14,
  DPID_HUMIDITY_WARN = 15,
  DPID_AIR_PRESSURE = 16,
}DPID_E;



Device dp point and type setting array

unsigned char dp_array[][2] =
{
  {DPID_TEMP_CURRENT, DP_TYPE_VALUE},
  {DPID_HUMIDITY_CURRENT, DP_TYPE_VALUE},
  {DPID_TEMP_WARN, DP_TYPE_ENUM},
  {DPID_HUMIDITY_WARN, DP_TYPE_ENUM},
  {DPID_AIR_PRESSURE, DP_TYPE_VALUE},
};



void setup()

{

  ...

  my_device.set_dp_cmd_total(dp_array, 5); 

 //register DP download processing callback function

 my_device.dp_process_func_register(dp_process); 

 //register upload all DP callback function

 my_device.dp_update_all_func_register(dp_update_all);

  ...

}



#### button

/* button pin */

\#define KEY_PIN  6



void setup()

{

...

 /* pin mode set */

 pinMode(KEY_PIN, INPUT_PULLUP);

...

}



void loop() 

{

...

 if (digitalRead(KEY_PIN) == HIGH) {  

   delay(20);  

  if (digitalRead(KEY_PIN) == HIGH) {

   /* ble work states reset */

   my_device.mcu_reset_ble();

  }

 }

...

}



#### LED indicator



/* Flashing time interval */

\#define LED_BLINK_INTERVAL  500  



void setup()

{

...

 //Initialize led port, turn off led.

 pinMode(LED_BUILTIN, OUTPUT);

 digitalWrite(LED_BUILTIN, LOW);

...

}



void loop() 

{

...

/* LED blinks when network is being connected */

 if ((BLE_CONNECTED != my_device.mcu_get_ble_work_state()) && (BLE_SATE_UNKNOW != my_device.mcu_get_ble_work_state())) {

  if (LED_BLINK_INTERVAL <= (millis()- last_time)) {

   last_time = millis();

   if (led_state == LOW) {

​    led_state = HIGH;

   } else {

​    led_state = LOW;

   }

   digitalWrite(LED_BUILTIN, led_state);

  }

 }

...

}



#### Temperature and humidity sensor (DHT11)

#include <DHT.h>



/* air pressure sensor choose and init*/

\#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);



void setup() 
{

...

  /* iic init */
  Wire.begin();
  dht.begin();

...

}



#### Air pressure sensor (BMP280)

#include <Seeed_BMP280.h>
#include <Wire.h>



BMP280 bmp280;



void setup() 
{

...

  /* iic init */
  Wire.begin();
  bmp280.init();

...

}



### Function description

Get temperature, humidity, and air pressure values

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
  



Get the temperature warning sign

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



Obtain the humidity warning label

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



Analyze all dp point data. The dp point data type in this demo is report only, no data can be sent, so there is no need to analyze the sent data

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



## Burn

### Hardware connection

Note: Compile and download the program to the seeed development board. Please do not insert the Tuya development board before downloading to prevent the serial ports from interfering with each other.

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire1.png)



### Software configuration

#### Development board, port configuration

Choose Arduino Uno for the development board, and choose the corresponding port for the serial port (COM3 here)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire2.png)



#### Compile and upload

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire3.png)



## Distribution network

After the device is powered on, it defaults to the network configuration status indicator blinking. (The red box selects the network indicator, the green box selects the button, the yellow box is the air pressure sensor, and the purple box is the temperature, humidity and air pressure sensor)![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/hard_ware_config.png)



Open the Tuya smart app and enter the network configuration interface. After the network configuration is successful, return to the main interface and you can see the icon of the temperature, humidity and pressure sensor demo, click to enter the control interface. (After the network configuration is successful, the indicator on the board of Seeed Duohe is always on)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/network_connect.png)



## function display

- Temperature display: display real-time temperature

- Humidity display: display real-time humidity

- Air pressure display: display real-time air pressure

- Temperature alarm: when the temperature is lower than 0℃, the temperature lower limit alarm is triggered; when the temperature is higher than 60℃, the temperature upper limit alarm is triggered

- Humidity alarm: when the reading is lower than 20%, the lower humidity alarm will be triggered; when the humidity is higher than 70%, the humidity upper limit alarm will be triggered

  ![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/function.png)



## Technical support

You can get support for Tuya by using the following methods:

- Developer Centre: https://developer.tuya.com
- Help Centre: https://support.tuya.com/help
- Technical Support Work Order Centre: https://service.console.tuya.com 