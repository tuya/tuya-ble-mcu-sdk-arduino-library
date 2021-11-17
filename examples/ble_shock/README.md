# BLE_SHOCK
[English](./README.md) | [中文](./README_zh.md) 

## Function Description

| Function            | Description                                                  |
| ------------------- | ------------------------------------------------------------ |
| Button              | After the network configuration is successful, short press the button to reset the Bluetooth working status |
| Indicator light     | After the Bluetooth device is powered on, it flashes for 500ms by default. After the pairing is successful, the indicator light is always on |
| Vibration detection | Detect whether the current device is vibrating               |
| Posture detection   | Check whether the current device is tilted                   |

Notice：

- Waiting for pairing after power-on by default。

- The default Serial port in Arduino has been taken over by Tuya mcu sdk, please do not do any operation on the default Serial (pin 0, 1).

## Create a product



enter [Tuya IoT platform](https://iot.tuya.com/?_source=97c44038fafc20e9c8dd5fdb508cc9c2) Create a product：

![](https://images.tuyacn.com/smart/shiliu_zone/Tuya_Arduino_library/creat_produce1.png)

Choose category, plan

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/11.png)

Complete product information, click to create a product：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/2.png)

Select the relevant function according to your own needs, click to confirm：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/3.png)



choose Device panel

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/4.png)



After selecting the device panel, enter the hardware development, select Tuya standard module MCU SDK development method, and select the corresponding module：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/5.png)



## Hardware preparation

### MCU burning tool

- Micro-USB cable supporting data transmission



### BLE MCU communication board (BT3L)

![](https://images.tuyacn.com/goat/20200225/ba81c4a030ae485c8bb4d24097302cc9.jpg)



### Grove Beginner Kit For Arduino Multi-One Development Board

This example uses the board's three-axis acceleration sensor (LIS3DHTR)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/seeed_board.png)

## Software environment preparation

### Dependent library installation

![image-20211105154507557](C:\Users\86178\AppData\Roaming\Typora\typora-user-images\image-20211105154507557.png)



Install the three-axis acceleration library (LIS3DHTR)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/lib2.png)



## software design



### Structure description

/* Acceleration in three directions */
typedef struct {
  float x_accel;
  float y_accel;
  float z_accel;
}XYZ_DATA_S;

/* The four states of movement */
typedef enum {
  NORMAL = 0,
  VIBRATION = 1,
  DROP = 2,
  TILT = 3,
}SHOCK_STATES_E;



### Function module description

#### Serial port

Initialize the serial port, the general firmware of Tuya WIFi module supports 115200, 9600 baud rate self-adaption, so the serial port baud rate should be initialized to 115200 or 9600 when initializing the serial port. In `setup()`:

Serial.begin(9600);

\```

Set up serial communication service

void loop()

{

  ...

  my_device.uart_service();

  ...

}



#### BLE MCU SDK 

The mcu needs to send the created PID and the software version number of the mcu to the Tuya Cloud module.



/* Create BLE MCU object  */

TuyaBLE my_device;



/* product ID */

unsigned char pid[] = {"rf0lakds"};

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

/* Data point define */
#define DPID_SHOCK 1



Device dp point and type setting array

unsigned char dp_array[][2] =
{
    {DPID_SHOCK, DP_TYPE_ENUM},
};

void setup()

{

  ...

  my_device.set_dp_cmd_total(dp_array, 1); 

  ...

}



It is also necessary to register the DP point issuing processing function and upload all DP point functions during initialization:

unsigned char led_state = 0;

void setup() 

{

...

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





#### Vibration sensor (LIS3DHTR)

#include <LIS3DHTR.h>
#include <Wire.h>



LIS3DHTR<TwoWire> LIS; 

/*iic Data reporting address */
#define LIS3DHTR_ADDRESS_UPDATED   (0x19)

void setup() 
{

...

  /* IIC init dafault :0x18 */
  LIS.begin(WIRE, LIS3DHTR_ADDRESS_UPDATED); 
  /* Sampling frequency */
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  /* Range setting */
  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
  /* High solution enable */
  LIS.setHighSolution(true); 

...

}



### Function description

Three-axis acceleration acquisition

/**

\* @Function: get_acceleration_date

\* @Description: get three-axis acceleration data

\* @Input: none

\* @Output: none

\* @Return: none

\* @Others: none

*/

void get_acceleration_date(void)
{
  if (!LIS) {
    Serial.println("LIS3DHTR didn't connect.");
  }
  xyz_data.x_accel = LIS.getAccelerationX()*1000;

  xyz_data.y_accel = LIS.getAccelerationY()*1000;

  xyz_data.z_accel = LIS.getAccelerationZ()*1000;

}



Vibration mode judgment

/**

\* @Function: shock_state_judge

\* @Description: Get the current sensor state

\* @Input: none

\* @Output: none

\* @Return: none

\* @Others: none

*/

void shock_state_judge(void)

{

 /* Pitch angle calculation */

 float pitch = (short)(atan2((0-xyz_data.y_accel),xyz_data.z_accel) * 180 / PI);

 /* Roll angle calculation */

 float roll = (short)(atan2((xyz_data.x_accel),xyz_data.z_accel) * 180 / PI);

  

 if(TILT_ANGLE < abs(pitch) || TILT_ANGLE < abs(roll)){

  shock_state = TILT;    /* titl mode */

 }else if(50 < abs(xyz_data.x_accel) || 50 < abs(xyz_data.y_accel) || 80 < (abs(xyz_data.z_accel) - 1000)){

  shock_state = VIBRATION;   /* shock mode */

 } else{

  shock_state = NORMAL; /* shock mode */

 }

 

 last_z_accel = xyz_data.z_accel;

 delay(50);

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



Report all dp point data

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
  my_device.mcu_dp_update(DPID_SHOCK, shock_state, 1);
}

\```



## Burn

### Hardware connection

Note: Compile and download the program to the seeed development board. Please do not insert the Tuya development board before downloading to prevent the serial ports from interfering with each other.

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire1.png)



### Software configuration

#### Development board, port configuration

Choose Arduino Uno for the development board, and choose the corresponding port for the serial port (COM3 here)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire2.png)



### Compile and upload

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire3.png)



## Distribution network

After the device is powered on, it defaults to the network configuration status indicator blinking. (The red box selects the network indicator, the green box selects the button, and the yellow box selects the three-axis acceleration sensor)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/network_config1.png)



Open the Tuya smart app and enter the network configuration interface. After the network configuration is successful, return to the main interface and you can see the icon of the temperature, humidity and pressure sensor demo, click to enter the control interface. (After the network configuration is successful, the indicator on the board of Seeed Duohe is always on)

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/network_config2.png)



## function display



![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/function2.png)

- Vibration detection: detect whether the current device is vibrating
- Tilt detection: detect whether the current device is tilted; when the tilt angle of the current device to the horizontal plane is greater than 30°, the tilt will be detected
- Object status historical record query: you can view the current historical motion status of the device in the background of the device



## Technical support

You can get support for Tuya by using the following methods:

- Developer Centre: https://developer.tuya.com
- Help Centre: https://support.tuya.com/help
- Technical Support Work Order Centre: https://service.console.tuya.com 

