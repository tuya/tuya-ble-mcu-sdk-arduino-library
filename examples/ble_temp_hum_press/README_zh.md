# 蓝牙温湿度气压传感
[English](./README.md) | [中文](./README_zh.md) 


## 功能说明 

| 功能     | 说明                                                         |
| -------- | ------------------------------------------------------------ |
| 按键     | 配网成功后，短按按键，可重置蓝牙工作状态(seeed多和一板D6引脚) |
| 指示灯   | 蓝牙设备上电后，默认500ms闪烁，配对成功后，指示灯常亮（seeed多和一板D4引脚） |
| 温度     | 面板可显示当前温度                                           |
| 湿度     | 面板可显示当前湿度                                           |
| 气压     | 面板可显示当前气压                                           |
| 温度报警 | 当温度高于60度时触发高温报警，当温度低于0度时触发低温报警    |
| 湿度报警 | 当湿度高于70是触发高湿报警，当温度低于20时触发低湿报警       |

注意：

- 默认上电后处于等待配对状态。

- Arduino 中的 默认Serial 串口已被Tuya mcu sdk 接管，请不要对默认Serial（引脚 0 ，1）做任何操作。

  

## 创建产品 



进入 [Tuya IoT平台](https://iot.tuya.com/?_source=97c44038fafc20e9c8dd5fdb508cc9c2) 创建产品：

![](https://images.tuyacn.com/smart/shiliu_zone/Tuya_Arduino_library/creat_produce1.png)



选择品类，方案：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/1.png)



完善产品信息，点击创建产品：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/3.png)



根据自身需求选择相关功能，点击确认：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/dp_point_choose.png)



选择设备面板后，进入硬件开发，选择 涂鸦标准模组 MCU SDK 开发方式，选择对应的模组：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/hardware.png)

## 硬件准备



### MCU烧录工具

- 支持数据传输的 Micro-USB线





### BLE MCU 通信板（BT3L）

![](https://images.tuyacn.com/goat/20200225/ba81c4a030ae485c8bb4d24097302cc9.jpg)



### Grove Beginner Kit For Arduino 多和一开发板

该例程使用该板的温湿度传感器（DHT11）、气压传感器（BMP280）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/seeed_board.png)

## 软件环境准备

### 依赖库安装

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/soft1.png)



安装温湿度传感库（DHT）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/lib1.png)



安装气压传感库（BMP280）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/lib2.png)



## 软件设计



### 结构体说明

/* have three temperature warning types */
typedef enum {
  TEMP_LOWER,         //温度下限报警
  TEMP_UPPER,		  //温度上限报警
  TEMP_CANCEL,	    //正常
  TEMP_NOT_EXIT,     //不存在
}TEMP_WARN_TYPE_E;



/* have three humitudy warning types */
typedef enum {
  HUM_LOWER,			低湿报警
  HUM_UPPER,			 高湿报警
  HUM_CANCEL,		   正常

  HUM_NOT_EXIT,        不存在
}HUM_WARN_TYPE_E;



/* 高温低温阈值（可调） */
#define UPPER_TEMP  60
#define LOWER_TEMP  0

/* 高湿低湿阈值（可调） */
#define UPPER_HUM   70
#define LOWER_HUM   20



### 功能模块说明



#### 串口

初始化串口，涂鸦WIFi模组的通用固件支持115200、9600波特率自适应，所以初始化串口时应将串口波特率初始化为115200或9600。在 `setup()` 中：

void setup()

{

...

Serial.begin(9600);

...

}



设置串口通信服务

void loop()

{

  ...

  my_device.uart_service();

  ...

}



#### BLE MCU SDK 

mcu需要将创建的PID和mcu的软件版本号发送给涂鸦云模组。



创建BLE MCU 对象

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



#### dp数据相关

/* five dp points*/
typedef enum {
  DPID_TEMP_CURRENT = 1,
  DPID_HUMIDITY_CURRENT = 2,
  DPID_TEMP_WARN = 14,
  DPID_HUMIDITY_WARN = 15,
  DPID_AIR_PRESSURE = 16,
}DPID_E;



设备dp点及类型设置数组

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

  my_device.set_dp_cmd_total(dp_array, 5); //dp_array:为存储DP点ID和数据类型的数组， 5：为数组内定义的DP个数

 //register DP download processing callback function

 my_device.dp_process_func_register(dp_process); //注册DP下发处理函数

 //register upload all DP callback function

 my_device.dp_update_all_func_register(dp_update_all);//注册设备状态函数，上传所有DP点

  ...

}



#### 按键

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

   delay(20);  长按20ms以上触发重置配网

  if (digitalRead(KEY_PIN) == HIGH) {

   /* ble work states reset */

   my_device.mcu_reset_ble();

  }

 }

...

}



#### LED指示灯



/* 闪烁时间间隔 */

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



#### 温湿度传感(DHT11)

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



#### 气压传感器(BMP280)

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



### 函数说明

获取温度、湿度、气压值

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
  



获取温度警告标识

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



获取湿度警告标识

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



解析所有dp点数据，该demo中dp点数据类型都为只上报，无可下发所以不用进行下发数据的解析

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



## 烧录

### 硬件连接

注意：编译下载程序到seeed开发板中，注意下载前不要插涂鸦开发板，防止串口互相干扰。

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire1.png)



### 软件配置

#### 开发板、端口配置

开发板选择Arduino Uno，串口选择对应的端口（我这里是COM3）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire2.png)



#### 编译上传

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire3.png)



## 配网

设备上电后，默认进入配网状态指示灯闪烁。（红框选中的是配网指示灯，绿框选中的为按键，黄框中的为气压传感器，紫框中的为温湿度气压传感器）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/hard_ware_config.png)



打开涂鸦智能app，进入配网界面，配网成功后返回主界面可以看到温湿度气压传感器demo的图标，点击进入控制界面。（配网成功后seeed多和一板上的指示灯常亮）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/network_connect.png)



## 功能展示



- 温度显示：显示实时温度

- 湿度显示：显示实时湿度

- 气压显示：显示实时气压

- 温度报警：当温度低于0℃时，触发温度下限报警；当温度高于60℃时触发温度上限报警

- 湿度报警：当时读低于20%时，触发湿度下限报警；当湿度高于70%时触发湿度上限报警

  ![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/function.png)



## 技术支持



您可以通过以下方法获得涂鸦的支持:



\- 开发者中心：https://developer.tuya.com

\- 帮助中心: https://support.tuya.com/help

\- 技术支持工单中心: https://service.console.tuya.com 