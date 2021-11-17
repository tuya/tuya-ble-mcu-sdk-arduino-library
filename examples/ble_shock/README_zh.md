# 蓝牙震动传感
[English](./README.md) | [中文](./README_zh.md) 


## 功能实现

| 功能     | 说明                                                  |
| -------- | ----------------------------------------------------- |
| 按键     | 配网成功后，短按按键，可重置蓝牙工作状态              |
| 指示灯   | 蓝牙设备上电后，默认500ms闪烁，配对成功后，指示灯常亮 |
| 震动检测 | 检测当前设备是否震动                                  |
| 姿态检测 | 检测当前设备是否倾斜                                  |

注意：

- 默认上电后处于等待配对状态。
- Arduino 中的 默认Serial 串口已被Tuya mcu sdk 接管，请不要对默认Serial（引脚 0 ，1）做任何操作。



## 创建产品 



进入 [Tuya IoT平台](https://iot.tuya.com/?_source=97c44038fafc20e9c8dd5fdb508cc9c2) 创建产品：

![](https://images.tuyacn.com/smart/shiliu_zone/Tuya_Arduino_library/creat_produce1.png)

选择品类，方案：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/11.png)

完善产品信息，点击创建产品：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/2.png)

根据自身需求选择相关功能，点击确认：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/3.png)

选择设备面板

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/4.png)

进入硬件开发，选择 涂鸦标准模组 MCU SDK 开发方式，选择对应的模组：

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/5.png)



## 硬件准备



### MCU烧录工具：

- 支持数据传输的 Micro-USB线



### BLE SoC 主控板（BT3L）

![](https://images.tuyacn.com/goat/20200225/ba81c4a030ae485c8bb4d24097302cc9.jpg)



### Grove Beginner Kit For Arduino 多和一开发板

该例程使用该板的三轴加速度传感（LIS3DHTR）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/seeed_board.png)

## 软件环境准备

### 依赖库安装

![image-20211105154507557](C:\Users\86178\AppData\Roaming\Typora\typora-user-images\image-20211105154507557.png)



安装三轴加速度库（LIS3DHTR）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/lib2.png)



## 软件设计



### 参数说明

#### 结构体

/* Acceleration in three directions */
typedef struct {
  float x_accel;
  float y_accel;
  float z_accel;
}XYZ_DATA_S;

/* The four states of movement */
typedef enum {
  NORMAL = 0,			/*  正常模式*/
  VIBRATION = 1,       /* 震动模式 */
  DROP = 2,				/*  跌落模式*/
  TILT = 3,				  /* 倾斜模式 */
}SHOCK_STATES_E;



### 功能模块说明



#### 串口

初始化串口，涂鸦WIFi模组的通用固件支持115200、9600波特率自适应，所以初始化串口时应将串口波特率初始化为115200或9600。在 `setup()` 中：

Serial.begin(9600);

\```

设置串口通信服务

void loop()

{

  ...

  my_device.uart_service();

  ...

}



#### BLE MCU SDK 

mcu需要将创建的PID和mcu的软件版本号发送给涂鸦云模组。在 `setup()` 中：

定义全局变量

创建BLE MCU 对象

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



#### dp点数据相关

/* Data point define */
#define DPID_SHOCK 1



设备dp点及类型设置数组

unsigned char dp_array[][2] =
{
    {DPID_SHOCK, DP_TYPE_ENUM},
};

void setup()

{

  ...

  my_device.set_dp_cmd_total(dp_array, 1); //dp_array:为存储DP点ID和数据类型的数组， 1：为数组内定义的DP个数

  ...

}



初始化时还需注册DP点下发处理函数和上传所有DP点函数：

unsigned char led_state = 0;

void setup() 

{

...

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





#### 震动传感器(LIS3DHTR)

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



### 函数说明

三轴加速度获取

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



震动模式判断

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



上报所有dp点数据

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



## 烧录



### 硬件连接

注意：编译下载程序到seeed开发板中，注意下载前不要插涂鸦开发板，防止串口互相干扰。

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire1.png)



### 开发板、端口配置

开发板选择Arduino Uno，串口选择对应的端口（我这里是COM3）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire2.png)



### 编译上传

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_temp_hum_press/fire3.png)



## 配网

设备上电后，默认进入配网状态指示灯闪烁。（红框选中的是配网指示灯，绿框选中的为按键，黄框中的为三轴加速度传感器）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/network_config1.png)



打开涂鸦智能app，进入配网界面，配网成功后返回主界面可以看到温湿度气压传感器demo的图标，点击进入控制界面。（配网成功后seeed多和一板上的指示灯常亮）

![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/network_config2.png)



## 功能展示



![](https://images.tuyacn.com/smart/Hardware_Developer/Arduino_Ble/ble_shock/function2.png)

- 震动检测：检测当前设备是否震动
- 倾斜检测：检测当前设备是否倾斜；当当前设备与水平面倾斜角大于30°时会检测到倾斜
- 物体状态历史记录查询：可以在设备后台查看当前设备的历史运动状态



## 技术支持



您可以通过以下方法获得涂鸦的支持:



\- 开发者中心：https://developer.tuya.com

\- 帮助中心: https://support.tuya.com/help

\- 技术支持工单中心: https://service.console.tuya.com 

