/*
 * @FileName: BLE_SHOCK.ino
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
#include <LIS3DHTR.h>
#include <Wire.h>

#define WIRE Wire
LIS3DHTR<TwoWire> LIS; 

/*iic Data reporting address */
#define LIS3DHTR_ADDRESS_UPDATED   (0x19)

TuyaBLE my_device;

/* Tilt angle threshold */
#define TILT_ANGLE   30    

/* Current LED status */
unsigned char led_state = 0;

/* Data point define */
#define DPID_SHOCK 1

/* data upload interval(ms) */
#define UPLOAD_INTERVAL  300  

#define LED_PIN  4

/* button pin */
#define KEY_PIN  6

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 * dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] =
{
    {DPID_SHOCK, DP_TYPE_ENUM},
};

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

/* State the vibration state */
SHOCK_STATES_E shock_state;
/* Declare the acceleration structure */
XYZ_DATA_S xyz_data;

/* product ID */
unsigned char pid[] = {"rf0lakds"};
/* mcu version */
unsigned char mcu_ver[] = {"1.0.0"};

/* last time */
unsigned long last_time = 0;

/* last upload time */
unsigned long last_upload_time = 0;

/* The last z-axis acceleration was used to check drop */
float last_z_accel = 10.0;

/* last state */
unsigned char last_states = 5;

void setup() 
{
  /* Serial port baud rate setting */
  Serial.begin(9600);
  /* IIC init dafault :0x18 */
  LIS.begin(WIRE, LIS3DHTR_ADDRESS_UPDATED); 
  /* Sampling frequency */
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  /* Range setting */
  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
  /* High solution enable */
  LIS.setHighSolution(true); 
  
  /*Initialize led port, turn off led.*/
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* pin mode set */
  pinMode(KEY_PIN, INPUT_PULLUP);

  /*Enter the PID and MCU software version*/
  my_device.init(pid, mcu_ver);
  /*incoming all DPs and their types array, DP numbers*/
  my_device.set_dp_cmd_total(dp_array, 1);
  /*register DP download processing callback function*/
  my_device.dp_process_func_register(dp_process);
  /*register upload all DP callback function*/
  my_device.dp_update_all_func_register(dp_update_all);
  /* Record start time */
  last_time = millis();
}

void loop() 
{
  /* init uart */
  my_device.uart_service();

  /* long press time three seconeds */
  if(digitalRead(KEY_PIN) == HIGH) {
    delay(80);
    if (digitalRead(KEY_PIN) == HIGH) {
      /* ble work states reset */
      my_device.mcu_reset_ble();
      //power_flag = false;
    }
  }
 
  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_ble_work_state() != BLE_CONNECTED) && (my_device.mcu_get_ble_work_state() != BLE_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
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

  /* report all dp data (300ms interval) */
  if (BLE_CONNECTED == my_device.mcu_get_ble_work_state() && UPLOAD_INTERVAL <  (millis() - last_upload_time) && last_states != shock_state) {
    last_states = shock_state;
    last_upload_time = millis();
    dp_update_all();
  }
  
  /* Get the acceleration in x, y, and z directions */
  get_acceleration_date();

  /* shock states judge */
  shock_state_judge();
  delay(10);
}

void get_acceleration_date(void)
{
  xyz_data.x_accel = LIS.getAccelerationX()*1000;
  delay(50);
  
  xyz_data.y_accel = LIS.getAccelerationY()*1000;
  delay(50);
  
  xyz_data.z_accel = LIS.getAccelerationZ()*1000;
  delay(50);
}

/**
* @Function: shock_state_judge
* @Description: Get the current sensor state
* @Input: none
* @Output: none
* @Return: none
* @Others: none
*/
void shock_state_judge(void)
{
  /* Pitch angle calculation */
  float pitch = (short)(atan2((0-xyz_data.y_accel),xyz_data.z_accel) * 180 / PI);
  /* Roll angle calculation */
  float roll = (short)(atan2((xyz_data.x_accel),xyz_data.z_accel) * 180 / PI);
    
  if(TILT_ANGLE < abs(pitch) || TILT_ANGLE < abs(roll)){
    shock_state = TILT;      /* titl mode */
  }else if(50 < abs(xyz_data.x_accel) || 50 < abs(xyz_data.y_accel) || 80 < (abs(xyz_data.z_accel) - 1000)){
    shock_state = VIBRATION;     /* shock mode */
  } else{
    shock_state = NORMAL; /* shock mode */
  }
  
  last_z_accel = xyz_data.z_accel;
  delay(50);
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
  my_device.mcu_dp_update(DPID_SHOCK, shock_state, 1);
}
