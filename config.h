/*
 * @FileName: config.h
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-10-22 10:44:27
 * @LastEditTime: 2021-10-27 19:50:37
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: Tuya mcu sdk Arduino library config file.
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "TuyaDefs.h"

#ifndef SUPPORT_MCU_FIRM_UPDATE
#define BLE_UART_RECV_BUF_LMT 16 //UART data receiving buffer size, can be reduced if the MCU has insufficient RAM
#define BLE_DATA_PROCESS_LMT 24  //UART data processing buffer size, according to the user DP data size, must be greater than 24
#else
#define BLE_UART_RECV_BUF_LMT 128 //UART data receiving buffer size, can be reduced if the MCU has insufficient RAM

/*  Select the appropriate UART data processing buffer size here 
    (select the buffer size based on the size selected by the above MCU firmware upgrade package and whether to turn on the weather service)  */
#define BLE_DATA_PROCESS_LMT 1000 //UART data processing buffer size. If the MCU firmware upgrade is required, the single-packet size is 256, the buffer must be greater than 260, or larger if the weather service is enabled
//#define BLE_DATA_PROCESS_LMT           600             //UART data processing buffer size. If the MCU firmware upgrade is required, the single-packet size is 512, the buffer must be greater than 520, or larger if the weather service is enabled
//#define BLE_DATA_PROCESS_LMT           1200            //UART data processing buffer size. If the MCU firmware upgrade is required, the single-packet size is 1024, the buffer must be greater than 1030, or larger if the weather service is enabled

#endif

#define BLE_UART_SEND_BUF_LMT 48 //According to the user's DP data size, it must be greater than 48

#endif /* __CONFIG_H__ */
