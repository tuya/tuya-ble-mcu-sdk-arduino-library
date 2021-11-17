/*
 * @FileName: TuyaDefs.h 
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-10-21 11:28:40
 * @LastEditTime: 2021-10-29 09:38:21
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: Some necessary constant definitions. 
 */
#ifndef __TUYA_DEFS_H__
#define __TUYA_DEFS_H__

/* Define constant */
#ifndef TY_TRUE
#define TY_TRUE 1
#endif

#ifndef TY_FALSE
#define TY_FALSE 0
#endif

#ifndef TY_NULL
#ifdef __cplusplus
#define TY_NULL    0
#else
#define TY_NULL   ((void *)0)
#endif 
#endif

#ifndef TY_SUCCESS
#define TY_SUCCESS 1
#endif

#ifndef TY_ERROR
#define TY_ERROR 0
#endif

#ifndef TY_INVALID
#define TY_INVALID 0xFF
#endif

#ifndef TY_ENABLE
#define TY_ENABLE 1
#endif

#ifndef TY_DISABLE
#define TY_DISABLE 0
#endif

#define MCU_RX_VER      0x00    //Module send frame protocol version number
#define MCU_TX_VER      0x00    //MCU send frame protocol version number(default)
#define PROTOCOL_HEAD   0x07    //Fixed protocol header length
#define FRAME_FIRST     0x55    //Frame header first byte
#define FRAME_SECOND    0xaa    //Frame header second byte

//=============================================================================
//Byte order of the frame
//=============================================================================
#define HEAD_FIRST          0
#define HEAD_SECOND         1
#define PROTOCOL_VERSION    2
#define FRAME_TYPE          3
#define LENGTH_HIGH         4
#define LENGTH_LOW          5
#define DATA_START          6

//=============================================================================
//Data frame type
//=============================================================================
#define         HEAT_BEAT_CMD                   0                               //Heartbeat package
#define         PRODUCT_INFO_CMD                1                               //Product information
#define         WORK_MODE_CMD                   2                               //Query the module working mode set by the MCU
#define         BLE_STATE_CMD                   3                               //BLE working status
#define         BLE_RESET_CMD                   4                               //Reset BLE
#define         DATA_QUERT_CMD                  6                               //Order send
#define         STATE_UPLOAD_CMD                7                               //Status upload	 
#define         STATE_QUERY_CMD                 8                               //Status query

//=============================================================================
//BLE work status
//=============================================================================
#define         BLE_UN_BIND                     0x00
#define         BLE_NOT_CONNECTED               0x01
#define         BLE_CONNECTED                   0x02
#define         BLE_SATE_UNKNOW                 0xff
//=============================================================================
//BLE reset status
//=============================================================================
#define         RESET_BLE_ERROR                0
#define         RESET_BLE_SUCCESS              1

//=============================================================================
//dp data point type
//=============================================================================
#define         DP_TYPE_RAW                     0x00        //RAW type
#define         DP_TYPE_BOOL                    0x01        //bool type
#define         DP_TYPE_VALUE                   0x02        //value type
#define         DP_TYPE_STRING                  0x03        //string type
#define         DP_TYPE_ENUM                    0x04        //enum type
#define         DP_TYPE_BITMAP                  0x05        //fault type


#endif /* __TUYA_DEFS_H__ */