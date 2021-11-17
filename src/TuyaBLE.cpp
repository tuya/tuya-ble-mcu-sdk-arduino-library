/*
 * @FileName: Tuya.cpp
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-10-21 20:35:17
 * @LastEditTime: 2021-10-29 09:38:21
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: The functions that the user needs to actively call are in this file.
 */

#define TUYA_GLOBAL

#include <Arduino.h>
#include <TuyaBLE.h>
#include "TuyaTools.h"
#include "TuyaDataPoint.h"

TuyaTools tuya_tools;
TuyaUart tuya_uart;
TuyaDataPoint tuya_dp;

/* Protocol serial port initialization */
TuyaBLE::TuyaBLE(void)
{
    tuya_uart.set_serial(&Serial);
}

TuyaBLE::TuyaBLE(HardwareSerial *serial)
{
    tuya_uart.set_serial(serial);
}

TuyaBLE::TuyaBLE(SoftwareSerial *serial)
{
    tuya_uart.set_serial(serial);
}

/**
 * @description: Initialize product information
 * @param {unsigned char} *pid : Product ID(Create products on the Tuya IoT platform to get)
 * @param {unsigned char} *mcu_ver : MCU Software Version Number
 * @return {*}
 */
unsigned char TuyaBLE::init(unsigned char *pid, unsigned char *mcu_ver)
{
    if (pid == TY_NULL || mcu_ver == TY_NULL)
    {
        return TY_ERROR;
    }

    if (tuya_tools.my_strlen(pid) <= PID_LEN)
    {
        tuya_tools.my_memcpy(product_id, pid, tuya_tools.my_strlen(pid));
    }
    else
    {
        tuya_tools.my_memcpy(product_id, pid, PID_LEN);
        return TY_ERROR;
    }

    if (tuya_tools.my_strlen(mcu_ver) <= VER_LEN)
    {
        tuya_tools.my_memcpy(mcu_ver_value, mcu_ver, tuya_tools.my_strlen(mcu_ver));
    }
    else
    {
        tuya_tools.my_memcpy(mcu_ver_value, mcu_ver, VER_LEN);
        return TY_ERROR;
    }

    return TY_SUCCESS;
}

/**
 * @description: BLE serial port processing service
 * @param {*}
 * @return {*}
 */
void TuyaBLE::uart_service(void)
{
    unsigned char ret;
    static unsigned short rx_in = 0;
    unsigned short offset = 0;
    unsigned short rx_value_len = 0;

    /* extract serial data */
    while(tuya_uart.available()) {
        ret = tuya_uart.uart_receive_input(tuya_uart.read());
        if (ret != TY_SUCCESS) {
            break;
        }
    }

    while ((rx_in < sizeof(tuya_uart.ble_data_process_buf)) && tuya_uart.with_data_rxbuff() > 0)
    {
        tuya_uart.ble_data_process_buf[rx_in++] = tuya_uart.take_byte_rxbuff();
    }

    if (rx_in < PROTOCOL_HEAD)
        return;

    while ((rx_in - offset) >= PROTOCOL_HEAD)
    {
        if (tuya_uart.ble_data_process_buf[offset + HEAD_FIRST] != FRAME_FIRST)
        {
            offset++;
            continue;
        }

        if (tuya_uart.ble_data_process_buf[offset + HEAD_SECOND] != FRAME_SECOND)
        {
            offset++;
            continue;
        }

        if (tuya_uart.ble_data_process_buf[offset + PROTOCOL_VERSION] != MCU_RX_VER)
        {
            offset += 2;
            continue;
        }

        rx_value_len = tuya_uart.ble_data_process_buf[offset + LENGTH_HIGH] * 0x100;
        rx_value_len += (tuya_uart.ble_data_process_buf[offset + LENGTH_LOW] + PROTOCOL_HEAD);
        if (rx_value_len > sizeof(tuya_uart.ble_data_process_buf) + PROTOCOL_HEAD)
        {
            offset += 3;
            continue;
        }

        if ((rx_in - offset) < rx_value_len)
        {
            break;
        }

        //数据接收完成
        if (tuya_tools.get_check_sum((unsigned char *)tuya_uart.ble_data_process_buf + offset, rx_value_len - 1) != tuya_uart.ble_data_process_buf[offset + rx_value_len - 1])
        {
            //校验出错
            //printf("crc error (crc:0x%X  but data:0x%X)\r\n",get_check_sum((unsigned char *)ble_data_process_buf + offset,rx_value_len - 1),ble_data_process_buf[offset + rx_value_len - 1]);
            offset += 3;
            continue;
        }

        data_handle(offset);
        offset += rx_value_len;
    } //end while

    rx_in -= offset;
    if (rx_in > 0)
    {
        tuya_tools.my_memcpy((char *)tuya_uart.ble_data_process_buf, (const char *)tuya_uart.ble_data_process_buf + offset, rx_in);
    }
}

/**
 * @description: Data frame processing
 * @param {unsigned short} offset : Data start position
 * @return {*}
 */
void TuyaBLE::data_handle(unsigned short offset)
{
#ifdef SUPPORT_MCU_FIRM_UPDATE
    unsigned char *firmware_addr = TY_NULL;
    static unsigned short firm_size;           //Upgrade package size
    static unsigned long firm_length;          //MCU upgrade file length
    static unsigned char firm_update_flag = 0; //MCU upgrade flag
    unsigned long dp_len;
    unsigned char firm_flag; //Upgrade package size flag
#else
    unsigned short dp_len;
#endif

    unsigned char ret;
    unsigned short i, total_len;
    unsigned char cmd_type = tuya_uart.ble_data_process_buf[offset + FRAME_TYPE];
    
    signed char ble_rssi;

#ifdef FILE_DOWNLOAD_ENABLE
    unsigned char *file_data_addr = TY_NULL;
    static unsigned short file_package_size = 0; //File packet size
    static unsigned char file_download_flag = 0; //File download flag
    unsigned int file_download_size = 0;
#endif
    
    switch (cmd_type)
    {
    case HEAT_BEAT_CMD: //Heartbeat package
        heat_beat_check();
        break;

    case PRODUCT_INFO_CMD: //Product information
        product_info_update();
        break;

    case WORK_MODE_CMD: //Query the module working mode set by the MCU
        get_mcu_ble_mode();
        break;

#ifndef BLE_CONTROL_SELF_MODE
    case BLE_STATE_CMD: //BLE working status
        ble_work_state = tuya_uart.ble_data_process_buf[offset + DATA_START];
        tuya_uart.ble_uart_write_frame(BLE_STATE_CMD, MCU_TX_VER, 0);
        break;

    case BLE_RESET_CMD: //Reset ble
        reset_ble_flag = RESET_BLE_SUCCESS;
        break;

#endif

    case DATA_QUERT_CMD: //Order send
        total_len = (tuya_uart.ble_data_process_buf[offset + LENGTH_HIGH] << 8) | tuya_uart.ble_data_process_buf[offset + LENGTH_LOW];

        for (i = 0; i < total_len;)
        {
            dp_len = tuya_uart.ble_data_process_buf[offset + DATA_START + i + 2] * 0x100;
            dp_len += tuya_uart.ble_data_process_buf[offset + DATA_START + i + 3];
            //
            ret = data_point_handle((unsigned char *)tuya_uart.ble_data_process_buf + offset + DATA_START + i);

            if (TY_SUCCESS == ret)
            {
                //Send success
            }
            else
            {
                //Send fault
            }

            i += (dp_len + 4);
        }
        break;

    case STATE_QUERY_CMD: //Status query
        all_data_update();
        break;


    default:
        break;
    }
}


/**
 * @description: Input product All DP ID, Type, total number of DPs
 * @param {unsigned char} dp_cmd_array : DP array. array[][0] : DP ID, 
 *                                                 array[][1] : DP Type(DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP)
 * @param {unsigned char} dp_cmd_num : total number of DPs
 * @return {*}
 */
void TuyaBLE::set_dp_cmd_total(unsigned char dp_cmd_array[][2], unsigned char dp_cmd_num)
{
    download_dp_number = dp_cmd_num;
    download_cmd = dp_cmd_array;
}

/**
 * @description: Get the serial number of the DPID in the array.
 * @param {unsigned char} dpid
 * @return {unsigned char} index : The index of the input dpid in the array
 */
unsigned char TuyaBLE::get_dowmload_dpid_index(unsigned char dpid)
{
    unsigned char index;

    for (index = 0; index < download_dp_number; index++)
    {
        if (download_cmd[index][0] == dpid)
        {
            break;
        }
    }
    return index;
}

/**
 * @description: Delivery data processing
 * @param {const unsigned char} value : Send data source pointer
 * @return Return data processing result
 */
unsigned char TuyaBLE::data_point_handle(const unsigned char value[])
{
    unsigned char dp_id, index;
    unsigned char dp_type;
    unsigned char ret;
    unsigned short dp_len;

    dp_id = value[0];
    dp_type = value[1];
    dp_len = value[2] * 0x100;
    dp_len += value[3];

    index = get_dowmload_dpid_index(dp_id);

    if (dp_type != download_cmd[index][1])
    {
        //Error message
        return TY_FALSE;
    }
    else
    {
        ret = dp_download_handle(dp_id, value + 4, dp_len);
    }

    return ret;
}

/**
 * @description: DP command processing callback function
 * @param {tuya_callback_dp_download} _func
 * @return {*}
 */
void TuyaBLE::dp_process_func_register(tuya_callback_dp_download _func)
{
    dp_download_handle = _func;
}

/**
 * @description: Reply to the current device status callback function
 * @param {tuya_callback_dp_update_all} _func
 * @return {*}
 */
void TuyaBLE::dp_update_all_func_register(tuya_callback_dp_update_all _func)
{
    all_data_update = _func;
}

/**
 * @description: Heartbeat packet detection
 * @param {*}
 * @return {*}
 */
void TuyaBLE::heat_beat_check(void)
{
    unsigned char length = 0;
    static unsigned char mcu_reset_state = TY_FALSE;

    if (TY_FALSE == mcu_reset_state)
    {
        length = tuya_uart.set_ble_uart_byte(length, TY_FALSE);
        mcu_reset_state = TY_TRUE;
    }
    else
    {
        length = tuya_uart.set_ble_uart_byte(length, TY_TRUE);
    }

    tuya_uart.ble_uart_write_frame(HEAT_BEAT_CMD, MCU_TX_VER, length);
}

/**
 * @description: Product information upload
 * @param {*}
 * @return {*}
 */
void TuyaBLE::product_info_update(void)
{
    unsigned char length = 0;

    length = tuya_uart.set_ble_uart_buffer(length, product_id, PID_LEN);
    length = tuya_uart.set_ble_uart_buffer(length, mcu_ver_value, VER_LEN);

    tuya_uart.ble_uart_write_frame(PRODUCT_INFO_CMD, MCU_TX_VER, length);
}

/**
 * @description: Query the working mode of mcu and ble
 * @param {*}
 * @return {*}
 */
void TuyaBLE::get_mcu_ble_mode(void)
{
    unsigned char length = 0;

#ifdef BLE_CONTROL_SELF_MODE //Module self-processing
    length = tuya_uart.set_ble_uart_byte(length, BLE_STATE_KEY);
    length = tuya_uart.set_ble_uart_byte(length, BLE_RESERT_KEY);
#else
    //No need to process data
#endif

    tuya_uart.ble_uart_write_frame(WORK_MODE_CMD, MCU_TX_VER, length);
}

/**
 * @description: mcu gets bool,value,enum type to send dp value. (raw, string type needs to be handled at the user's discretion. fault only report)
 * @param {unsigned char} dpid : data point ID 
 * @param {const unsigned char} value : dp data buffer address 
 * @param {unsigned short} len : data length
 * @return {unsigned char} Parsed data
 */
unsigned long TuyaBLE::mcu_get_dp_download_data(unsigned char dpid, const unsigned char value[], unsigned short len)
{
    unsigned long ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
    case DP_TYPE_BOOL:
        ret = tuya_dp.mcu_get_dp_download_bool(value, len);
        break;

    case DP_TYPE_VALUE:
        ret = tuya_dp.mcu_get_dp_download_value(value, len);
        break;

    case DP_TYPE_ENUM:
        ret = tuya_dp.mcu_get_dp_download_enum(value, len);
        break;

    default:
        break;
    }
    return ret;
}

/**
 * @description: dp data upload
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} len
 * @return {*}
 */
unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, const unsigned char value[], unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_RAW:
            ret = tuya_dp.mcu_dp_raw_update(dpid, value, len);
        break;

        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, *value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, *value);
        break;

        case DP_TYPE_STRING:
            ret = tuya_dp.mcu_dp_string_update(dpid, value, len);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, *value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, *value);
        break;


        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, unsigned char value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, char value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, unsigned long value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, long value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, unsigned int value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

unsigned char TuyaBLE::mcu_dp_update(unsigned char dpid, int value, unsigned short len)
{
    unsigned char ret;
    switch (download_cmd[get_dowmload_dpid_index(dpid)][1])
    {
        case DP_TYPE_BOOL:
            ret = tuya_dp.mcu_dp_bool_update(dpid, value);
        break;

        case DP_TYPE_ENUM:
            ret = tuya_dp.mcu_dp_enum_update(dpid, value);
        break;

        case DP_TYPE_VALUE:
            ret = tuya_dp.mcu_dp_value_update(dpid, value);
        break;

        case DP_TYPE_BITMAP:
            ret = tuya_dp.mcu_dp_fault_update(dpid, value);
        break;

        default:
            break;
    }
    return ret;
}

/**
 * @description: The MCU actively obtains the current ble working status.
 * @param {*}
 * @return {unsigned char} ble work state
 *                          BLE_UN_BIND:Unbound state
 *                          BLE_UNCONNECT:Bound but not connected
 *                          BLE_CONNECTED:Bound and connected
 * @note   1:If the module is in self-processing mode, the MCU does not need to call this function.
 */
unsigned char TuyaBLE::mcu_get_ble_work_state(void)
{
    return ble_work_state;
}


/**
 * @description: MCU actively resets ble working mode
 * @param {*}
 * @return {*}
 * @note   1:The MCU actively calls to obtain whether the reset ble is successful through the mcu_get_reset_ble_flag() function.
 *         2:If the module is in self-processing mode, the MCU does not need to call this function.
 */
void TuyaBLE::mcu_reset_ble(void)
{
    reset_ble_flag = RESET_BLE_ERROR;
    
    tuya_uart.ble_uart_write_frame(BLE_RESET_CMD, MCU_TX_VER, 0);
}

