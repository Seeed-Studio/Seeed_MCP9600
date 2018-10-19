/*
 * Seeed_MCP9600.cpp
 * Driver for MCP9600
 *  
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Author     : downey
 * Create Time: May 2018
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Seeed_MCP9600.h"


MCP9600::MCP9600(u8 IIC_ADDR)
{
    set_iic_addr(IIC_ADDR);
}


/**@brief set type of thermocouple.read version.
 * @param therm_type
 * @return 0 if successed.
 * */
err_t MCP9600::init(u8 therm_type)
{
    err_t ret=NO_ERROR;
    u16 ver;
    IIC_begin();
    ret=read_version(&ver);
    if(!ret)
    {
        Serial.print("version =");
        Serial.println(ver,HEX);
    }
    if(ret=set_therm_type(therm_type))
    {
        return ret; 
    }
    return ret;
}   



/**@brief read version.
 * @param ver.
 * @return 0 if successed.
 * */
err_t MCP9600::read_version(u16 *ver)
{
    if(IIC_read_16bit(VERSION_ID_REG_ADDR,ver))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief read hot-junction,the temperature result.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_hot_junc(float *value)
{
    *value=0;
    u16 read_value=0;
    if(IIC_read_16bit(HOT_JUNCTION_REG_ADDR,&read_value))
    {
        return ERROR_COMM;
    }
    // Serial.print("read hot junc value=");
    // Serial.println(read_value,HEX);
    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096.0;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}


/**@brief read junction delta.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_junc_temp_delta(float *value)
{
    *value=0;
    u16 read_value=0;
    if(IIC_read_16bit(JUNCTION_TEMP_DELTA_REG_ADDR,&read_value))
    {
        return ERROR_COMM;
    }
    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096.0;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}

/**@brief read cold-junction.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_cold_junc(float *value)
{
    *value=0;
    u16 read_value=0;
    if(IIC_read_16bit(COLD_JUNCTION_TEMP_REG_ADDR,&read_value))
    {
        return ERROR_COMM;
    }

    
    if(read_value&0xf000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}

/**@brief read raw ADC value.
 * @param data
 * @param data_len
 * @return 0 if successed.
 * */
err_t MCP9600::read_ADC_data(u8* data,u32 data_len)
{
    if(IIC_read_bytes(RAW_ADC_DATA_REG_ADDR,data,data_len))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}   

/**@brief read sensor status.
 * @param byte: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_status(u8* byte)
{
    *byte=0;
    if(IIC_read_byte(STAT_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_therm_cfg
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_therm_cfg(u8 set_byte)
{
    if(IIC_write_byte(THERM_SENS_CFG_REG_ADDR,set_byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}


/**@brief read thermocouple status.
 * @param byte: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_therm_cfg(u8* byte)
{
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_therm_type
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_therm_type(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }

    byte_to_set=(therm_cfg_data&0x8f)|set_byte;

    return IIC_write_byte(THERM_SENS_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_filt_coefficients
 * The  content of filt-coefficients register are:code-junction-resolution/ADC measurement resolution/burst mode temp samples/sensor mode
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_filt_coefficients(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }

    byte_to_set=(therm_cfg_data&0xf8)|set_byte;

    return IIC_write_byte(THERM_SENS_CFG_REG_ADDR,byte_to_set);
}


/**@brief set_dev_cfg
 * The content of device configuration register are:interrupt clear/monitor TH or TC/inerrupt pin rise or FALL/
 *                                                  Active-high or low/compare mode or int mode /alert enable or not
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_dev_cfg(u8 set_byte)
{
    if(IIC_write_byte(DEVICE_CFG_REG_ADDR,set_byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief read_dev_cfg
 * @param byte: the byte to be read in.
 * @return 0 if successed.
 * */
err_t MCP9600::read_dev_cfg(u8* byte)
{
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_sensor_mode
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_sensor_mode(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfc)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_burst_mode_samp
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_burst_mode_samp(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xe3)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}


/**@brief set_ADC_meas_resolution
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_ADC_meas_resolution(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0x9f)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_cold_junc_resolution
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_cold_junc_resolution(u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0x7f)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_alert_limitation
 * @param alert num the channel of alert
 * @param value: the 16bit value to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_limit(u8 alert_num,u16 value)
{
    if(IIC_write_16bit(TEMP_ALERT1_LIMIT_REG_ADDR+alert_num,value))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_alert_hysteresis
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set..
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_hys(u8 alert_num,u16 value)
{
    if(IIC_write_byte(ALERT1_HYS_REG_ADDR+alert_num,value))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_alert_cfg
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_cfg(u8 alert_num,u8 set_byte)
{
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,set_byte);
}

/**@brief read_alert_cfg
 * @param alert num the channel of alert
 * @param byte: the byte to be read in.
 * @return 0 if successed.
 * */
err_t MCP9600::read_alert_cfg(u8 alert_num,u8 *byte)
{
    *byte=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief clear_int_flag
 * @param alert num the channel of alert
 * @return 0 if successed.
 * */
err_t MCP9600::clear_int_flag(u8 alert_num)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data|0x80);
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}


/**@brief set_alert_for_TH_or_TC
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_for_TH_or_TC(u8 alert_num,u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xef)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_limit_direction
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_limit_direction(u8 alert_num,u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xf7)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_bit
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_bit(u8 alert_num,u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }

    byte_to_set=(therm_cfg_data&0xfb)|set_byte;

    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_mode_bit
 * set alert mode:comparator mode or INT mode.
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_mode_bit(u8 alert_num,u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfd)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_enable
 * Eable alert pin or not.
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_enable(u8 alert_num,u8 set_byte)
{
    u8 therm_cfg_data=0;
    u8 byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfe)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}


/**@brief check_data_update
 * check if data ready.
 * @param stat :indicate if data ready
 * @return 0 if successed.
 * */
err_t MCP9600::check_data_update(bool *stat)
{
    *stat=0;
    err_t ret=NO_ERROR;
    u8 byte=0;
    CHECK_RESULT(ret,read_status(&byte));
    if(byte&0x40)
    {
        *stat=true;
    }
    else
    {
        *stat=false;
    }
    return NO_ERROR;
}

/**@brief read_INT_stat
 * check if any interruption is generated.
 * @param stat :indicate if any interruption is generated
 * @return 0 if successed.
 * */
err_t MCP9600::read_INT_stat(u8 *stat)
{
    *stat=0;
    err_t ret=NO_ERROR;
    u8 byte=0;
    CHECK_RESULT(ret,read_status(&byte));
    for(int i=0;i<4;i++)
    {
        if(byte & 1<<i)
        {
            Serial.print("channel ");
            Serial.print(i);
            Serial.println("generate interruption!!!");
        }
    }
    *stat=byte;
    return NO_ERROR;
}




/******************************************************************************************************/
u16 MCP9600::covert_temp_to_reg_form(float temp)
{
    u8 negetive=0;
    if(temp<0) negetive=1;
    temp=abs(temp);
    u16 dest_temp=0;
    u8 temp_H=0,temp_L=0;
    u16 interger=(u16)temp;
    float decimal=temp-interger;
    temp_H=interger/16;
    temp_L|=(interger%16)<<4;
    temp_L|=(u8)(decimal/0.25)<<2;
    if(negetive)
        temp_H|=0x80;
    dest_temp=(u16)temp_H<<8|temp_L;
    return dest_temp;
}




/**********************************************************************************************************/
/************************************************IIC PART************************************************/
/**********************************************************************************************************/


/**@brief I2C write byte
 * @param reg :Register address of operation object
 * @param byte :The byte to be wrote.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600_IIC_OPRTS::IIC_write_byte(u8 reg,u8 byte)
{
    s32 ret=0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.write(byte);
    ret=Wire.endTransmission();
    if(!ret)
        return NO_ERROR;
    else
        return ERROR_COMM;
}


/**@brief I2C write 16bit value
 * @param reg: Register address of operation object
 * @param value: The 16bit value to be wrote .
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600_IIC_OPRTS::IIC_write_16bit(u8 reg,u16 value)
{
    s32 ret=0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    
    Wire.write((u8)(value>>8));
    Wire.write((u8)value);
    ret=Wire.endTransmission();
    if(!ret)
        return NO_ERROR;
    else
        return ERROR_COMM;
}



/**@brief I2C read byte
 * @param reg: Register address of operation object
 * @param byte: The byte to be read in.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600_IIC_OPRTS::IIC_read_byte(u8 reg,u8* byte)
{
    err_t ret=NO_ERROR;
    u32 time_out_count=0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR,(u8)1);
    while(1!=Wire.available())
    {
        time_out_count++;
        if(time_out_count>10)  return ERROR_COMM;
        delay(1);
    }
    *byte=Wire.read();
    return NO_ERROR;
}

/**@brief I2C read 16bit value
 * @param reg: Register address of operation object
 * @param byte: The 16bit value to be read in.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600_IIC_OPRTS::IIC_read_16bit(u8 start_reg,u16 *value)
{
    err_t ret=NO_ERROR;
    u32 time_out_count=0;
    u8 val=0;
    *value=0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR,sizeof(u16));
    while(sizeof(u16)!=Wire.available())
    {
        time_out_count++;
        if(time_out_count>10)  return ERROR_COMM;
        delay(1);
    }
    val=Wire.read();
    *value|=(u16)val<<8;
    val=Wire.read();
    *value|=val;
    return NO_ERROR;
}


/**@brief I2C read some bytes
 * @param reg: Register address of operation object
 * @param data: The buf  to be read in.
 * @param data_len: The length of buf need to read in.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600_IIC_OPRTS::IIC_read_bytes(u8 start_reg,u8 *data,u32 data_len)
{
    err_t ret=NO_ERROR;
    u32 time_out_count=0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR,data_len);
    while(data_len!=Wire.available())
    {
        time_out_count++;
        if(time_out_count>10)  return ERROR_COMM;
        delay(1);
    }
    
    for(int i=0;i<data_len;i++)
    {
        data[i]=Wire.read();
    }
    return ret;
}


/**@brief change the I2C address from default.
 * @param IIC_ADDR: I2C address to be set 
 * */
void MCP9600_IIC_OPRTS::set_iic_addr(u8 IIC_ADDR)
{
    _IIC_ADDR=IIC_ADDR;
}

