#include "Seeed_MCP9600.h"


MCP9600 sensor;

error_t sensor_basic_config()
{
    s32 ret=0;
    CHECK_RESULT(ret,sensor.set_filt_coefficients(FILT_MID));
    CHECK_RESULT(ret,sensor.set_cold_junc_resolution(COLD_JUNC_RESOLUTION_0_25));
    CHECK_RESULT(ret,sensor.set_ADC_meas_resolution(ADC_14BIT_RESOLUTION));
    CHECK_RESULT(ret,sensor.set_burst_mode_samp(BURST_32_SAMPLE));
    CHECK_RESULT(ret,sensor.set_sensor_mode(NORMAL_OPERATION));
    return NO_ERROR;
}


error_t get_temperature(float *value)
{
    s32 ret=0;
    float hot_junc=0;
    float junc_delta=0;
    float cold_junc=0;
    CHECK_RESULT(ret,sensor.read_hot_junc(&hot_junc));
    CHECK_RESULT(ret,sensor.read_junc_temp_delta(&junc_delta));
    
    CHECK_RESULT(ret,sensor.read_cold_junc(&cold_junc));
    
    // Serial.print("hot junc=");
    // Serial.println(hot_junc);
    // Serial.print("junc_delta=");
    // Serial.println(junc_delta);
    // Serial.print("cold_junc=");
    // Serial.println(cold_junc);

    *value=hot_junc;

    return NO_ERROR;
}


void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println("serial start!!");
    if(sensor.init(THER_TYPE_K))
    {
        Serial.println("sensor init failed!!");
    }
    sensor_basic_config();
}



void loop()
{
    float temp=0;
    get_temperature(&temp);
    Serial.print("temperature ===================>");
    Serial.println(temp);
    Serial.println(" ");
    Serial.println(" ");
    delay(1000);
}