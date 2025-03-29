/*
 * ADXL335.c
 *
 *  Created on: Mar 7, 2025
 *      Author: ace-rv
 */

#include "ADXL335.h"
#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <string.h>

void ADXL335_convert_ADCtomV(ADXL335_t* sensor, ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start_DMA(hadc, (uint32_t*)sensor->adc_value, numberOfAxes);
	for(int i = 0; i < numberOfAxes; i++)
	{
		sensor->mV_value[i] = (sensor->adc_value[i] * V_in) / adcResolution;
	}
}

void ADXL335_convert_mVtog(ADXL335_t* sensor)
{
	const uint16_t mV_zero_g_bias[numberOfAxes] = {mV_X_bias, mV_Y_bias, mV_Z_bias};

	for (int i = 0; i < numberOfAxes; i++)
	{
		sensor->g_value[i] = (sensor->mV_value[i] - mV_zero_g_bias[i]) / sensitivity_XYZ;
	}

////  If mv Bias is defined in ADXL335_t struct
//	for (int i = 0; i < numberOfAxes; i++)
//	{
//		sensor->g_value[i] = (sensor->mV_value[i] - sensor->mV_bias[i]) / sensitivity_XYZ;
//	}
}

void ADXL335_runSelfTest(ADXL335_t* sensor, uint8_t runSelfTest)
{
    // Check runSelfTest flag if true
    if(runSelfTest)
    {
    	for(int i = 0; i < numberOfAxes; i++)
    	{
    		sensor->selfTest_mV_value[i] = sensor->mV_value[i]; // Store mV value output affected by ST routine
    	}

    }

    for(int i = 0; i < numberOfAxes; i++)
    {
    	sensor->selfTest_mV_value[i] = sensor->selfTest_mV_value[i] - sensor->mV_value[i]; // Store ST mV to be compared later

    }
}

void ADXL335_checkSelfTestIfInRange(ADXL335_t* sensor)
{
	// For self-test routine, values obtained from ADXL335 datasheet
    const int16_t mV_STValueRange[numberOfAxes][2] = {
        { -600, -150 },  // X-axis: min = -600 mV, max = -150 mV
        {  150,  600 },  // Y-axis: min = 150 mV, max = 600 mV
        {  150, 1000 }   // Z-axis: min = 150 mV, max = 1000 mV
    };

    for(int i = 0; i < numberOfAxes; i++)
    {
    	sensor->selfTest_result[i] = (sensor->selfTest_mV_value[i] >= mV_STValueRange[i][0] &&
    								  sensor->selfTest_mV_value[i] <= mV_STValueRange[i][1]) ? 1 : 0; // Ternary operator to check pass or fail (1/0)
    }
}

void ADXL335_runCalibration(ADXL335_t* sensor)
{
	// 1. calculate x, y and z mV and g values on initial mV_bias estimates
}


//void ADXL335_sendSensorData(ADXL335_t* sensor, char* buffer, size_t buffer_size) {
//    if (sensor == NULL || buffer == NULL) return;
//
//    int len = 0;
//
//    // Format ADC values
//    len += sprintf(buffer + len, "ADC: X=%d Y=%d Z=%d ",
//                   sensor->adc_value[0], sensor->adc_value[1], sensor->adc_value[2]);
//
//    // Format mV values
//    len += sprintf(buffer + len, "mV: X=%.2f Y=%.2f Z=%.2f ",
//                   sensor->mV_value[0], sensor->mV_value[1], sensor->mV_value[2]);
//
//    // Format acceleration values
//    len += sprintf(buffer + len, "g: X=%.2f Y=%.2f Z=%.2f\r\n",
//                   sensor->g_value[0], sensor->g_value[1], sensor->g_value[2]);
//}

void ADXL335_sendSensorData(ADXL335_t* sensor, char* buffer, uint16_t buffer_size) {
    int len = 0;

    // Format as CSV: "ADC_X,ADC_Y,ADC_Z,mV_X,mV_Y,mV_Z,g_X,g_Y,g_Z"
    len += snprintf(buffer + len, buffer_size - len, "%d,%d,%d\r\n",
                   sensor->adc_value[0], sensor->adc_value[1], sensor->adc_value[2]);

//    len += snprintf(buffer + len, buffer_size - len, "%.2f,%.2f,%.2f,",
//                   sensor->mV_value[0], sensor->mV_value[1], sensor->mV_value[2]);
//
//    len += snprintf(buffer + len, buffer_size - len, "%.3f,%.3f,%.3f\r\n",
//                   sensor->g_value[0], sensor->g_value[1], sensor->g_value[2]);

    // Ensure null termination
    buffer[buffer_size - 1] = '\0';
}

uint16_t* ADXL335_getADC(ADXL335_t* sensor)
{
	return sensor->adc_value;
}

float* ADXL335_getmV(ADXL335_t* sensor)
{
	return sensor->mV_value;
}

float* ADXL335_getg(ADXL335_t* sensor)
{
	return sensor->g_value;
}


//void ADXL335_sendSensorData(ADXL335_t* sensor) {
//    if (sensor == NULL) return;
//
//    char buffer[100]; // Buffer to store the formatted string
//
////    // Format the output as: "ADC: X=1234 Y=2345 Z=3456 mV: X=1234.56 Y=2345.78 Z=3456.89 g: X=1.23 Y=2.34 Z=3.45\r\n"
////    snprintf(buffer, sizeof(buffer),
////             "ADC: X=%d Y=%d Z=%d mV: X=%.2f Y=%.2f Z=%.2f g: X=%.2f Y=%.2f Z=%.2f\r\n",
////             sensor->adc_value[0], sensor->adc_value[1], sensor->adc_value[2],
////             sensor->mV_value[0], sensor->mV_value[1], sensor->mV_value[2],
////             sensor->g_value[0], sensor->g_value[1], sensor->g_value[2]);
//
//    // Send the formatted string via UART
//    //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
//}


