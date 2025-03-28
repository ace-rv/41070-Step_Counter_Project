/*
 * ADXL335.h
 *
 *  Created on: Mar 7, 2025
 *      Author: ace-rv
 *
 */

#ifndef INC_ADXL335_H_
#define INC_ADXL335_H_

#include <stdint.h>
#include "stm32g0xx_hal.h"

// For ADC conversion to mV
/*
 * The Vin is from the LD1117S33CTR voltage regulator
 * The output should be at 3.3v or 3300 mV
 *
 * The resolution value is based off the 12-bit ADC setting
 * of the STM32G050F6 that is currently set in the .ioc
 */
#define numberOfAxes 3
#define V_in 3300 // Adjust V_in based on board voltage regulator output
#define adcResolution 4095

// For mV to acceleration conversion (g)
/*
 *  For Zero-g Bias Voltage Refer to ADXL335 Data sheet
 */

#define mV_X_bias 1650 // Can check and edit these values based on voltage regulator output
#define mV_Y_bias 1650
#define mV_Z_bias 1800

#define sensitivity_XYZ 330 // in mV/g units, assumed same for XYZ

typedef struct
{
	uint16_t adc_value[numberOfAxes]; // X, Y, Z :adc values stored here after reading from ADXL335
	float mV_bias[numberOfAxes];
	float mV_value[numberOfAxes]; // X, Y, Z :mV values stored here after conversion (adc to mV)
	float g_value[numberOfAxes]; // X, Y, Z :g values stored here after conversion (mV to g)
	float selfTest_mV_value[numberOfAxes]; // Store the change in X, Y, Z axis after self-test
	uint8_t selfTest_result[numberOfAxes]; // Store the result of the selfTest routine for X, Y, Z

} ADXL335_t;


// Functions for ADXL335
void ADXL335_convert_ADCtomV(ADXL335_t* sensor, ADC_HandleTypeDef* hadc);
void ADXL335_convert_mVtog(ADXL335_t* sensor);
void ADXL335_runSelfTest(ADXL335_t* sensor, uint8_t runSelfTest);
void ADXL335_checkSelfTestIfInRange(ADXL335_t* sensor);
void ADXL335_runCalibration(ADXL335_t* sensor);
void ADXL335_sendSensorData(ADXL335_t* sensor, char* buffer, uint16_t buffer_size);
uint16_t* ADXL335_getADC(ADXL335_t* sensor);
float* ADXL335_getmV(ADXL335_t* sensor);
float* ADXL335_getg(ADXL335_t* sensor);

#endif /* INC_ADXL335_H_ */
