#include <math.h>
#include <string.h>
#include "fancy_ntc.h"
#include "main.h"

/* specs (probably):
 * NTC 10k B3950 -20°C to 105°C
 * 10k series resistor
 *
 * calculations from
 * https://www.giangrandi.org/electronics/ntc/ntc.shtml
 *
 */

//post-NTC-curve calibration in degC space
static const float ntc_cal_T1_meas = 18.2;
static const float ntc_cal_T1_ref = 18.0;

static const float ntc_cal_T2_meas = 38.0;
static const float ntc_cal_T2_ref = 37.8;

static const float ntc_cal_slope = (ntc_cal_T2_meas-ntc_cal_T1_meas) / (ntc_cal_T2_ref-ntc_cal_T1_ref);
static const float ntc_cal_intercept = ntc_cal_T2_meas - ntc_cal_slope*ntc_cal_T2_ref;


//NTC measurements to calculate actual beta and R25 of my specific NTC
//two reference points (IR thermogun + voltmeter)
//19.2 degC / 13330 Ohm
//20.4 degC / 12560 Ohm
static const float ntc_meas_T1 = 19.2 + 273.15; //19.2 degC -> K
static const float ntc_meas_R1 = 13330; //Ohm

//42.4 degC / 4990 Ohm
//43.6 degC / 4770 Ohm
static const float ntc_meas_T2 = 43.6 + 273.15; //43.6 degC -> K
static const float ntc_meas_R2 = 4770; //Ohm

//const float ntc_beta_meas = (log(ntc_meas_R1) - log(ntc_meas_R2)) / (1/ntc_meas_T1 - 1/ntc_meas_T2); // = 3900.17163
//const float ntc_r25_meas = ntc_meas_R1 / exp(ntc_beta_meas * (1/ntc_meas_T1 - 1/(25.0 + 273.15))); //10283.0322 Ohm
static const float ntc_beta_meas = 3900.17163;
static const float ntc_r25_meas = 10283.0322;

//serial voltage divider resistor measurement
static const float ntc_r_ser_meas = 9970; //Ohm
//reference voltage measurement
static const float ntc_v_ref_meas = 3.31f;

//datasheet values
static const float ntc_beta_spec = 3950; //K
static const float ntc_r25_spec = 10000; //Ohm
static const float ntc_r_ser_spec = 10000; //Ohm
static const float ntc_v_ref_spec = 3.3f;

//choose measured/spec values
static const float ntc_beta = ntc_beta_meas;
static const float ntc_r25 = ntc_r25_meas;
static const float ntc_r_ser = ntc_r_ser_meas;
static const float ntc_v_ref = ntc_v_ref_meas;

void ntc_init(struct ntc_t *ntc, ADC_HandleTypeDef *ntc_adc) {
	memset(ntc, 0, sizeof(*ntc));
	ntc->hadc = ntc_adc;
}

static uint32_t ntc_adc_conv(struct ntc_t *ntc) {
	const HAL_StatusTypeDef start_status = HAL_ADC_Start(ntc->hadc);
	if(start_status != HAL_OK) {
		return INT32_MIN;
	}

	const HAL_StatusTypeDef conv_status = HAL_ADC_PollForConversion(ntc->hadc, 1);
	if(conv_status != HAL_OK) {
		return INT32_MIN;
	}

	const uint32_t result = HAL_ADC_GetValue(ntc->hadc);
	return result;
}

float ntc_measure_degC(struct ntc_t *ntc) {
	//const float ntc_beta_meas = (log(ntc_meas_R1) - log(ntc_meas_R2)) / (1/ntc_meas_T1 - 1/ntc_meas_T2); //3900.17163
	//const float ntc_r25_meas = ntc_meas_R1 / exp(ntc_beta_meas * (1/ntc_meas_T1 - 1/(25.0 + 273.15))); //10283.0322 Ohm

	//STM32 ADC self-calibration
	HAL_ADCEx_Calibration_Start(ntc->hadc);
	HAL_ADC_Stop(ntc->hadc);

	uint32_t adc_accu = 0;
	uint8_t  adc_cnt = 0;
	for(uint8_t i=0; i<32; i++) {
		const uint32_t conv = ntc_adc_conv(ntc);
		if(conv == INT32_MIN) continue;
		adc_accu += conv;
		adc_cnt++;
	}
	if(adc_cnt == 0) {
		return NAN;
	}

	const float adc_avg = (float)adc_accu / (float)adc_cnt;

	//calculations
	const float v_step = (float)(ntc_v_ref / (float)((1<<12)-1)); // V/step
	const float v_adc = adc_avg * v_step; //V

	const float t_25 = 25.0 + 273.15; //25deg in K
	const float r_ntc = ntc_r_ser * (v_adc / (ntc_v_ref - v_adc)); //Ohm
	const float t_K = 1.0 / (log(r_ntc/ntc_r25) / ntc_beta + 1/t_25); //K
	const float t_degC = t_K - 273.15;


	const float t_degC_cal = ntc_cal_slope * t_degC + ntc_cal_intercept;

	return t_degC_cal;
	//return t_degC;
}

bool ntc_is_valid(const float val) {
	return !isnan(val) && !isinf(val);
}

