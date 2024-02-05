#include "fancy_adctemp.h"
#include <math.h>
#include <string.h>
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

struct adctemp_t *g_fancy_at = NULL;

void adctemp_init(struct adctemp_t *at, ADC_HandleTypeDef *hadc) {
	memset(at, 0, sizeof(*at));
	at->hadc = hadc;
	g_fancy_at = at;
}

static void adctemp_delay_us(uint8_t delay)
{
  while (delay > 0)
  {
    delay--;
    //__nop();__nop();__nop();__nop();
    for (int i = 0; i < 4; i++) {
    	__asm__ __volatile__("nop\n\t":::"memory");
    }
  }
}

static bool adctemp_conv(struct adctemp_t *at) {
	memset(at->data, 0xFF, sizeof(at->data));

	at->conversion_complete = false;
	HAL_StatusTypeDef start_dma = HAL_ADC_Start_DMA(at->hadc, at->data, 3);
	if(start_dma != HAL_OK) {
		return false;
	}

	for(int count=0; count<10000 && !at->conversion_complete; count++) {
		adctemp_delay_us(1);
	}

	HAL_ADC_Stop_DMA(at->hadc);

	if(!at->conversion_complete) {
		return false;
	}
	return true;
	/*
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
	*/
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	g_fancy_at->conversion_complete = true;
}

void adctemp_measure(struct adctemp_t *at, struct adctemp_measurement_t *result) {
	result->core_degC_is_valid = false;
	result->ntc_degC_is_valid = false;
	result->vdda_V_is_valid = false;

	//const float ntc_beta_meas = (log(ntc_meas_R1) - log(ntc_meas_R2)) / (1/ntc_meas_T1 - 1/ntc_meas_T2); //3900.17163
	//const float ntc_r25_meas = ntc_meas_R1 / exp(ntc_beta_meas * (1/ntc_meas_T1 - 1/(25.0 + 273.15))); //10283.0322 Ohm

	//STM32 ADC self-calibration
	HAL_ADCEx_Calibration_Start(at->hadc);
	HAL_ADC_Stop(at->hadc);

	uint32_t ntc_accu = 0;
	uint32_t core_accu = 0;
	uint32_t vrefint_accu = 0;

	uint8_t ntc_cnt = 0;
	uint8_t core_cnt = 0;
	uint8_t vrefint_cnt = 0;

	for(uint8_t i=0; i<16; i++) {
		adctemp_conv(at);
		const uint16_t ntc_conv  = ((const uint16_t*)at->data)[0];
		const uint16_t core_conv = ((const uint16_t*)at->data)[1];
		const uint16_t vrefint_conv = ((const uint16_t*)at->data)[2];

		if(ntc_conv != INT16_MIN) {
			ntc_accu += ntc_conv;
			ntc_cnt++;
		}
		if(core_conv != INT16_MIN) {
			core_accu += core_conv;
			core_cnt++;
		}
		if(vrefint_conv != INT16_MIN) {
			vrefint_accu += vrefint_conv;
			vrefint_cnt++;
		}
	}

	float vdda = 3.3f;

	if(vrefint_cnt != 0) {
		const float vrefint_avg = (float)vrefint_accu / (float)vrefint_cnt;
		if(vrefint_avg >= (4095.0f/3.3f) * 1.15f && vrefint_avg <= (4095.0f/3.3f) * 1.25f) {
			vdda = 4095.0f * 1.20f / vrefint_avg;
			result->vdda_V = vdda;
			result->vdda_V_is_valid = true;
		}
	}

	if(core_cnt != 0) {
		const float core_avg = (float)core_accu / (float)core_cnt;

		//raw ADC to voltage
		const float v_step = (float)(vdda / 4095.0); // V/step
		const float VSENSE = core_avg * v_step; //V

		//voltage to temperature acc. to reference manual
		const float V25 = 1.43; //V at 25 degC
		const float Avg_Slope = 4.3 * 0.001; // V / degC

		const float core_temp_degC = ((V25 - VSENSE) / Avg_Slope) + 25.0;
		result->core_degC = core_temp_degC;
		result->core_degC_is_valid = true;
	}

	if(ntc_cnt != 0) {
		const float ntc_avg = (float)ntc_accu / (float)ntc_cnt;

		//raw ADC to voltage
		const float v_step = vdda / 4095.0; // V/step
		const float v_adc = ntc_avg * v_step; //V

		//NTC curve calculation
		const float t_25 = 25.0 + 273.15; //25deg in K
		const float r_ntc = ntc_r_ser * (v_adc / (vdda - v_adc)); //Ohm
		const float t_K = 1.0 / (logf(r_ntc/ntc_r25) / ntc_beta + 1/t_25); //K
		const float t_degC = t_K - 273.15;

		//2point calibration
		const float t_degC_cal = ntc_cal_slope * t_degC + ntc_cal_intercept;

		result->ntc_degC = t_degC;
		//result->ntc_degC = t_degC_cal;
		result->ntc_degC_is_valid = true;
	}

}
