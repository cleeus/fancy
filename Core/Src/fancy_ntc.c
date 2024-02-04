#include <math.h>
#include "fancy_ntc.h"
#include "main.h"

static ADC_HandleTypeDef *g_ntc_adc;

/* specs (probably):
 * NTC 10k B3950 -20°C to 105°C
 * 10k series resistor
 *
 * calculations from
 * https://www.giangrandi.org/electronics/ntc/ntc.shtml
 *
 */


void ntc_init(ADC_HandleTypeDef *ntc_adc) {
	g_ntc_adc = ntc_adc;
}

float ntc_get_degrees(void) {
	const HAL_StatusTypeDef start_status = HAL_ADC_Start(g_ntc_adc);
	if(start_status != HAL_OK) {
		return NAN;
	}

	const HAL_StatusTypeDef conv_status = HAL_ADC_PollForConversion(g_ntc_adc, 100);
	if(conv_status != HAL_OK) {
		return NAN;
	}

	const uint32_t result = HAL_ADC_GetValue(g_ntc_adc);

	const float beta = 3950; //Ohm
	const float r_ntc_25 = 10000; //Ohm
	const float r_ser = 10000; //Ohm

	const float v_ref = 3.3f; //V
	const float v_step = (float)(v_ref / (float)((1<<12)-1)); // V/step
	const float v_adc = result * v_step; //V

	const float t_25 = 298.15; //25deg in K
	const float r_ntc = r_ser * (v_adc / (v_ref - v_adc)); //Ohm
	const float t_K = 1.0 / (log(r_ntc/r_ntc_25) / beta + 1/t_25); //K
	const float t_degC = t_K - 273.15;

	return t_degC;
}

bool ntc_is_valid(const float val) {
	return !isnan(val);
}

