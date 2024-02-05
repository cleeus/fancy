#ifndef INC_FANCY_ADCTEMP_H_
#define INC_FANCY_ADCTEMP_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

struct adctemp_t {
	ADC_HandleTypeDef *hadc;
	uint32_t data[4];
	bool conversion_complete;
};

void  adctemp_init(struct adctemp_t *at, ADC_HandleTypeDef *hadc);

struct adctemp_measurement_t {
	float ntc_degC;
	bool ntc_degC_is_valid;

	float core_degC;
	bool core_degC_is_valid;

	float vdda_V;
	bool vdda_V_is_valid;
};

void adctemp_measure(struct adctemp_t *at, struct adctemp_measurement_t *result);


#endif
