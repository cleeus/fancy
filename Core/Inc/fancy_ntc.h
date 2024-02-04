#ifndef INC_FANCY_NTC_H_
#define INC_FANCY_NTC_H_

#include <stdbool.h>
#include "main.h"

struct ntc_t {
	ADC_HandleTypeDef *hadc;
};

void  ntc_init(struct ntc_t *ntc, ADC_HandleTypeDef *ntc_adc);
float ntc_measure_degC(struct ntc_t *ntc);
bool  ntc_is_valid(const float val);

#endif
