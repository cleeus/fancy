#ifndef INC_FANCY_NTC_H_
#define INC_FANCY_NTC_H_

#include <stdbool.h>
#include "main.h"

void  ntc_init(ADC_HandleTypeDef *ntc_adc);
float ntc_get_degrees(void);
bool  ntc_is_valid(const float val);

#endif
