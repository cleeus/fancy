#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include "fancy_adctemp.h"
#include "main.h"
#include "tm1637.h"
#include "dht.h"
#include "fancy_rectrl.h"
#include "fancy_regulator.h"
#include "fancy_ticks.h"

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))


struct measurement_t {
	float val;
	bool is_valid;
	int64_t time;
};

struct fancy_measurements_t {
	struct measurement_t temp_dht;
	struct measurement_t temp_ntc;
	struct measurement_t temp_core;
	struct measurement_t vdda;
};

struct fancy_lpfilt_t {
	float state;
	float coef;
};

enum SensorFusionLevel {
	SFL_FUNCTIONING = 0,
	SFL_DEGRADED = 1,
	SFL_DEFUNCT = 2
};

static struct Fancy_t {
	bool is_initialized;

	struct fancy_measurements_t msm_raw;
	struct fancy_measurements_t msm_checked;
	struct fancy_measurements_t msm_lastvalid;
	struct fancy_measurements_t msm_filtered;
	struct measurement_t temp_fused;
	struct measurement_t temp_fused_filtered;
	enum SensorFusionLevel temp_fusion_level;
	int64_t                temp_fusion_level_time;

	struct fancy_lpfilt_t filt_temp_core;
	struct fancy_lpfilt_t filt_temp_ntc;
	struct fancy_lpfilt_t filt_temp_dht;
	struct fancy_lpfilt_t filt_temp;

	struct FancyRegulator_t regulator;

	rectrl_t rectrl;
	int8_t switch_state;

	TIM_HandleTypeDef *buzzer_tim;
	uint8_t            buzzer_tim_channel;

	struct adctemp_t adctemp;

} g_fancy = {0};

//static tm1637_t g_tm1637;
static tm1637_t g_tm1637_INV;
static DHT_t    g_dht11;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(!g_fancy.is_initialized) {
		return;
	}

  if(GPIO_Pin == g_dht11.pin) {
    DHT_pinChangeCallBack(&g_dht11);
  }
}

static void fancy_init(
		TIM_HandleTypeDef *dht11_tim,
		TIM_HandleTypeDef *buzzer_tim,
		uint8_t            buzzer_tim_channel,
		ADC_HandleTypeDef *ntc_adc)
{
	//tm1637_init(&g_tm1637, TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, TM1637_DIO_GPIO_Port, TM1637_DIO_Pin, TM1637_POLARITY_NORMAL_OD);
	//tm1637_show_zero(&g_tm1637, true);

	tm1637_init(&g_tm1637_INV, TM1637_NCLK_GPIO_Port, TM1637_NCLK_Pin, TM1637_NDIO_GPIO_Port, TM1637_NDIO_Pin, TM1637_POLARITY_INV_PP);
	tm1637_show_zero(&g_tm1637_INV, true);

	DHT_init(&g_dht11, DHT_Type_DHT11, dht11_tim, 8 /*MHz system clock*/, DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);

	g_fancy.buzzer_tim = buzzer_tim;
	g_fancy.buzzer_tim_channel = buzzer_tim_channel;

	rectrl_init(&g_fancy.rectrl);

	adctemp_init(&g_fancy.adctemp, ntc_adc);


	g_fancy.filt_temp_dht.state = NAN;
	g_fancy.filt_temp_dht.coef = 1.3f;

	g_fancy.filt_temp_ntc.state = NAN;
	g_fancy.filt_temp_ntc.coef = 1.3f;

	g_fancy.filt_temp_core.state = NAN;
	g_fancy.filt_temp_core.coef = 1.3f;

	g_fancy.filt_temp.state = NAN;
	g_fancy.filt_temp.coef = 1.7f;

	fancy_regulator_init(&g_fancy.regulator);
	g_fancy.switch_state = 0;

	g_fancy.is_initialized = true;
}

static bool fancy_is_panicked(void) {
	if(g_fancy.temp_fusion_level == SFL_DEFUNCT) {
		const int64_t now = fancy_gettick();
		const int64_t defunct_duration_ms = (now - g_fancy.temp_fusion_level_time);
		if(defunct_duration_ms > 5000) {
			return true;
		}
	}
	return false;
}

static bool fancy_is_degraded(void) {
	if(g_fancy.temp_fusion_level == SFL_DEGRADED) {
		const int64_t now = fancy_gettick();
		const int64_t degraded_duration_ms = (now - g_fancy.temp_fusion_level_time);
		if(degraded_duration_ms > 10000) {
			return true;
		}
	}
	return false;
}

static void fancy_heartbeat(void) {
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_RESET);
  HAL_Delay(40);
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
}

static void fancy_tm1637_display_update(void) {
	//tm1637_brightness(&g_tm1637, 2);
	//tm1637_write_fractional(&g_tm1637, g_fancy.temp, 1, 0);

	static int8_t brightness_counter = 0;
	static int8_t brightness_inc = 1;
	static int8_t value_cycle = 0;

	brightness_counter += brightness_inc;
	if(brightness_counter >= 3) {
		brightness_inc = -1;
	} else if(brightness_counter <= 0) {
		brightness_inc = 1;
		value_cycle++;
	}

	tm1637_brightness(&g_tm1637_INV, brightness_counter);

	switch(value_cycle) {
		case 0:
			if(g_fancy.temp_fused_filtered.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 'A', g_fancy.temp_fused_filtered.val, 1, 0);
			} else {
				const char A_invalid[] = "A---";
				tm1637_write_str(&g_tm1637_INV, A_invalid, sizeof(A_invalid), 0);
			}
			break;
		case 1:
			tm1637_write_fractional(&g_tm1637_INV, 'L', g_fancy.switch_state, 0, 0);
			break;
		case 2:
			if(g_fancy.msm_raw.temp_dht.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 'd', g_fancy.msm_raw.temp_dht.val, 1, 0);
			} else {
				const char d_invalid[] = "d---";
				tm1637_write_str(&g_tm1637_INV, d_invalid, sizeof(d_invalid), 0);
			}
			break;
		case 3:
			if(g_fancy.msm_raw.temp_ntc.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 't', g_fancy.msm_raw.temp_ntc.val, 1, 0);
			} else {
				const char t_invalid[] = "t---";
				tm1637_write_str(&g_tm1637_INV, t_invalid, sizeof(t_invalid), 0);
			}
			break;
		case 4:
			if(g_fancy.msm_raw.temp_core.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 'c', g_fancy.msm_raw.temp_core.val, 1, 0);
			} else {
				const char c_invalid[] = "c---";
				tm1637_write_str(&g_tm1637_INV, c_invalid, sizeof(c_invalid), 0);
			}
			break;
		case 5:
			if(g_fancy.msm_raw.vdda.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 'u', g_fancy.msm_raw.vdda.val, 2, 0);
			} else {
				const char u_invalid[] = "u---";
				tm1637_write_str(&g_tm1637_INV, u_invalid, sizeof(u_invalid), 0);
			}
			break;

		default:
			value_cycle = 0;
			break;
	}
}

static void fancy_sensor_range_check(const struct measurement_t * const in, struct measurement_t * const out, const float limit_min, const float limit_max, const uint32_t max_age) {
	const int64_t now = fancy_gettick();
	out->is_valid = false;
	if(    in->is_valid
			&& !isinff(in->val)
			&& !isnanf(in->val)
			&& in->val >= limit_min
			&& in->val <= limit_max
			&& (now - in->time) <= max_age)
	{
		*out = *in;
	}
}

static void fancy_sensor_range_checks(void) {
	fancy_sensor_range_check(&g_fancy.msm_raw.vdda,      &g_fancy.msm_checked.vdda,      3.2f,  3.4f,  10000);
	fancy_sensor_range_check(&g_fancy.msm_raw.temp_dht,  &g_fancy.msm_checked.temp_dht,  15.0f, 50.0f, 10000);
	fancy_sensor_range_check(&g_fancy.msm_raw.temp_ntc,  &g_fancy.msm_checked.temp_ntc,  15.0f, 50.0f, 10000);
	fancy_sensor_range_check(&g_fancy.msm_raw.temp_core, &g_fancy.msm_checked.temp_core, 15.0f, 50.0f, 10000);

	//if vdda 3.3V is not valid, core and ntc can not be trusted either
	if(!g_fancy.msm_checked.vdda.is_valid) {
		g_fancy.msm_checked.temp_core.is_valid = false;
		g_fancy.msm_checked.temp_ntc.is_valid = false;
	}

	if(g_fancy.msm_checked.temp_dht.is_valid)  { g_fancy.msm_lastvalid.temp_dht  = g_fancy.msm_checked.temp_dht;}
	if(g_fancy.msm_checked.temp_ntc.is_valid)  { g_fancy.msm_lastvalid.temp_ntc  = g_fancy.msm_checked.temp_ntc;}
	if(g_fancy.msm_checked.temp_core.is_valid) { g_fancy.msm_lastvalid.temp_core = g_fancy.msm_checked.temp_core;}

}


static float fancy_lpfilt(struct fancy_lpfilt_t *filt, const float in) {
	if(isnanf(filt->state)) {
		filt->state = in;
	} else {
		filt->state = (filt->state * filt->coef + in) / (1+filt->coef);
	}

	return filt->state;
}

static void fancy_sensor_filter_apply(struct fancy_lpfilt_t *filt, const struct measurement_t * const in, struct measurement_t * const out) {
	out->val      = fancy_lpfilt(filt, in->val);
	out->is_valid = false; //choose validity later
	out->time     = in->time;
}

static void fancy_sensor_filters(void) {
	fancy_sensor_filter_apply(&g_fancy.filt_temp_dht,  &g_fancy.msm_lastvalid.temp_dht,  &g_fancy.msm_filtered.temp_dht);
	fancy_sensor_filter_apply(&g_fancy.filt_temp_ntc,  &g_fancy.msm_lastvalid.temp_ntc,  &g_fancy.msm_filtered.temp_ntc);
	fancy_sensor_filter_apply(&g_fancy.filt_temp_core, &g_fancy.msm_lastvalid.temp_core, &g_fancy.msm_filtered.temp_core);

	//validity from original signal
	g_fancy.msm_filtered.temp_dht.is_valid  = g_fancy.msm_checked.temp_dht.is_valid;
	g_fancy.msm_filtered.temp_ntc.is_valid  = g_fancy.msm_checked.temp_ntc.is_valid;
	g_fancy.msm_filtered.temp_core.is_valid = g_fancy.msm_checked.temp_core.is_valid;
}

static void fancy_update_sensor_fusion_level(enum SensorFusionLevel const level) {
	if(level != g_fancy.temp_fusion_level) {
		g_fancy.temp_fusion_level = level;
		g_fancy.temp_fusion_level_time = fancy_gettick();
	}
}

static void fancy_sensor_fusion(void) {
	const float weight_core = 0.7f;
	const float weight_ntc = 1.5f;
	const float weight_dht = 1.0f;

	if(g_fancy.msm_filtered.temp_core.is_valid
	&& g_fancy.msm_filtered.temp_dht.is_valid
	&& g_fancy.msm_filtered.temp_ntc.is_valid)
	{
		const float inv_weight_sum = 1.0f / (weight_core + weight_ntc + weight_dht);

		g_fancy.temp_fused.val =
					(g_fancy.msm_filtered.temp_core.val * weight_core
				 + g_fancy.msm_filtered.temp_dht.val * weight_dht
				 + g_fancy.msm_filtered.temp_ntc.val * weight_ntc) * inv_weight_sum;
		g_fancy.temp_fused.is_valid = true;
		g_fancy.temp_fused.time = min(min(g_fancy.msm_filtered.temp_core.time, g_fancy.msm_filtered.temp_dht.time), g_fancy.msm_filtered.temp_ntc.time);
		fancy_update_sensor_fusion_level(SFL_FUNCTIONING);
	}
	else {
		const struct measurement_t * in[3];
		float weight[3] = {0};
		int mcount = 0;
		if(g_fancy.msm_filtered.temp_core.is_valid) {
			in[mcount] = &g_fancy.msm_filtered.temp_core;
			weight[mcount] = weight_core;
			mcount++;
		}
		if(g_fancy.msm_filtered.temp_dht.is_valid) {
			in[mcount] = &g_fancy.msm_filtered.temp_dht;
			weight[mcount] = weight_dht;
			mcount++;
		}
		if(g_fancy.msm_filtered.temp_ntc.is_valid) {
			in[mcount] = &g_fancy.msm_filtered.temp_ntc;
			weight[mcount] = weight_ntc;
			mcount++;
		}

		if(mcount == 2) {
			const float inv_weight_sum = 1.0f / (weight[0] + weight[1]);
			g_fancy.temp_fused.val =
							(in[0]->val * weight[0]
						 + in[1]->val * weight[1]) * inv_weight_sum;
			g_fancy.temp_fused.is_valid = true;
			g_fancy.temp_fused.time = min(in[0]->time, in[1]->time);

			fancy_update_sensor_fusion_level(SFL_DEGRADED);
		} else if(mcount == 1) {
			g_fancy.temp_fused = *in[0];
			fancy_update_sensor_fusion_level(SFL_DEGRADED);
		} else if(mcount == 0) {
			g_fancy.temp_fused.is_valid = false;
			fancy_update_sensor_fusion_level(SFL_DEFUNCT);
		}
	}

	if(g_fancy.temp_fused.is_valid) {
		fancy_sensor_filter_apply(&g_fancy.filt_temp,  &g_fancy.temp_fused,  &g_fancy.temp_fused_filtered);
	}
	g_fancy.temp_fused_filtered.is_valid = g_fancy.temp_fused.is_valid;
}

enum BuzzerFreqs {
	BUZZER_FREQ_LOUDEST = 2150,
	BUZZER_FREQ_ALIVE1 = 1046,
	BUZZER_FREQ_ALIVE2 = 1760,
};
static void fancy_buzzer_sound(const uint16_t freq, const uint8_t duration_ms) {
	//const uint16_t freq = 2150; //Hz
  const uint32_t timer_clk_freq_hz = HAL_RCC_GetPCLK1Freq();

	// f_PWM = f_CLK / ((ARR+1)*(PSC+1))
	// f_PWM * (ARR+1) = f_CLK / (PSC+1)
	// ARR+1 = f_CLK / ((PSC+1)*f_PWM)
	// ARR = f_CLK / ((PSC+1)*f_PWM) - 1

	TIM_HandleTypeDef *tim = g_fancy.buzzer_tim;

	//calculate counter length
	tim->Instance->ARR =
			timer_clk_freq_hz / ((tim->Init.Prescaler+1) * freq) - 1;

	//set duty cycle 50%
	__HAL_TIM_SET_COMPARE(tim, g_fancy.buzzer_tim_channel, tim->Instance->ARR/2);

	HAL_TIM_PWM_Start(tim, g_fancy.buzzer_tim_channel);

	HAL_Delay(duration_ms);

	HAL_TIM_PWM_Stop(tim, g_fancy.buzzer_tim_channel);
}

static void fancy_periodic_alive_sound(void) {
	static uint32_t cycle_count = 0;
	cycle_count++;
	//if(cycle_count >= 7 /*hours*/ * 60 /*minutes*/ * 60 /*seconds*/) {
	//if(cycle_count >= 1 /*minutes*/ * 60 /*seconds*/) {
	if(cycle_count >= 10 /* seconds */) {
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE1, 30);
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE2, 30);
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE1, 30);
		cycle_count = 0;
	}
}

struct fancy_switchconf_t {
	uint16_t  R_CH0:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH1:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH2:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH3:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH4:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH5:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH6:1; //output, switches between 12V (inactive) and alternative voltage supply (active)
	uint16_t  R_CH7_12V:1; //voltage routing, switches between 12V supply on (inactive) and off (active)
	uint16_t  M_ALTV:1; //voltage routing, altvolt mosfet 1&2, switches between off (inactive) and on (active)
	uint16_t  M_CH7_12V:1; //output, switches 12V on output channel 8, off=inactive, on=active
	uint16_t  M_CH7_ALTV:1; //output, switches altvolt on output channel 8, off=inactive, on=active
};

/*
 * Fan array description
 *
 * AF8 = https://www.tacens-anima.com/wp-content/uploads/AF8-ficha-en.pdf
 * 			 1800 RPM @12V
 * 			 14dBA
 * 			 0.12A
 * 			 51 m^3/h (30.5CFM)
 *
 * NR8 = https://noctua.at/de/nf-r8-redux-1800/specification
 *       1800 RPM @12V
 *       17dBA
 *       53m^3/h
 *       0.06A (max 0.11A)
 *       0.73W (max 1.32W)
 *
 *
 *     +---------+--------+--------+--------+--------+
 *     | CH2     | CH7    | CH3    | CH5    | CH6    |
 *     |         |        |        |        |        |
 *     | NR8     | AF8    | AF8    | AF8    |        |
 *     +---------+--------+--------+-----}--+-----}--+
 *     | CH1     | CH0    | CH4    | CH5    | CH6    |
 *     |         |        |        |        |        |
 *     | AF8     | AF8    | AF8    | AF8    | AF8    |
 *     +---------+--------+--------+--------+--------+
 */

static const struct fancy_switchconf_t fancy_switchconf[] = {
	//max (reset state), all 12V outputs on, altvolt off
	{.R_CH7_12V=0, .M_ALTV=0, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
	//switch on altvolt supply, move NR8 to altvolt
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=1},
	//move channels to altvolt one by one
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=1, .R_CH2=1},
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=1, .R_CH6=1, .R_CH2=1},
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=1, .R_CH5=1, .R_CH6=1, .R_CH2=1},
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=1, .R_CH4=1, .R_CH5=1, .R_CH6=1, .R_CH2=1},
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=1, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=1, .R_CH3=1, .R_CH4=1, .R_CH5=1, .R_CH6=1, .R_CH2=1},
	//center, all outputs on altvolt, 12V still on (saves relay driver power)
	{.R_CH7_12V=0, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=1, .R_CH4=1, .R_CH5=1, .R_CH6=1, .R_CH2=1},
	//switch off 12V, switch off NR8
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=1, .R_CH4=1, .R_CH5=1, .R_CH6=1, .R_CH2=0},
	//switch off channels one by one
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=1, .R_CH4=1, .R_CH5=1, .R_CH6=0, .R_CH2=0},
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=1, .R_CH4=1, .R_CH5=0, .R_CH6=0, .R_CH2=0},
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=1, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=1, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=1, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
	{.R_CH7_12V=1, .M_ALTV=1, .M_CH7_12V=0, .M_CH7_ALTV=1, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
  //min, 12V supply switched off, CH1-7 on 12V, CH7 off
	{.R_CH7_12V=1, .M_ALTV=0, .M_CH7_12V=0, .M_CH7_ALTV=0, .R_CH1=0, .R_CH0=0, .R_CH3=0, .R_CH4=0, .R_CH5=0, .R_CH6=0, .R_CH2=0},
};

static void fancy_transition_switch_state(const int old_switch_i, const int new_switch_i) {
	if(old_switch_i < 0 || old_switch_i >= sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0])) {
		return;
	}
	if(new_switch_i < 0 || new_switch_i >= sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0])) {
		return;
	}

	const struct fancy_switchconf_t * const swconf = &fancy_switchconf[new_switch_i];

	const int load_switch_delay = 500;
	if(new_switch_i != old_switch_i) {
		//actually switch transition

		const bool R_CH7_12V      = rectrl_is_active(&g_fancy.rectrl, 7);
		const bool M_ALTV         = HAL_GPIO_ReadPin(ALTVOLT_MOSFET_1_GPIO_Port, ALTVOLT_MOSFET_1_Pin) == GPIO_PIN_SET;
		const bool switch_on_12V  =  R_CH7_12V && !swconf->R_CH7_12V;
		const bool switch_on_ALTV = !M_ALTV    &&  swconf->M_ALTV;

		if(switch_on_12V) {
			//turn off CH7 consumer first
			HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port, CH7_MOSFET_12V_Pin, GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
		}
		if(switch_on_ALTV) {
			//turn off CH7 consumer first
			HAL_GPIO_WritePin(CH7_MOSFET_ALTVOLT_GPIO_Port, CH7_MOSFET_ALTVOLT_Pin, GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
		}

		if(!swconf->M_ALTV) {
			//ALTVOLT supply can be disabled
			static_assert(ALTVOLT_MOSFET_1_GPIO_Port == ALTVOLT_MOSFET_2_GPIO_Port, "ALTVOLT_1 and ALTVOLT_2 must be on the same port");
			HAL_GPIO_WritePin(ALTVOLT_MOSFET_1_GPIO_Port, ALTVOLT_MOSFET_1_Pin | ALTVOLT_MOSFET_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
		}
		if(swconf->R_CH7_12V) {
			//12V supply can be disabled
			rectrl_switch_active(&g_fancy.rectrl, 7);
			HAL_Delay(load_switch_delay);
		}


		if(switch_on_12V && switch_on_ALTV) {
			//1. slowly switch consumers over to ALTV channel
			for(int i=0; i<7; i++) {
				rectrl_switch_active(&g_fancy.rectrl, i);
				HAL_Delay(load_switch_delay);
			}
			//2. enable 12V supply
			rectrl_switch_inactive(&g_fancy.rectrl, 7);

			//3. slowly switch consumers over to 12V channel
			for(int i=0; i<7; i++) {
				rectrl_switch_inactive(&g_fancy.rectrl, i);
				HAL_Delay(load_switch_delay);
			}

			//4. enable ALTVOLT supply
			static_assert(ALTVOLT_MOSFET_1_GPIO_Port == ALTVOLT_MOSFET_2_GPIO_Port, "ALTVOLT_1 and ALTVOLT_2 must be on the same port");
			HAL_GPIO_WritePin(ALTVOLT_MOSFET_1_GPIO_Port, ALTVOLT_MOSFET_1_Pin | ALTVOLT_MOSFET_2_Pin, GPIO_PIN_SET);
			HAL_Delay(load_switch_delay);
		}
		else if(switch_on_12V) {
			//1. slowly switch consumers over to ALTV channel
			for(int i=0; i<7; i++) {
				rectrl_switch_active(&g_fancy.rectrl, i);
				HAL_Delay(load_switch_delay);
			}
			//2. enable 12V supply
			rectrl_switch_inactive(&g_fancy.rectrl, 7);
		}
		else if(switch_on_ALTV) {
			//1. slowly switch consumers over to 12V channel
			for(int i=0; i<7; i++) {
				rectrl_switch_inactive(&g_fancy.rectrl, i);
				HAL_Delay(load_switch_delay);
			}

			//2. enable ALTVOLT supply
			static_assert(ALTVOLT_MOSFET_1_GPIO_Port == ALTVOLT_MOSFET_2_GPIO_Port, "ALTVOLT_1 and ALTVOLT_2 must be on the same port");
			HAL_GPIO_WritePin(ALTVOLT_MOSFET_1_GPIO_Port, ALTVOLT_MOSFET_1_Pin | ALTVOLT_MOSFET_2_Pin, GPIO_PIN_SET);
			HAL_Delay(load_switch_delay);
		}

		//slowly switch consumers to final supply
		if(swconf->R_CH0) { rectrl_switch_active(&g_fancy.rectrl, 0); } else { rectrl_switch_inactive(&g_fancy.rectrl, 0); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH1) { rectrl_switch_active(&g_fancy.rectrl, 1); } else { rectrl_switch_inactive(&g_fancy.rectrl, 1); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH2) { rectrl_switch_active(&g_fancy.rectrl, 2); } else { rectrl_switch_inactive(&g_fancy.rectrl, 2); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH3) { rectrl_switch_active(&g_fancy.rectrl, 3); } else { rectrl_switch_inactive(&g_fancy.rectrl, 3); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH4) { rectrl_switch_active(&g_fancy.rectrl, 4); } else { rectrl_switch_inactive(&g_fancy.rectrl, 4); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH5) { rectrl_switch_active(&g_fancy.rectrl, 5); } else { rectrl_switch_inactive(&g_fancy.rectrl, 5); }
		HAL_Delay(load_switch_delay);
		if(swconf->R_CH6) { rectrl_switch_active(&g_fancy.rectrl, 6); } else { rectrl_switch_inactive(&g_fancy.rectrl, 6); }
		HAL_Delay(load_switch_delay);

		//ch7 consumer is on MOSFETS
		if(swconf->M_CH7_12V) {
			HAL_GPIO_WritePin(CH7_MOSFET_ALTVOLT_GPIO_Port, CH7_MOSFET_ALTVOLT_Pin, GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
			HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port,     CH7_MOSFET_12V_Pin,     GPIO_PIN_SET);
			HAL_Delay(load_switch_delay);
		} else if(swconf->M_CH7_ALTV) {
			HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port,     CH7_MOSFET_12V_Pin,     GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
			HAL_GPIO_WritePin(CH7_MOSFET_ALTVOLT_GPIO_Port, CH7_MOSFET_ALTVOLT_Pin, GPIO_PIN_SET);
			HAL_Delay(load_switch_delay);
		} else {
			HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port,     CH7_MOSFET_12V_Pin,     GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
			HAL_GPIO_WritePin(CH7_MOSFET_ALTVOLT_GPIO_Port, CH7_MOSFET_ALTVOLT_Pin, GPIO_PIN_RESET);
			HAL_Delay(load_switch_delay);
		}
	}
}

static void fancy_panic(void) {
	fancy_transition_switch_state(sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0]) - 1, 0);
}

static void fancy_cycle_switches(void) {
	static int8_t cycle_count = 0;
	static int8_t direction = 1;

	const int old_switch_i = g_fancy.switch_state;

	cycle_count++;
	if(cycle_count == 10) {
		fancy_panic();
	}
	if(cycle_count >= 20) {
		cycle_count = 0;
		g_fancy.switch_state += direction;
	}

	if(g_fancy.switch_state >= (int8_t)(sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0]))) {
		direction = -1;
		g_fancy.switch_state = (int8_t)(sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0]) - 1);
	}
	if(g_fancy.switch_state < 0) {
		direction = 1;
		g_fancy.switch_state = 0;
	}

	fancy_transition_switch_state(old_switch_i, g_fancy.switch_state);
}

static void fancy_read_dht_sensor(void) {
	float humi = 0;
	const bool ok = DHT_readData(&g_dht11, &g_fancy.msm_raw.temp_dht.val, &humi);
	g_fancy.msm_raw.temp_dht.is_valid = ok && (humi != 0);
	g_fancy.msm_raw.temp_dht.time = fancy_gettick();
}

static void fancy_read_adc_sensors(void) {
	struct adctemp_measurement_t result;

	adctemp_measure(&g_fancy.adctemp, &result);

	g_fancy.msm_raw.temp_ntc.val = result.ntc_degC;
	g_fancy.msm_raw.temp_ntc.is_valid = result.ntc_degC_is_valid;
	g_fancy.msm_raw.temp_ntc.time = fancy_gettick();

	g_fancy.msm_raw.temp_core.val = result.core_degC;
	g_fancy.msm_raw.temp_core.is_valid = result.ntc_degC_is_valid;
	g_fancy.msm_raw.temp_core.time = fancy_gettick();

	g_fancy.msm_raw.vdda.val = result.vdda_V;
	g_fancy.msm_raw.vdda.is_valid = result.vdda_V_is_valid;
	g_fancy.msm_raw.vdda.time = fancy_gettick();
}

static void fancy_degraded_alarm(void) {
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 200);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST/2, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST*2, 100);
}

static void fancy_panic_alarm(void) {
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 200);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST/2, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST*2, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 200);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST/2, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST*2, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 100);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST/2, 50);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST*2, 50);
}

static void fancy_cyclic(void) {
	const uint32_t start_time_ms = HAL_GetTick();

	fancy_heartbeat();
	fancy_read_dht_sensor();
	fancy_read_adc_sensors();

	fancy_sensor_range_checks();
	fancy_sensor_filters();
	fancy_sensor_fusion();

	HAL_Delay(1);
	fancy_tm1637_display_update();

	if(fancy_is_degraded()) {
		fancy_degraded_alarm();
	} else if(fancy_is_panicked()) {
		fancy_panic_alarm();
	} else {
		fancy_periodic_alive_sound();
	}

	if(fancy_is_panicked()) {
		const int max_switch_state = sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0])-1;
		fancy_transition_switch_state(g_fancy.switch_state, max_switch_state);
		g_fancy.switch_state = max_switch_state;
	} else if(g_fancy.temp_fused_filtered.is_valid) {
		const int max_switch_state = sizeof(fancy_switchconf)/sizeof(fancy_switchconf[0])-1;
		const int switch_step = fancy_regulate(&g_fancy.regulator, g_fancy.temp_fused_filtered.val);
		//const int switch_step = fancy_regulate(&g_fancy.regulator, g_fancy.msm_filtered.temp_ntc.val); //for testing
		int new_switch_state = g_fancy.switch_state + switch_step;
		if(new_switch_state < 0) {
			new_switch_state = 0;
		}
		if(new_switch_state > max_switch_state) {
			new_switch_state = max_switch_state;
		}
		fancy_transition_switch_state(g_fancy.switch_state, new_switch_state);
		g_fancy.switch_state = new_switch_state;
	}

	fancy_heartbeat();
	//fancy_cycle_switches();

	const uint32_t elapsed_time_ms = HAL_GetTick() - start_time_ms;
	const uint32_t cycle_duration_ms = 1000;
	if(elapsed_time_ms < cycle_duration_ms) {
		const uint32_t remaining_ms = cycle_duration_ms - elapsed_time_ms;
		HAL_Delay(remaining_ms);
	}
}


static void fancy_relay_switch_test(void) {
	const int delay_ms = 1000;

	for(int i=0; i<7; i++) {
		rectrl_switch_active(&g_fancy.rectrl, i);
		HAL_Delay(delay_ms);
	}
	HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port, CH7_MOSFET_12V_Pin, GPIO_PIN_RESET);
	HAL_Delay(delay_ms);

	for(int i=0; i<7; i++) {
		rectrl_switch_inactive(&g_fancy.rectrl, i);
		HAL_Delay(delay_ms);
	}
	HAL_GPIO_WritePin(CH7_MOSFET_12V_GPIO_Port, CH7_MOSFET_12V_Pin, GPIO_PIN_SET);
	HAL_Delay(delay_ms);
}


void fancy(
		TIM_HandleTypeDef *dht11_tim,
		TIM_HandleTypeDef *buzzer_tim,
		uint8_t            buzzer_tim_channel,
		ADC_HandleTypeDef *ntc_adc)
{
	fancy_init(dht11_tim, buzzer_tim, buzzer_tim_channel, ntc_adc);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 200);

	fancy_transition_switch_state(0, 7);
	g_fancy.switch_state = 7;

	//fancy_relay_switch_test();
	while(1) {
		fancy_cyclic();
	}
}
