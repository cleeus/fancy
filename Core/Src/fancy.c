/*
 * fancy.c
 *
 *  Created on: Jan 13, 2024
 *      Author: kai.dietrich
 */
#include <stdbool.h>
#include <assert.h>
#include "main.h"
#include "tm1637.h"
#include "dht.h"
#include "fancy_rectrl.h"
#include "fancy_ntc.h"

static struct Fancy_t {
	bool is_initialzed;

	struct {
		float val;
		bool is_valid;
	} temp_dht;

	struct {
		float val;
		bool is_valid;
	} temp_ntc;

	rectrl_t rectrl;
	int8_t switch_state;

	TIM_HandleTypeDef *buzzer_tim;
	uint8_t            buzzer_tim_channel;

	struct ntc_t ntc;

} g_fancy = {0};

//static tm1637_t g_tm1637;
static tm1637_t g_tm1637_INV;
static DHT_t    g_dht11;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(!g_fancy.is_initialzed) {
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

	ntc_init(&g_fancy.ntc, ntc_adc);

	g_fancy.is_initialzed = true;
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
			if(g_fancy.temp_dht.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 'd', g_fancy.temp_dht.val, 1, 0);
			} else {
				const char d_invalid[] = "d---";
				tm1637_write_str(&g_tm1637_INV, d_invalid, sizeof(d_invalid), 0);
			}
			break;
		case 1:
			if(g_fancy.temp_ntc.is_valid) {
				tm1637_write_fractional(&g_tm1637_INV, 't', g_fancy.temp_ntc.val, 1, 0);
			} else {
				const char t_invalid[] = "t---";
				tm1637_write_str(&g_tm1637_INV, t_invalid, sizeof(t_invalid), 0);
			}
			break;
		default:
			value_cycle = 0;
			break;
	}
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
	const bool ok = DHT_readData(&g_dht11, &g_fancy.temp_dht.val, &humi);
	g_fancy.temp_dht.is_valid = ok && (humi != 0);
}

static void fancy_read_ntc_sensor(void) {
	g_fancy.temp_ntc.val = ntc_measure_degC(&g_fancy.ntc);
	g_fancy.temp_ntc.is_valid = ntc_is_valid(g_fancy.temp_ntc.val);
}

static void fancy_cyclic(void) {
	const uint32_t start_time_ms = HAL_GetTick();

	fancy_heartbeat();
	fancy_read_dht_sensor();
	fancy_read_ntc_sensor();
	fancy_tm1637_display_update();
	fancy_periodic_alive_sound();
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
	//fancy_relay_switch_test();
	while(1) {
		fancy_cyclic();
	}
}
