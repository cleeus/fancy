/*
 * fancy.c
 *
 *  Created on: Jan 13, 2024
 *      Author: kai.dietrich
 */
#include <stdbool.h>
#include "main.h"
#include "tm1637.h"
#include "dht.h"

static struct Fancy_t {
	bool is_initialzed;
	float temp;
	float humi;

	TIM_HandleTypeDef *buzzer_tim;
	uint8_t            buzzer_tim_channel;
} g_fancy = {0};

static tm1637_t g_tm1637;
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
		uint8_t            buzzer_tim_channel)
{
	tm1637_init(&g_tm1637, TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, TM1637_DIO_GPIO_Port, TM1637_DIO_Pin);
	tm1637_show_zero(&g_tm1637, true);

	DHT_init(&g_dht11, DHT_Type_DHT11, dht11_tim, 8 /*MHz system clock*/, DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);

	g_fancy.buzzer_tim = buzzer_tim;
	g_fancy.buzzer_tim_channel = buzzer_tim_channel;
	g_fancy.is_initialzed = true;
}

static void fancy_heartbeat(void) {
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_RESET);
  HAL_Delay(40);
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
}

static void fancy_tm1637_display_update(void) {
	tm1637_brightness(&g_tm1637, 2);
	tm1637_write_fractional(&g_tm1637, g_fancy.temp, 1, 0);
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
	if(cycle_count >= 5 /* seconds */) {
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE1, 30);
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE2, 30);
		fancy_buzzer_sound(BUZZER_FREQ_ALIVE1, 30);
		cycle_count = 0;
	}
}

static void fancy_cyclic(void) {
	const uint32_t start_time_ms = HAL_GetTick();

	fancy_heartbeat();
	DHT_readData(&g_dht11, &g_fancy.temp, &g_fancy.humi);
	fancy_tm1637_display_update();
	fancy_periodic_alive_sound();
	fancy_heartbeat();

	const uint32_t elapsed_time_ms = HAL_GetTick() - start_time_ms;
	const uint32_t cycle_duration_ms = 1000;
	if(elapsed_time_ms < cycle_duration_ms) {
		const uint32_t remaining_ms = cycle_duration_ms - elapsed_time_ms;
		HAL_Delay(remaining_ms);
	}
}

void fancy(
		TIM_HandleTypeDef *dht11_tim,
		TIM_HandleTypeDef *buzzer_tim,
		uint8_t            buzzer_tim_channel)
{
	fancy_init(dht11_tim, buzzer_tim, buzzer_tim_channel);
	fancy_buzzer_sound(BUZZER_FREQ_LOUDEST, 200);
	while(1) {
		fancy_cyclic();
	}
}
