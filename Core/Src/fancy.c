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

static void fancy_init(TIM_HandleTypeDef *dht11_tim) {
	tm1637_init(&g_tm1637, TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, TM1637_DIO_GPIO_Port, TM1637_DIO_Pin);
	tm1637_show_zero(&g_tm1637, true);

	DHT_init(&g_dht11, DHT_Type_DHT11, dht11_tim, 8 /*MHz system clock*/, DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);

	g_fancy.is_initialzed = true;
}

static void fancy_heartbeat(void) {
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
}

static void fancy_tm1637_display_update() {
	static int32_t count = 0;
	tm1637_brightness(&g_tm1637, 3);
	//tm1637_write_int(&g_tm1637, g_fancy.temp, 0);
	//uint8_t small_t = 0x0f;
	//tm1637_write_segment(&g_tm1637, &small_t, sizeof(small_t), 0);
	//tm1637_write_float(&g_tm1637, g_fancy.temp, 1, 0);
	tm1637_write_fractional(&g_tm1637, g_fancy.temp, 1, 0);
	count++;
}

static void fancy_cyclic(void) {
	fancy_heartbeat();
	DHT_readData(&g_dht11, &g_fancy.temp, &g_fancy.humi);
	fancy_tm1637_display_update();
  HAL_Delay(700);
}

void fancy(TIM_HandleTypeDef *dht11_tim) {
	fancy_init(dht11_tim);

	while(1) {
		fancy_cyclic();
	}
}
