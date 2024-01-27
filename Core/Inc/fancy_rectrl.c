#include "fancy_rectrl.h"
#include "main.h"

struct rectrl_pin_t {
	GPIO_TypeDef *port;
	uint16_t      pin;
};

static const struct rectrl_pin_t rectrl_pin_list[] = {
		[0] = {.port = RELAY_K1_GPIO_Port, .pin = RELAY_K1_Pin},
		[1] = {.port = RELAY_K2_GPIO_Port, .pin = RELAY_K2_Pin},
		[2] = {.port = RELAY_K3_GPIO_Port, .pin = RELAY_K3_Pin},
		[3] = {.port = RELAY_K4_GPIO_Port, .pin = RELAY_K4_Pin},
		[4] = {.port = RELAY_K5_GPIO_Port, .pin = RELAY_K5_Pin},
		[5] = {.port = RELAY_K6_GPIO_Port, .pin = RELAY_K6_Pin},
		[6] = {.port = RELAY_K7_GPIO_Port, .pin = RELAY_K7_Pin},
		[7] = {.port = RELAY_K8_GPIO_Port, .pin = RELAY_K8_Pin},
};


void rectrl_init(rectrl_t *rectrl) {
	rectrl->output_enable = false;
	rectrl->switch_state = RECTRL_ALL_INACTIVE;
	rectrl->is_initialized = true;

	rectrl_switch(rectrl, RECTRL_ALL_INACTIVE);

	HAL_GPIO_WritePin(RELAY_OE_GPIO_Port, RELAY_OE_Pin, GPIO_PIN_SET);
	rectrl->output_enable = true;
}

void rectrl_switch_active(rectrl_t *rectrl, const int relay_index) {
	if(!rectrl->is_initialized) {
		return;
	}
	if(relay_index < 0 || relay_index > 7) {
		return;
	}

	rectrl->switch_state |= (0x01 << relay_index) & 0xFF;

	HAL_GPIO_WritePin(rectrl_pin_list[relay_index].port, rectrl_pin_list[relay_index].pin, GPIO_PIN_RESET);
}

void rectrl_switch_inactive(rectrl_t *rectrl, const int relay_index) {
	if(!rectrl->is_initialized) {
		return;
	}

	if(relay_index < 0 || relay_index > 7) {
		return;
	}

	rectrl->switch_state &= (~(0x01 << relay_index)) & 0xFF;

	HAL_GPIO_WritePin(rectrl_pin_list[relay_index].port, rectrl_pin_list[relay_index].pin, GPIO_PIN_SET);
}


void rectrl_switch(rectrl_t *rectrl, const uint8_t switch_state) {
	if(!rectrl->is_initialized) {
		return;
	}

	rectrl->switch_state = switch_state;

	for(int i=0; i<8; i++) {
		const GPIO_PinState pin_state = ((switch_state >> i) & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		HAL_GPIO_WritePin(rectrl_pin_list[i].port, rectrl_pin_list[i].pin, pin_state);
	}
}
