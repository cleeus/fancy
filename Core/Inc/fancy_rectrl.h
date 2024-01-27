#ifndef INC_FANCY_RECTRL_H_
#define INC_FANCY_RECTRL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool is_initialized;
	bool output_enable;
	uint8_t switch_state;
} rectrl_t;

enum rectrl_switch_e {
	RECTRL_ALL_INACTIVE = 0x00,
	RECTRL_RE0_ACTIVE = 0x01,
	RECTRL_RE1_ACTIVE = 0x02,
	RECTRL_RE2_ACTIVE = 0x04,
	RECTRL_RE3_ACTIVE = 0x08,
	RECTRL_RE4_ACTIVE = 0x10,
	RECTRL_RE5_ACTIVE = 0x20,
	RECTRL_RE6_ACTIVE = 0x40,
	RECTRL_RE7_ACTIVE = 0x80,
	RECTRL_ALL_ACTIVE = 0xFF,
};

void rectrl_init(rectrl_t *rectrl);
void rectrl_switch_active  (rectrl_t *rectrl, const int relay_index);
void rectrl_switch_inactive(rectrl_t *rectrl, const int relay_index);
void rectrl_switch         (rectrl_t *rectrl, const uint8_t switch_state);

#endif
