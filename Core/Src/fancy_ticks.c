#include "fancy_ticks.h"
#include "main.h"

int64_t fancy_gettick(void) {
	static uint32_t hal_last_ticks = 0;
	static int64_t fancy_ticks = 0;

	const uint32_t hal_ticks = HAL_GetTick();

	fancy_ticks += (int32_t)(hal_ticks - hal_last_ticks);
	hal_last_ticks = hal_ticks;

	return fancy_ticks;
}
