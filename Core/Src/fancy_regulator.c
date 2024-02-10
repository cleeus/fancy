#include <math.h>
#include "fancy_regulator.h"
#include "fancy_ticks.h"

static const float FANCY_REGULATOR_TARGET = 28.0f;
static const float FANCY_HYSTERESIS_UP_TRESHHOLD = 1.5f;
static const float FANCY_HYSTERESIS_DOWN_TRESHHOLD = 3.0f;

void fancy_regulator_init(struct FancyRegulator_t *rg) {
	rg->state = FRS_STABLE;
}

static int fancy_regulate_stable(struct FancyRegulator_t *rg, float const temperature) {
	if(temperature > (FANCY_REGULATOR_TARGET + FANCY_HYSTERESIS_UP_TRESHHOLD)) {
		rg->state = FRS_UP;
		rg->stime = fancy_gettick();
		return -2;
	} else if(temperature < (FANCY_REGULATOR_TARGET - FANCY_HYSTERESIS_DOWN_TRESHHOLD)) {
		rg->state = FRS_DOWN;
		rg->stime = fancy_gettick();
		return 1;
	} else {
		return 0;
	}
}

static int fancy_regulate_up(struct FancyRegulator_t *rg, float const temperature) {
	const int64_t now = fancy_gettick();
	const int64_t now_duration_ms = now - rg->stime;
	if(now_duration_ms > 30*1000) {
		rg->state = FRS_STABLE;
		rg->stime = fancy_gettick();
	}
	return 0;
}

static int fancy_regulate_down(struct FancyRegulator_t *rg, float const temperature) {
	const int64_t now = fancy_gettick();
	const int64_t now_duration_ms = now - rg->stime;
	if(now_duration_ms > 30*1000) {
		rg->state = FRS_STABLE;
		rg->stime = fancy_gettick();
	}
	return 0;
}

int fancy_regulate(struct FancyRegulator_t *rg, float const temperature) {
	switch(rg->state) {
		case FRS_UP:     return fancy_regulate_up    (rg, temperature);
		case FRS_DOWN:   return fancy_regulate_down  (rg, temperature);
		case FRS_STABLE: //fallthrough
		default: return fancy_regulate_stable(rg, temperature);
	}
}


