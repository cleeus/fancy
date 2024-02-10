#ifndef INC_FANCY_REGULATOR_H_
#define INC_FANCY_REGULATOR_H_

#include <stdint.h>

enum FancyRegulatorState_e {
	FRS_STABLE = 0,
	FRS_UP = 1,
	FRS_DOWN = 2
};

struct FancyRegulator_t {
	enum FancyRegulatorState_e state;
	int64_t stime;
};


void fancy_regulator_init(struct FancyRegulator_t *rg);
int  fancy_regulate(struct FancyRegulator_t *rg, float const temperature);


#endif /* INC_FANCY_REGULATOR_H_ */
