/*
 * core.h
 *
 *  Created on: 2026. 10. 12.
 *      Author: 왕학승
 */

#ifndef SRC_MAIN_FC_CORE_H_
#define SRC_MAIN_FC_CORE_H_

#include "hw.h"

#include "common/time.h"

void taskGyroSample(timeUs_t currentTimeUs);

bool gyroFilterReady(void);

bool pidLoopReady(void);

#endif /* SRC_MAIN_FC_CORE_H_ */
