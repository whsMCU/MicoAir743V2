/*
 * rtc.h
 *
 *  Created on: 2020. 12. 9.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_RTC_H_
#define SRC_COMMON_HW_INCLUDE_RTC_H_

#include "hw.h"


#ifdef _USE_HW_RTC

bool rtcInit(void);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_RTC_H_ */
