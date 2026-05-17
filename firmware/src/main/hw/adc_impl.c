/*
 * adc_impl.c
 *
 *  Created on: 2026. 5. 15.
 *      Author: WANG
 */


#include "adc_impl.h"

adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
volatile DMA_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];

uint16_t adcGetValue(adcSource_e source)
{
    adcGetChannelValues();

#ifdef DEBUG_ADC_CHANNELS
    for (int i = 0 ; i < MIN(4, ARRAYLEN(adcOperatingConfig)) ; i++) {
        if (adcOperatingConfig[i].enabled) {
            debug[i] = adcValues[adcOperatingConfig[i].dmaIndex];
        }
    }
#endif

    if ((unsigned)source >= ADC_SOURCE_COUNT) {
        return 0;
    }

    switch (source) {
#ifdef USE_ADC_INTERNAL
    case ADC_VREFINT:
    case ADC_TEMPSENSOR:
#if ADC_INTERNAL_VBAT4_ENABLED
    case ADC_VBAT4:
#endif
        return adcInternalRead(source);
#endif
    default:
        return adcValues[source];
    }
}
