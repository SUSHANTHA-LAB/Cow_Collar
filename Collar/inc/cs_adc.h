/*
 * cd_adc.h
 *
 *  Created on: May 14, 2025
 *      Author: sushantha
 */

#ifndef CS_ADC_H_
#define CS_ADC_H_


#include "em_iadc.h"
#include "sl_device_clock.h"
#include "app_assert.h"
#include "sl_clock_manager.h"

void adc_init(void);

uint16_t get_adc_sample(void);

void adc_deinit(void);

#endif /* CS_ADC_H_ */
