/*
 * cs_adc.c
 *
 *  Created on: May 14, 2025
 *      Author: sushantha
 */

#include "cs_adc.h"

void adc_init(void)
{
  sl_status_t sc;

  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t all_configs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t init_single = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t input = IADC_SINGLEINPUT_DEFAULT;

  sc = sl_clock_manager_enable_bus_clock(SL_BUS_CLOCK_IADC0);
  app_assert_status(sc);
  sc = sl_clock_manager_enable_bus_clock(SL_BUS_CLOCK_PRS);
  app_assert_status(sc);

  // Only configure the ADC if it is not already running
  if ( IADC0->CTRL == _IADC_CTRL_RESETVALUE ) {
    IADC_init(IADC0, &init, &all_configs);
  }

  input.posInput = iadcPosInputAvdd;

  IADC_initSingle(IADC0, &init_single, &input);
  IADC_enableInt(IADC0, IADC_IEN_SINGLEDONE);


}

uint16_t get_adc_sample(void)
{
#if defined(ADC_PRESENT)
  // Start conversion and wait for result
  ADC_Start(ADC0, adcStartSingle);
  while ( !(ADC_IntGet(ADC0) & ADC_IF_SINGLE) ) ;

  return ADC_DataSingleGet(ADC0);
#elif defined(IADC_PRESENT)
  // Clear single done interrupt
  IADC_clearInt(IADC0, IADC_IF_SINGLEDONE);

  // Start conversion and wait for result
  IADC_command(IADC0, iadcCmdStartSingle);
  while ( !(IADC_getInt(IADC0) & IADC_IF_SINGLEDONE) ) ;

  return IADC_readSingleData(IADC0);
#endif
}


void adc_deinit(void){
  IADC_reset(IADC0);
}
