
#ifndef ADC_H
#define ADC_H

#include "stdint.h"
#include "em_adc.h"

#define ADC_VDD_CH      adcSingleInpVDD
#define ADC_VSS_CH      adcSingleInpVSS
#define ADC_TEMP_CH     adcSingleInpTemp
#define ADC_CH0         adcSingleInputCh0
#define ADC_CH1         adcSingleInpCh1
#define ADC_CH2         adcSingleInpCh2
#define ADC_CH3         adcSingleInpCh3
#define ADC_CH4         adcSingleInpCh4
#define ADC_CH5         adcSingleInpCh5
#define ADC_VOLTAGE     adcSingleInpCh6
#define ADC_CURRENT     adcSingleInpCh7

void ADC_init(uint8_t channel); //ADC initialization
uint16_t ADC_getValue(uint8_t channel); //Get value in channel
uint16_t ADC_mv(uint32_t adcSample); //Convert to mV
float ADC_Celsius(int32_t adcSample); //Convert

#endif