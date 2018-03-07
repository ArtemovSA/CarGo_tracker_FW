
#include "ADC.h"
#include "em_adc.h"
#include "Delay.h"

#define ADC_12BIT_MAX           4096

uint16_t sample_val;
volatile uint8_t sample_flag = 0;

//--------------------------------------------------------------------------------------------------
//ADC initialization
void ADC_init(uint8_t channel)
{
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

  init.ovsRateSel = adcOvsRateSel2;
  init.lpfMode = adcLPFilterRC;
  init.warmUpMode = adcWarmupNormal;
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(70000, 0);
  init.tailgate = 0;

  ADC_Init(ADC0, &init);

  ADC_InitSingle_TypeDef initsingle = ADC_INITSINGLE_DEFAULT;

  initsingle.prsSel = adcPRSSELCh0;
  initsingle.acqTime = adcAcqTime2;
  initsingle.reference = adcRef2V5;
  initsingle.resolution = adcRes12Bit;
  initsingle.input = (ADC_SingleInput_TypeDef)channel;
  initsingle.diff = 0;
  initsingle.prsEnable = 0;
  initsingle.leftAdjust = 0;
  initsingle.rep = 0;

  ADC_InitSingle(ADC0, &initsingle);
  
  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

  /* Enable ADC interrupt vector in NVIC*/
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  sample_flag = 0;
}
//--------------------------------------------------------------------------------------------------
//Reset
void ADC_data_Reset()
{
  /* Rest ADC registers */
  ADC_Reset(ADC0);
  NVIC_DisableIRQ(ADC0_IRQn);
}
//--------------------------------------------------------------------------------------------------
//Get value in channel
uint16_t ADC_getValue(uint8_t channel)
{
  ADC_init(channel);
  sample_flag = 0;
  ADC_Start(ADC0, adcStartSingle);
  uint16_t try_count = 0;
  
  /* Wait while conversion is active */
  while (1)
  {
    if (sample_flag == 1)
      break;
    if (try_count > 600)
      break;
    try_count++;
  };
  
  ADC_data_Reset();

  return sample_val;
}
//--------------------------------------------------------------------------------------------------
//Convert to mV
uint16_t ADC_mv(uint32_t adcSample)
{
  /* Calculate supply voltage relative to 2.5V reference */
  return (adcSample * 2500) / ADC_12BIT_MAX;
}
//--------------------------------------------------------------------------------------------------
//Convert
float ADC_Celsius(int32_t adcSample)
{
  uint32_t calTemp0;
  uint32_t calValue0;
  int32_t readDiff;
  float temp;

  /* Factory calibration temperature from device information page. */
  calTemp0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
              >> _DEVINFO_CAL_TEMP_SHIFT);

  calValue0 = ((DEVINFO->ADC0CAL2
                /* _DEVINFO_ADC0CAL2_TEMPREAD1V25_MASK is not correct in
                    current CMSIS. This is a 12-bit value, not 16-bit. */
                & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
               >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  if ((calTemp0 == 0xFF) || (calValue0 == 0xFFF))
  {
    /* The temperature sensor is not calibrated */
    return -100.0;
  }

  /* Vref = 1250mV
     TGRAD_ADCTH = 1.835 mV/degC (from datasheet)
  */
  readDiff = calValue0 - adcSample * 2;
  temp     = ((float)readDiff * 1250);
  temp    /= (4096 * -1.835);

  /* Calculate offset from calibration temperature */
  temp     = (float)calTemp0 - temp;
  return temp;
}
//--------------------------------------------------------------------------------------------------
//IRQ
void ADC0_IRQHandler(void)
{
  /* Clear ADC0 interrupt flag */
  ADC_IntClear(ADC0, ADC_IFC_SINGLE);

  /* Read conversion result to clear Single Data Valid flag */
  sample_val = ADC_DataSingleGet(ADC0);
  sample_flag = 1; //Conversation OK
}