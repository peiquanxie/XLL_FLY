#include "myadc.h"

double ValueAvg;   
uint32_t ulADC0_Value[8];   

void ADC_Init(void)//ADC??ʼ??????   
{    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC0 module.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));// Wait for the ADC0 module to be ready.	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
  //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  //Enable the first sample sequencer to capture the value of channel 0 when
  //the processor trigger occurs.  
  ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH4 |ADC_CTL_END | ADC_CTL_IE );     
  ADCSequenceEnable(ADC0_BASE, 0);   
  //ADCIntClear(ADC0_BASE, 0);   
  //ADCIntEnable(ADC0_BASE, 0);   
}   



float ADC_StartSample(uint8_t num)//ADC??ȡ   
{
  uint32_t data_temp=0;	
  ADCProcessorTrigger(ADC0_BASE, num); // Trigger the sample sequence.  
  while(!ADCIntStatus(ADC0_BASE, num, false)) ; // Wait until the sample sequence has completed.  
  ADCSequenceDataGet(ADC0_BASE, num, &data_temp);// Read the value from the ADC.    
  return (float)(data_temp*3.3/4095.0f);   
}



