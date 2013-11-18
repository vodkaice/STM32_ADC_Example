
/* Reference to stm32 discovery demo example.*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555
#define BUFFERSIZE 128
#define ADC3_DR_Addr   ((uint32_t)0x4001224C)
 
uint16_t ADCConvertedValues[BUFFERSIZE];
__IO uint16_t ADCoverVaule;
uint16_t USING_PIN[]={GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
  
uint16_t PrescalerValue = 0;

__IO uint32_t TimingDelay;
__IO uint8_t DemoEnterCondition = 0x00;
__IO uint8_t UserButtonPressed = 0x00;
LIS302DL_InitTypeDef  LIS302DL_InitStruct;
LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;  
__IO int8_t X_Offset, Y_Offset, Z_Offset  = 0x00;
uint8_t Buffer[6];
volatile int ConvertedValue = 0; //Converted value readed from ADC

/* Private function prototypes -----------------------------------------------*/
static uint32_t Demo_USBConfig(void);
//static void TIM4_Config(void);

/* Private functions ---------------------------------------------------------*/

inline int conv2temp(uint16_t value){
  return ( ( ( ( value * 2960 ) / 4096 ) - 760 ) / ( 25 / 10 ) ) + 25;
}

void ADC_Config(void)
{
    ADC_InitTypeDef ADC_InitStructure; // Structure for single-ADC configuration
    ADC_CommonInitTypeDef ADC_CommonInitStructure; // Structure for inter-ADC configuration

    // Clock configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // ADC1 is connected to APB2 peripheral bus
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE); // Clock for the ADC port!! (do not forget it)

    // ADC structure configuration
    ADC_DeInit(); // Reset all parameters to their default values
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // Input voltage is converted into a 12-bit number whose maximum value is 4095
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // No scan (only one channel)
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // the conversion is continuous (periodic)
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // no external trigger for conversion
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // use timer 1 capture/compare channel 1 for external trigger (may be forced)
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // converted data will be shifted to the right
    ADC_InitStructure.ADC_NbrOfConversion = 1; // Number of used ADC channels
    ADC_Init(ADC1, &ADC_InitStructure);      

    // ADC common structure configuration
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // independent mode
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // f(ADC3)=84/4=21MHz
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // disable DMA_MODE
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // there are 5 clock cycles between 2 samplings
    ADC_CommonInit(&ADC_CommonInitStructure);

    // use channel 10 from ADC1, with sample time 15 cycles
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);

    ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE); // not ready for interrupt
    ADC_Cmd(ADC1, ENABLE);
}

void NVIC_Config()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* ADC interrupt configure */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void GPIO_Output_Config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3|GPIO_PinSource4|GPIO_PinSource5|GPIO_PinSource6|GPIO_PinSource7|GPIO_PinSource8|GPIO_PinSource9|GPIO_PinSource10|GPIO_PinSource11|GPIO_PinSource12|GPIO_PinSource13|GPIO_PinSource14, GPIO_AF_TIM3);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init( GPIOE, &GPIO_InitStructure ); 
}

void GPIO_Input_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Set GPIO clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);

  //Analog input pin configuration
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//volatile int count_interrupt = 10; // count-down counter for interrupts
void ADC_IRQHandler(void)
{

  static int count=0;
  count++;  


  ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);

  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET){
   ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }

  static int flag=0;
  if(count%3==0){
	if(flag==0)
		GPIO_SetBits(GPIOE, GPIO_Pin_8), flag=1;
	else
		GPIO_ResetBits(GPIOE, GPIO_Pin_8), flag=0;

  } 
 
  return;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Configure SysTick */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  while(TimingDelay);

  /* Write PASS code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);

  SystemInit();
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  GPIO_Output_Config();
  GPIO_Input_Config();
  ADC_Config();
  NVIC_Config();

  ADC_SoftwareStartConv(ADC1); // Start conversion by software.
  
  int count=10;
  while (1) {
      GPIO_ResetBits(GPIOE, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
      ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Ready to handle interrupt.
      
      if(count<0) count=10;

      
      uint16_t sum = 0;
      
      register int i;
      for(i=0; i<4; ++i)
         sum|=(count & (1 << i)?USING_PIN[i]:0);
  
      GPIO_SetBits(GPIOE, sum);

      count--;
      //ConvertedValue = ADC_GetConversionValue(ADC1);
      Delay(300);
  }
}

/**
  * @brief  Initializes the USB for the demonstration application.
  * @param  None
  * @retval None
  */
static uint32_t Demo_USBConfig(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);
  
  return 0;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */ 
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
  
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    Delay(5);
  }
}

/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured during Test program execution */
  if (DemoEnterCondition == 0x00)
  {
    /* Timeout error occured for SPI TXE/RXNE flags waiting loops.*/
    Fail_Handler();    
  }
  /* MEMS Accelerometer Timeout error occured during Demo execution */
  else
  {
    while (1)
    {   
    }
  }
  return 0;  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
