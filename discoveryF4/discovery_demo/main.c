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
int ConvertedValue = 0; //Converted value readed from ADC

/* Private function prototypes -----------------------------------------------*/
static uint32_t Demo_USBConfig(void);
//static void TIM4_Config(void);

/* Private functions ---------------------------------------------------------*/
int adc_convert();
void adc_configure();
void GPIO_PIN_INIT(void);

inline int conv2temp(uint16_t value){
  return ( ( ( ( value * 2960 ) / 4096 ) - 760 ) / ( 25 / 10 ) ) + 25;
}

void GPIO_Config(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
/* 使能GPIOC\GPIOF\GPIOG时钟*/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOF, ENABLE);
GPIO_StructInit(&GPIO_InitStructure);
/* 初始化GPIOG的Pin_6为LED输出 */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //指定第六引脚
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //模式为输出
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //频率为快速
//GPIO_Init(GPIOG, &GPIO_InitStructure);      //调用IO初始化函数
GPIO_Init(GPIOE, &GPIO_InitStructure);      //调用IO初始化函数
/* 初始化GPIOG的Pin_9为模拟量输入 */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
GPIO_Init(GPIOF, &GPIO_InitStructure);
/* GPIO引脚复用功能设置 */
//GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);//这相当于M3的开启复用时钟，只配置复用的引脚，
//GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);//               
}

void ADC_Config(void)
{
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); //开ADC时钟
ADC_DeInit();
ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  //精度为12位           
ADC_InitStructure.ADC_ScanConvMode = DISABLE;   //扫描转换模式失能,单通道不用
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //连续转换使能
ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //不用外部触发，软件触发转换
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐，低字节对齐
ADC_InitStructure.ADC_NbrOfConversion = 1;    //规定了顺序进行规则转换的ADC通道的数目
ADC_Init(ADC3, &ADC_InitStructure);      
ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;    //独立模式
ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; //分频为4，f(ADC3)=21M
ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //失能DMA_MODE
ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两次采样间隔5个周期
ADC_CommonInit(&ADC_CommonInitStructure);
ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_15Cycles); //规则通道配置，1表示规则组采样顺序
//ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE); //使能ADC转换结束中断
ADC_Cmd(ADC3, ENABLE);  //使能ADC3
/*********************ADC看门狗配置***************************/
ADC_AnalogWatchdogCmd(ADC3, ADC_AnalogWatchdog_SingleRegEnable);
ADC_AnalogWatchdogThresholdsConfig(ADC3, 0x0E8B, 0x0555);  //阈值设置。高：3V 低：1V
ADC_AnalogWatchdogSingleChannelConfig(ADC3, ADC_Channel_7);
ADC_ITConfig(ADC3, ADC_IT_AWD, ENABLE);
ADC_DMACmd(ADC3, ENABLE);   //使能ADC3的DMA  
ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE); //单通道模式下上次转换完成后DMA请求允许，也就是持续DMA
}

void NVIC_Config()
{
/* DMA中断配置 */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);        //嵌套优先级分组为 1
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;   //嵌套通道为DMA2_Stream0_IRQn
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    //响应优先级为 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //通道中断使能
  NVIC_Init(&NVIC_InitStructure);
  /* ADC中断配置 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);        //嵌套优先级分组为 1
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;           //嵌套通道为ADC_IRQn
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;    //响应优先级为 2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //通道中断使能
  NVIC_Init(&NVIC_InitStructure);
}

void DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  /*首先开DMA2时钟，由407参考手册-RM0090-Reference manual
   *165页可知，UASRT6与DMA2映射，而且DMA2挂载在AHB1时钟总线上*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream0);
  DMA_StructInit( &DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;           //选择Channel_2
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_Addr; //数据传输的外设首地址，详解见上
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCoverVaule;  //自己定义待发送数组的首地址，要强制转换为32位
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;    //数据传输方向选择为外设到内存
  DMA_InitStructure.DMA_BufferSize = 1;                      //传输数据大小为1，单位由以下确定，大小要配合定义的数组类型和外设数据类型
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器自动增加禁止，因为这里只用到了DR数据寄存器
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;    //内存地址自增不允许
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //外设的数据大小，因为ADC6_DR数据寄存器为16为，故选HalfWord
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //这里也选Byte
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;       //DMA传输模式为Circular,将会循环传输
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; //优先级为High
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);      //使能DMA2_Stream0通道
  /* DMA中断开 */
  DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
     {
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
      /*添加用户代码*/
     }
}
/**名称：ADC看门狗中断服务程序
  *作用：ADC输入超过界限产生中断，并点亮LED
  */
void ADC_IRQHandler(void)
{
 GPIO_SetBits(GPIOE,GPIO_Pin_6);
    if (ADC_GetITStatus(ADC3, ADC_IT_AWD) == SET)
     {
      ADC_ClearITPendingBit(ADC3, ADC_IT_AWD);
      ADC_Cmd(ADC3, DISABLE);
     }
  Delay(10);
  //GPIO_ResetBits(GPIOE,GPIO_Pin_6);
}
 

//static uint16_t testing=10;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 
  
  STM_EVAL_LEDInit(LED4);
  
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
    /* Turn on LEDs available on STM32F4-Discovery ---------------------------*/
  STM_EVAL_LEDOn(LED4);

  if (TimingDelay == 0x00)
  {
    /* Turn off LEDs available on STM32F4-Discovery ------------------------*/
    STM_EVAL_LEDOff(LED4);
    
    /* Write PASS code at last word in the flash memory */
    FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);

/* try to use dma*/
    SystemInit();
    RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOE , ENABLE );
    GPIO_PIN_INIT();
    adc_configure();//Start configuration
    GPIO_Config();
    ADC_Config();
    DMA_Config();
    NVIC_Config();
    //GPIO_ResetBits(GPIOG, GPIO_Pin_6); //关闭LED
    ADC_SoftwareStartConv(ADC3);     //如果不是外部触发则必须软件开始转换

    while (1)
     {
      //Delay(0x0ffffff);
      //printf("size of int is %d \n", sizeof(int));  //测试可知32位系统的int占4个字节
      //printf("ADCoverVaule=%04X VolVaule=%d mV\n", ADCoverVaule, ADCoverVaule*3300/4096);  //串口输出电压值
     /*因为DMA工作是独立于CPU之外的，所以在DMA工作的同时CPU可以做其他事*/
     }


/* Try to test ADC.
    //Delay(3000);
    SystemInit();
    RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOE , ENABLE );
    GPIO_PIN_INIT();
    adc_configure();//Start configuration
    
    int flag=0;

    uint16_t USING_PIN[]={GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14};
    while(1){//loop while the board is working
      GPIO_ResetBits(GPIOE, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);

      ConvertedValue = conv2temp(adc_convert());//Read the ADC converted value
      uint16_t sum=0;
      
      register int i;
      for(i=0; i<12; ++i)
        sum|=(ConvertedValue & (1 << i)?USING_PIN[i]:0);
      GPIO_SetBits(GPIOE, sum);
      
      Delay(100);
    }

/* Try to test ADC.*/
  }
  else{}
}
 
void adc_configure(){
  ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
  GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin

  //Clock configuration
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1 is connected to APB2 peripheral bus
  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE);//Clock for the ADC port!! Do not forget about this one ;)

  //Analog input pin configuration
  GPIO_initStructre.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
  GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
  GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
  GPIO_Init(GPIOC, &GPIO_initStructre);

  //ADC structure configuration
  ADC_DeInit();//reset all parameters to their default values
  ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//converted data will be shifted to right
  ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12-bit number whose maximum value is 4095
  ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
  ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;//use timer 1 capture/compare channel 1 for external trigger
  ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
  ADC_init_structure.ADC_NbrOfConversion = 1;//Number of used ADC channels
  ADC_init_structure.ADC_ScanConvMode = DISABLE;//No scan (only one channel)
  ADC_Init(ADC1, &ADC_init_structure);

  ADC_TempSensorVrefintCmd(ENABLE);

  // use channel 16 from ADC1, with sample time 144 cycles
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_144Cycles);

  ADC_Cmd(ADC1, ENABLE);
}

int adc_convert(){
 ADC_SoftwareStartConv(ADC1);//Start the conversion
 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC1); //Return the converted data
}

void GPIO_PIN_INIT(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource3|GPIO_PinSource4|GPIO_PinSource5|GPIO_PinSource6|GPIO_PinSource7|GPIO_PinSource8|GPIO_PinSource9|GPIO_PinSource10|GPIO_PinSource11|GPIO_PinSource12|GPIO_PinSource13|GPIO_PinSource14, GPIO_AF_TIM3);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( GPIOE, &GPIO_InitStructure ); 
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
