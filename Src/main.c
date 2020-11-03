/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "motor.h"
#include "string.h"
#include "ctype.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define RXBUFFERSIZE  256     //???????
char RxBuffer[RXBUFFERSIZE];   //????
uint8_t aRxBuffer;			//??????
uint8_t Uart1_Rx_Cnt = 0;
unsigned char RxBuf;//用于暂存USART1 数据
unsigned char RecFlag = 0;//USART1接收完成标志 1：完成
uint8_t TxData[] = "欢迎来到STM32的世界！"; //发送信息

unsigned char RxData[20]; //USART1串口数据缓存

static unsigned char i=0;
char flag=0;
int voltage=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int plase=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	uint16_t u16TempLeft;   //
  uint16_t u16TempRight;
  int Voltage=0;
  float leftspeed,rightspeed,sumspeed;
  char speed[10];
  float L;      //需要行驶的路程
  int Angle=90,wheelbase=22;
  char disflag=0;   //0表示第一次进入函数，1表示函数在持续运算中
  char turn=1;    //左转或右转
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int fputc(int ch, FILE *f)
//{
//  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}
//int fgetc(FILE *f)
//{
//  uint8_t ch = 0;
//  HAL_UART_Receive(&huart3, &ch, 1, 0xffff);
//  return ch;
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 char ss[11]={'#','#','S','=','0','0','0','0','0','&','&'};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
  //printf("ready\r\n");
	//dianjioutput(300);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //printf("run\r\n");
    //printf("%f\r\n",u16TempLeft);
    //printf("%f\r\n",u16TempRight);
//    //车速部分
    u16TempLeft=__HAL_TIM_GET_COUNTER(&htim3);
		u16TempRight=__HAL_TIM_GET_COUNTER(&htim4);
    leftspeed=u16TempLeft*21.0/33/16;     //此时单位cm/s
    rightspeed=u16TempRight*21.0/33/16;
    if(u16TempLeft>30000)
      leftspeed=(65535-u16TempLeft)*21.0/33/16;
    if(u16TempRight>30000)
      rightspeed=(65535-u16TempRight)*21.0/33/16;
   
		//printf("左侧转速为：%d\r\n",u16TempLeft);
		//printf("右侧转速为：%d\r\n",u16TempRight);
    sumspeed=(leftspeed+rightspeed)/100.0/2;
    sprintf(speed, "%f", sumspeed);

    //speed[0]=sumspeed/10+'0';
    ss[4]=(char)sumspeed%10+'0';
    ss[5]='.';
    ss[6]=(char)(sumspeed*10)%10+'0';
    ss[7]=(char)(sumspeed*100)%10+'0';
    ss[8]=(char)(sumspeed*1000)%10+'0';
//    for(char i=0;i<5;i++)
//    {
//      printf("%c\n",speed[i]);
//    }
    //printf("s=%s\r\n",speed);    //车速，单位m/s，字符串
//    for(i=0;i<11;i++)
//		{
//			HAL_UART_Transmit(&huart3,&ss[i],1,1000);
//			HAL_Delay(2);
//		}
    //车速，单位m/s，字符串
    //printf("s=%f\r\n",sumspeed);    //float型
		//printf("\r\n",rightspeed/100.0);   //右侧车速，单位m/s
//    if(disflag==0)
//    {
//      L=distance_output(Angle,wheelbase);
//      disflag=1;
//    }
//    else
//      L=turning_time_output(L,turn,disflag);
    //printf("剩余路程：%f\r\n",L);
    //printf("\r\n");
    

		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);
    //车速部分结束
    
    if(flag==1)
    {
      HAL_Delay(500);flag=0;
      //printf("**%s&& \r\n",RxData);//接收信息
      if(strstr(RxData,"s0l0r0f0b0&&")!=NULL )    //停
      {
        //printf("s0l0r0f0b0\r\n");
        dianjioutput(0);
      }
      else if(strstr(RxData,"s1l0r0f0b0&&")!=NULL )   //启
      {
        //printf("s1l0r0f0b0\r\n");
        dianjioutput(0);
      }
      else if(strstr(RxData,"s1l0r0f1b0&&")!=NULL )   //前进
      {
        //printf("s1l0r0f0b0\r\n");
        dianjioutput(500);
      }
      else if(strstr(RxData,"s1l0r0f0b1&&")!=NULL )   //后退
      {
        //printf("s1l0r0f0b1\r\n");
        dianjioutput(-300);
      }
      else if(strstr(RxData,"s1l0r1f1b0&&")!=NULL )   //右转
      {
        //printf("s1l0r1f1b0\r\n");
        leftrightoutput(3);
      }
      else if(strstr(RxData,"s1l1r0f1b0&&")!=NULL )   //左转
      {
        //printf("s1l1r0f1b0\r\n");
        leftrightoutput(2);
      }

      memset(RxData,0,sizeof(RxData));
			i=0;
    }
		HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */
void USART1_WRITE_DATA_IN_Buf(unsigned char data)
{
		RxData[i++] = data;
		HAL_UART_Receive_IT(&huart3, &RxBuf, 1);  //重新打开USART1接收中断
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart == &huart3)  //判断是哪个串口产生的中断
	{
		flag=1;
		USART1_WRITE_DATA_IN_Buf(RxBuf); //向USART1串口缓冲区写入数据

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
