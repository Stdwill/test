#include "motor.h"
extern TIM_HandleTypeDef htim2;
extern float leftspeed,rightspeed;
extern float L;

void dianjioutput(int Voltage)
{
	//printf("the dianjioutput Voltage:%d\r\n",Voltage);
	if(Voltage>0)
	{
	    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	     //M2电机控制模块AIN1端 PB14		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);      //M2电机控制模块AIN2端 PB15	

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);			   //M1电机控制模块BIN2端 PB13		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);        //M1电机控制模块BIN1端 PB12	
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);  	     //M2电机控制模块AIN2端 PB15			    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);        //M2电机控制模块AIN1端 PB14	
  
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);  			   //M1电机控制模块BIN1端 PB12		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);       //M1电机控制模块BIN2端 PB13	
			
			Voltage=-Voltage;		
	}
	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Voltage);//TIM2 channel3与 Voltage对比，不相同则翻转波形，调节PWM占空比
	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Voltage);//TIM2 channel4与 Voltage对比，不相同则翻转波形，调节PWM占空比
}
void leftrightoutput(int Turn)
{
	//printf("the dianjioutput Voltage:%d\r\n",Turn);
	if(Turn==2)
	{
	    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	     //M2电机控制模块AIN1端 PB14		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);      //M2电机控制模块AIN2端 PB15	

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);			   //M1电机控制模块BIN2端 PB13		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);        //M1电机控制模块BIN1端 PB12	
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,600);//TIM2 channel3与 Voltage对比，不相同则翻转波形，调节PWM占空比
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);//TIM2 channel4与 Voltage对比，不相同则翻转波形，调节PWM占空比
	}
	else if(Turn==3)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);  	     //M2电机控制模块AIN2端 PB15			    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);        //M2电机控制模块AIN1端 PB14	
  
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);  			   //M1电机控制模块BIN1端 PB12		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);       //M1电机控制模块BIN2端 PB13	
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);//TIM2 channel3与 Voltage对比，不相同则翻转波形，调节PWM占空比
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,600);//TIM2 channel4与 Voltage对比，不相同则翻转波形，调节PWM占空比
			
	}

}
float distance_output(int Angle,int wheelbase)    //计算路程
{
  L=Angle*wheelbase*3.14/180;     //计量单位和轮距一致，cm或者mm
  return L;
}
float turning_time_output(int L,int Turn,int flag)    //执行转弯函数,L路程，turn左转右转，flag是否第一次进入函数
{
  //int r=34;     //r车轮半径mm
  //float L;      //需要行驶的路程
  //char flag;    //判断是否第一次进入函数，0是1否。
  if(Turn==0)
  {
    L=L-leftspeed;
    
  }
  else
  {
    L=L-rightspeed;
    
  }
  if(L<=0)
  {
    L=0;
    flag=0;
    
  }
  return L;
}

