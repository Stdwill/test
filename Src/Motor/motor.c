#include "motor.h"
extern TIM_HandleTypeDef htim2;
extern float leftspeed,rightspeed;
extern float L;

void dianjioutput(int Voltage)
{
	//printf("the dianjioutput Voltage:%d\r\n",Voltage);
	if(Voltage>0)
	{
	    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	     //M2�������ģ��AIN1�� PB14		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);      //M2�������ģ��AIN2�� PB15	

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);			   //M1�������ģ��BIN2�� PB13		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);        //M1�������ģ��BIN1�� PB12	
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);  	     //M2�������ģ��AIN2�� PB15			    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);        //M2�������ģ��AIN1�� PB14	
  
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);  			   //M1�������ģ��BIN1�� PB12		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);       //M1�������ģ��BIN2�� PB13	
			
			Voltage=-Voltage;		
	}
	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Voltage);//TIM2 channel3�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Voltage);//TIM2 channel4�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
}
void leftrightoutput(int Turn)
{
	//printf("the dianjioutput Voltage:%d\r\n",Turn);
	if(Turn==2)
	{
	    
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	     //M2�������ģ��AIN1�� PB14		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);      //M2�������ģ��AIN2�� PB15	

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);			   //M1�������ģ��BIN2�� PB13		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);        //M1�������ģ��BIN1�� PB12	
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,600);//TIM2 channel3�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);//TIM2 channel4�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	}
	else if(Turn==3)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);  	     //M2�������ģ��AIN2�� PB15			    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);        //M2�������ģ��AIN1�� PB14	
  
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);  			   //M1�������ģ��BIN1�� PB12		    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);       //M1�������ģ��BIN2�� PB13	
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);//TIM2 channel3�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,600);//TIM2 channel4�� Voltage�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
			
	}

}
float distance_output(int Angle,int wheelbase)    //����·��
{
  L=Angle*wheelbase*3.14/180;     //������λ���־�һ�£�cm����mm
  return L;
}
float turning_time_output(int L,int Turn,int flag)    //ִ��ת�亯��,L·�̣�turn��ת��ת��flag�Ƿ��һ�ν��뺯��
{
  //int r=34;     //r���ְ뾶mm
  //float L;      //��Ҫ��ʻ��·��
  //char flag;    //�ж��Ƿ��һ�ν��뺯����0��1��
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

