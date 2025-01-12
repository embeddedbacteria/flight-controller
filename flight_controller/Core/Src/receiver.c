/*
 * receiver.c
 *
 *  Created on: Dec 18, 2024
 *      Author: SinlessKid
 */

#include "stm32f4xx_it.h"

#include "receiver.h"






extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;




volatile uint32_t IC_Val1_CH1 = 0, IC_Val2_CH1 = 0;
volatile uint32_t IC_Val1_CH2 = 0, IC_Val2_CH2 = 0;
volatile uint32_t IC_Val1_CH3 = 0, IC_Val2_CH3 = 0;
volatile uint32_t IC_Val1_CH4 = 0, IC_Val2_CH4 = 0;
volatile uint32_t IC_Val1_CH5 = 0, IC_Val2_CH5 = 0;
volatile uint32_t IC_Val1_CH6 = 0, IC_Val2_CH6 = 0;
volatile uint32_t IC_Val1_CH7 = 0, IC_Val2_CH7 = 0;
volatile uint32_t IC_Val1_CH8 = 0, IC_Val2_CH8 = 0;

volatile uint32_t Difference_CH1 = 0, Difference_CH2 = 0, Difference_CH3 = 0, Difference_CH4 = 0, Difference_CH5 = 0 , Difference_CH6 = 0, Difference_CH7 = 0, Difference_CH8 = 0;
uint8_t Is_First_Captured_CH1 = 0, Is_First_Captured_CH2 = 0;
uint8_t Is_First_Captured_CH3 = 0, Is_First_Captured_CH4 = 0;
uint8_t Is_First_Captured_CH5 = 0, Is_First_Captured_CH6 = 0;
uint8_t Is_First_Captured_CH7 = 0, Is_First_Captured_CH8 = 0;

volatile uint32_t Frequency_CH1 = 0, Frequency_CH2 = 0;
volatile uint32_t Frequency_CH3 = 0, Frequency_CH4 = 0;
volatile uint32_t Frequency_CH5 = 0, Frequency_CH6 = 0;
volatile uint32_t Frequency_CH7 = 0, Frequency_CH8 = 0;

void Receiver_Init()
{
	/*------- PWM READ AREAD--------*/

	  // TIM2 and TIM5 Interrupt Start
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);

	 /*------------------------------------*/

	  /*----- PWM OUTPUT AREA----------*/

	  //TIM3 PWM START

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	  //TIM4 PWM START
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);


	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim ==&htim2)
	{

	/* Channel 1 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	        if (Is_First_Captured_CH1 == 0)
	        {
	            IC_Val1_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            Is_First_Captured_CH1 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH1 > IC_Val1_CH1)
	            	{
	            		Difference_CH1 = IC_Val2_CH1 - IC_Val1_CH1;
	                 }
	            else
	                {
	                            Difference_CH1 = (0xFFFF - IC_Val1_CH1) + IC_Val2_CH1 + 1;
	                }
	            Frequency_CH1 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH1);
	            Is_First_Captured_CH1 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 2 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	    {
	        if (Is_First_Captured_CH2 == 0)
	        {
	            IC_Val1_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	            Is_First_Captured_CH2 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH2 > IC_Val1_CH2)
	            	            	{
	            	            		Difference_CH2 = IC_Val2_CH2 - IC_Val1_CH2;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH2 = (0xFFFF - IC_Val1_CH2) + IC_Val2_CH2 + 1;
	            	                }
	            	            Frequency_CH2 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH2);
	            Is_First_Captured_CH2 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 3 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	    {
	        if (Is_First_Captured_CH3 == 0)
	        {
	            IC_Val1_CH3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            Is_First_Captured_CH3 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH3 > IC_Val1_CH3)
	            	            	{
	            	            		Difference_CH3 = IC_Val2_CH3 - IC_Val1_CH3;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH3 = (0xFFFF - IC_Val1_CH3) + IC_Val2_CH3 + 1;
	            	                }
	            	            Frequency_CH3 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH3);
	            Is_First_Captured_CH3 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 4 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	    {
	        if (Is_First_Captured_CH4 == 0)
	        {
	            IC_Val1_CH4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	            Is_First_Captured_CH4 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH4 > IC_Val1_CH4)
	            	            	{
	            	            		Difference_CH4 = IC_Val2_CH4 - IC_Val1_CH4;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH4 = (0xFFFF - IC_Val1_CH4) + IC_Val2_CH4 + 1;
	            	                }
	            	            Frequency_CH4 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH4);
	            Is_First_Captured_CH4 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

}
	else if(htim ==&htim5)
		{

		/* Channel 1 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		    {
		        if (Is_First_Captured_CH5 == 0)
		        {
		            IC_Val1_CH5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		            Is_First_Captured_CH5 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH5 > IC_Val1_CH5)
		            	{
		            		Difference_CH5 = IC_Val2_CH5 - IC_Val1_CH5;
		                 }
		            else
		                {
		                            Difference_CH5 = (0xFFFF - IC_Val1_CH5) + IC_Val2_CH5 + 1;
		                }
		            Frequency_CH5 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH5);
		            Is_First_Captured_CH5 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

		    /* Channel 2 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		    {
		        if (Is_First_Captured_CH6 == 0)
		        {
		            IC_Val1_CH6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		            Is_First_Captured_CH6 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH6 > IC_Val1_CH6)
		            	            	{
		            	            		Difference_CH6 = IC_Val2_CH6 - IC_Val1_CH6;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH6 = (0xFFFF - IC_Val1_CH6) + IC_Val2_CH6 + 1;
		            	                }
		            	            Frequency_CH6 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH6);
		            Is_First_Captured_CH6 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		        }



		    }

		    /* Channel 3 */

		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		    {
		        if (Is_First_Captured_CH7 == 0)
		        {
		            IC_Val1_CH7 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		            Is_First_Captured_CH7 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH7 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH7 > IC_Val1_CH7)
		            	            	{
		            	            		Difference_CH7 = IC_Val2_CH7 - IC_Val1_CH7;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH7 = (0xFFFF - IC_Val1_CH7) + IC_Val2_CH7 + 1;
		            	                }
		            	            Frequency_CH7 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH7);
		            Is_First_Captured_CH7 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

		    /* Channel 4 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		    {
		        if (Is_First_Captured_CH8 == 0)
		        {
		            IC_Val1_CH8 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		            Is_First_Captured_CH8 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH8 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH8 > IC_Val1_CH8)
		            	            	{
		            	            		Difference_CH8 = IC_Val2_CH8 - IC_Val1_CH8;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH8 = (0xFFFF - IC_Val1_CH8) + IC_Val2_CH8 + 1;
		            	                }
		            	            Frequency_CH8 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH8);
		            Is_First_Captured_CH8 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

	}








	Update_PwmValues();
}




void Update_PwmValues(void)
{

if(Difference_CH1<600 || Difference_CH1>2000)
	Difference_CH1 =0;
if(Difference_CH2<600 || Difference_CH2>2000)
	Difference_CH2 =0;
if(Difference_CH3<600 || Difference_CH3>2000)
	Difference_CH3 =0;
if(Difference_CH4<600 || Difference_CH4>2000)
	Difference_CH4 =0;
if(Difference_CH5<600 || Difference_CH5>2000)
	Difference_CH5 =0;
if(Difference_CH6<600 || Difference_CH6>2000)
	Difference_CH6 =0;
if(Difference_CH7<600 || Difference_CH7>2000)
	Difference_CH7 =0;
if(Difference_CH8<600 || Difference_CH8>2000)
	Difference_CH8 =0;

	      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Difference_CH1);
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Difference_CH2);
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Difference_CH3);
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,Difference_CH4);



		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Difference_CH5);
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Difference_CH6);
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Difference_CH7);
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Difference_CH8);



}
