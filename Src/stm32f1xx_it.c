/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
extern char* ATCMD_PING     ;
extern char* ATCMD_SLEEP    ;
extern char* ATCMD_BAT_OK   ;
extern char* ATCMD_BAT_LOW  ;
extern char* ATCMD_PA1_EXTI1;
extern char* ATCMD_PA4_EXTI4;
extern char* ATCMD_PA5_EXTI5;
extern char* ATCMD_PA6_EXTI6;
extern char* ATCMD_PA7_EXTI7;

RTC_TimeTypeDef rtc_time;
extern int8_t get_PVD(void);
extern int8_t tran_ATCMD(const char* cmd, uint16_t timeOut);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_ON);
  tran_ATCMD(ATCMD_PING, ATCMD_TO_GEN);
  tran_ATCMD(ATCMD_PA1_EXTI1, ATCMD_TO_TX);
  tran_ATCMD(ATCMD_SLEEP, ATCMD_TO_GEN);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_OFF);
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_ON);
  tran_ATCMD(ATCMD_PING, ATCMD_TO_GEN);
  tran_ATCMD(ATCMD_PA4_EXTI4, ATCMD_TO_TX);
  tran_ATCMD(ATCMD_SLEEP, ATCMD_TO_GEN);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_OFF);
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_ON);
  tran_ATCMD(ATCMD_PING, ATCMD_TO_GEN);
  if(EXTI->PR&(0x1U<<5))
    tran_ATCMD(ATCMD_PA5_EXTI5, ATCMD_TO_TX);
  else if(EXTI->PR&(0x1U<<6))
    tran_ATCMD(ATCMD_PA6_EXTI6, ATCMD_TO_TX);
  else if(EXTI->PR&(0x1U<<7))
    tran_ATCMD(ATCMD_PA7_EXTI7, ATCMD_TO_TX);
  tran_ATCMD(ATCMD_SLEEP, ATCMD_TO_GEN);  //tran_ATCMD("AT$CMD\r\n", timeout);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_OFF);
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles RTC alarm interrupt through EXTI line 17.
*/
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */
  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_ON);
  //printf("Alarm triggered at %02d:%02d:%02d\n", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
  RTC_AlarmTypeDef alarm; //this takes 1 second with 2MHz main clock.
  alarm.AlarmTime=rtc_time;
  //for minutes/hours alarm, second is neglectable,
  alarm.AlarmTime.Seconds=0;//set to 0 for compensation
  alarm.AlarmTime.Hours+=1;
  if(alarm.AlarmTime.Hours >= 24)
    alarm.AlarmTime.Hours=alarm.AlarmTime.Hours-24;
  /*alarm.AlarmTime.Minutes+=1;
  if(alarm.AlarmTime.Minutes>=60){
    alarm.AlarmTime.Minutes=0;
    alarm.AlarmTime.Hours+=1;
  }*/
  alarm.Alarm=RTC_ALARM_A; //only 1 alarm available for stm32f103c8
  HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
  
  tran_ATCMD(ATCMD_PING, ATCMD_TO_GEN);
  tran_ATCMD(get_PVD()==-1?ATCMD_BAT_OK:ATCMD_BAT_LOW, ATCMD_TO_TX);
  tran_ATCMD(ATCMD_SLEEP, ATCMD_TO_GEN);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_OFF);
  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);// alarm can wakes up MCU from stop mode
  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
