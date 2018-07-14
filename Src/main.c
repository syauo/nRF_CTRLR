
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>     // for abs() 
#include <string.h>
#include "nrf24l01.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t     receive_value[6];           // ������߽���ֵ
uint8_t     reset_flag = 1;             // ��λ��־
uint8_t     REC_flag = 0;               // ѧϰ��־
uint8_t     rcv_not_ok = 0;             // ���߽���ʧ�ܱ�־
volatile u8_Bits_t  DIO_byte;           // ��ʾ���ֶ˿�״̬���ֽ�

uint16_t    up_len = 0, dwn_len = 0;    // �������½�ѧϰ��¼����
uint16_t    r_index = 0;
uint16_t    tLEDflicker=0;
uint16_t    tLEDflash=0;

uint16_t    trans_value = 0;            // ����ADת��ֵ

run_state_t  ctrlr_State = S_Standby;
REC_data_t   up_rec[256], dwn_rec[256]; // ѧϰ��¼����
volatile  LED_Status_t    LED_Status;   // ָʾ��״̬
volatile  error_status_t  s_error;       
volatile  run_status_t    s_run;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  VALVE_A_Off();
  VALVE_B_Off();
  LED_LAMP_Off();
  LED_STA_Off();
  LED_AUTO_Off();
  HORN_Off();
  ACC_RELAY_Off();
  ENGINE_STOP_Reset();
  ENGINE_START_Reset();

  PWM_Init();
  check_nrf();
  HAL_TIM_Base_Start(&htim2);
  ENCODER_CNT_Set(0);

#if UART_DEBUG
printf("�����ʼ��OK��\r\n");
#endif
    reset_flag = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

         

   
   /* LED ��˸λ bits (����� CANopenNode V1.1)*/
   //10Hz
   if   (++tLEDflicker == 1)    LED_Status.Flickering = LED_ON;
   else if(tLEDflicker >= 2){  LED_Status.Flickering = LED_OFF; tLEDflicker = 0;}
   //
   if   (++tLEDflash == 3)     LED_Status.SingleFlash = LED_ON;
   else if(tLEDflash >= 4){    LED_Status.SingleFlash = LED_OFF; tLEDflash = 0;}
   
   if(s_error.nrf_not_exist==1 || s_error.is_emergency==1)  LED_RD_Turn(LED_Status.Flickering);
   else if(s_error.switch_init_fail==1)                     LED_RD_Turn(LED_Status.Off);
//   else if(s_error.nrf_rcv_fail==1 )                        LED_RD_Turn(LED_Status.Off);
   else                                                     LED_RD_Turn(LED_Status.SingleFlash);
   
    // ѧϰ/�Զ�����ָʾ��
    if(s_run.sys_autorun==1)            LED_GR_Turn(LED_Status.Flickering);
    else if(s_run.rec_success==1)       LED_GR_Turn(LED_Status.On);
    else if(s_run.sys_under_rec==1)     LED_GR_Turn(LED_Status.SingleFlash);
    //else if(s_run.sys_manual==1)        LED_GR_Turn(LED_Status.SingleFlash);
    else                                LED_GR_Turn(LED_Status.Off);
    
   
    rcv_not_ok = rcv_nrf_data();     // ����ң��������
   
    digit_state_update();            // ���ȡ���������������ͣ

    // ����ͣ��ť�������ϵ縴λ ���뼱ͣ״̬
    if(Is_EMER() || reset_flag == 1){
        ctrlr_State = S_Emerg;
    }
/////////////////////////////////////////////////////////////////////////////////////////
            switch(ctrlr_State){
                //////// ��ͣģʽ /////////////////
                case S_Emerg:
#if UART_DEBUG
printf("!!!!��ͣ״̬***************\r\n");
#endif 
                    // ����״̬
                      s_error.is_emergency = 1;
                      
                      TIM1->CCR1 = 0;   // PWM ���0
                      TIM1->CCR4 = 0;
                      
                      VALVE_A_Off();    // �رշ� A �� B
                      VALVE_B_Off();
                
                      analog_state_update(0);
                
                      reset_flag = 0;

                    // �л�
                      if(rcv_not_ok ==1 ||
                         (DIO_byte.Byte_Val & 0x3F)!=0 || 
                         trans_value > MIN_VALID){
                             
                          s_error.switch_init_fail = 1;
#if UART_DEBUG
printf("XXXXXXXX ����δ��λ����\r\n");
#endif
                      }
                      else{
                          s_error.switch_init_fail = 0;
                          s_error.is_emergency = 0;
                          ctrlr_State = S_Standby;
                        }
                    break;
                        
                //////// ����ģʽ /////////////////
                case S_Standby: 
#if UART_DEBUG
printf(">>>>����״̬---------------\r\n");
#endif
                    // ����״̬
                    s_run.sys_autorun = 0;

                    analog_state_update(0);              // �رձ����� AB�� AO���� 
                    ENCODER_CNT_Set(0); // ��������������
                    r_index = 1;
                    // �жϱȽ�����ѧϰ��� �ɹ��� REC_flag |= 0x04
                    if(REC_flag == 0x03){
                        if(abs(up_rec[up_len].enc_cnt - dwn_rec[dwn_len].enc_cnt) <= REC_POS_TLR){
                            REC_flag |= 0x24; // ��λѧϰ�ɹ���־������ֹ���Զ��½�
                            s_run.rec_success = 1;
#if UART_DEBUG
printf("        **** ����˫��ѧϰ�ɹ� ****\r\n");
#endif
                        }else if((REC_flag & 0x04)!=0x04){   
                            REC_flag &= 0xFB;
                            s_run.rec_success = 0;
#if UART_DEBUG
printf("        !!!! ����˫��ѧϰδ�ɹ� !!!!\r\n");
#endif
                        }else
                        s_run.rec_success = 0;
                    }
                    // �л�
                    if(Is_HOPPER_UP() && !Is_AUTORUN()){
                        if(Is_REC_UP()){
                            memset(&up_rec,0,sizeof(REC_data_t)*256); // �������
                        }
                        ctrlr_State = S_Manual_Up;
                    }
                    else 
                        if(Is_HOPPER_DOWN() && !Is_AUTORUN()){
                            if(Is_REC_DWN()){
                                memset(&dwn_rec,0,sizeof(REC_data_t)*256); // �������
                            }
                            ctrlr_State = S_Manual_Down;
                        }
                    else 
                        if(Is_HOPPER_UP() && Is_AUTORUN() && (REC_flag&0x07) == 0x07 &&
                            (REC_flag & 0x10) == 0){
                                ctrlr_State = S_Auto_Up;
                                s_run.rec_success = 0; // ���ѧϰ�ɹ�״̬��
                            }
                    else 
                        if(Is_HOPPER_DOWN() && Is_AUTORUN() && (REC_flag&0x07) == 0x07 &&
                            (REC_flag & 0x20) == 0){
                                ctrlr_State = S_Auto_Down;
                                s_run.rec_success = 0; // ���ѧϰ�ɹ�״̬��
                            }
                    break;
                /////// �ֶ����� /////////////////
                case S_Manual_Up:
#if UART_DEBUG
printf(">>>>�ֶ�����״̬----------\r\n");
#endif
                    // ����״̬    
                    s_run.sys_manual = 1;
                    if(Is_REC_UP()){
                        s_run.sys_under_rec = 1;
                        if(abs(trans_value - up_rec[r_index-1].adc_data)>=REC_STEP){
                            up_rec[r_index].adc_data = trans_value;   // ��¼����ֵ
                            up_rec[r_index].enc_cnt  = ENCODER_CNT_VALUE; // ��¼������ֵ
                            r_index += 1;
                            if(r_index > 254) r_index = 0;
                        }
#if UART_DEBUG
printf("        ����ѧϰ�� [ r_index = %02d, adc = %04d, enc = %05d ]\r\n",r_index,up_rec[r_index-1].adc_data,up_rec[r_index-1].enc_cnt);
#endif
                    }
                    else 
                        s_run.sys_under_rec = 0;
                    
                    analog_state_update(trans_value);
                    // �л�
                    if(!Is_HOPPER_UP()){
                        // �ж��Ƿ�ѧϰ�ɹ�
                        if(r_index>=5 &&
                           //up_rec[r_index-1].adc_data < REC_STEP && 
                           up_rec[r_index].enc_cnt == 0 &&
                           up_rec[0].enc_cnt == 0 &&
                           rec_increment_check(up_rec, r_index) == 1){
                               up_rec[r_index].adc_data = 0;
                               up_rec[r_index].enc_cnt = ENCODER_CNT_VALUE;
                               up_len = r_index;
                               REC_flag |= 0x01;
#if UART_DEBUG
printf("        ��������ѧϰ��ɣ����� OK��\r\n");
#endif
                        }else{
                            REC_flag &= 0xFE;
#if UART_DEBUG
printf("        ��������ѧϰ NG !!!\r\n");
#endif
                        }
                        VALVE_A_Off();     
                        ctrlr_State = S_Standby;
                        s_run.sys_manual = 0;
                    }
                    break;
                /////// �ֶ��½� /////////////////
                case S_Manual_Down:
#if UART_DEBUG
printf(">>>>�ֶ��½�״̬----------\r\n");
#endif
                    // ����״̬    
                    s_run.sys_manual = 1;
                    if(Is_REC_DWN()){
                        s_run.sys_under_rec = 1;
                        if(abs(trans_value - dwn_rec[r_index-1].adc_data)>=REC_STEP){
                            dwn_rec[r_index].adc_data = trans_value;   // ��¼����ֵ
                            dwn_rec[r_index].enc_cnt  = ENCODER_CNT_VALUE; // ��¼������ֵ
                            r_index += 1;
                            if(r_index > 254) r_index = 0;
                        }
#if UART_DEBUG
printf("        �½�ѧϰ�� [ r_index = %02d, adc = %04d, enc = %05d ]\r\n",r_index,dwn_rec[r_index-1].adc_data,dwn_rec[r_index-1].enc_cnt);
#endif
                    }
                    else
                        s_run.sys_under_rec = 0;
                    analog_state_update(trans_value);
                    
                    // �л�
                    if(!Is_HOPPER_DOWN()){
                        // �ж��Ƿ�ѧϰ�ɹ�
                        if(r_index>=5 &&
                           //dwn_rec[r_index-1].adc_data < REC_STEP && 
                           dwn_rec[r_index].enc_cnt == 0 &&
                           dwn_rec[0].enc_cnt == 0 &&
                           rec_increment_check(dwn_rec, r_index) == 1){
                               dwn_rec[r_index].adc_data = 0;
                               dwn_rec[r_index].enc_cnt = ENCODER_CNT_VALUE;
                               dwn_len = r_index;
                               REC_flag |= 0x02;
#if UART_DEBUG
printf("        �½�����ѧϰ��ɣ����� OK��\r\n");
#endif
                        }else{
                            REC_flag &= 0xFD;
#if UART_DEBUG
printf("        �½�����ѧϰ NG !!!\r\n");
#endif
                        }
                        VALVE_B_Off();
                        ctrlr_State = S_Standby;
                        s_run.sys_manual = 0;
                    }
                    break;
                /////// �Զ����� /////////////////
                case S_Auto_Up:
#if UART_DEBUG
printf(">>>>�Զ�����״̬----------\r\n");
#endif
                    // ����״̬
                    s_run.sys_autorun = 1;
                    if(ENCODER_CNT_VALUE <= up_rec[r_index].enc_cnt && 
                       ENCODER_CNT_VALUE >= up_rec[r_index-1].enc_cnt  ){
                           
                            analog_state_update(up_rec[r_index].adc_data);
#if UART_DEBUG
printf("        �Զ������� [ r_index = %02d, adc = %04d, enc = %05d | act %05d]\r\n",r_index,up_rec[r_index].adc_data,up_rec[r_index].enc_cnt,ENCODER_CNT_VALUE);
#endif
                            
                    }
                    if(ENCODER_CNT_VALUE >= up_rec[r_index].enc_cnt||
                       ENCODER_CNT_VALUE <200){
                           if(r_index<254 && r_index<=up_len)
                            r_index++;
                    }
                    // ֹͣ
                    if(up_rec[up_len].enc_cnt <= ENCODER_CNT_VALUE ||
                       r_index >= up_len){
                        
                        analog_state_update(0);
                    }
                    // �л�
                    if(!Is_HOPPER_UP() || !Is_AUTORUN()){
                        
                        REC_flag |= 0x10;
                        REC_flag &= 0xDF;
                        s_run.sys_autorun = 0;
                        VALVE_A_Off();
                        ctrlr_State = S_Standby;
                    }
                    break;
                /////// �Զ��½� /////////////////
                case S_Auto_Down:
#if UART_DEBUG
printf(">>>>�Զ��½�״̬----------\r\n");
#endif
                    // ����״̬
                    s_run.sys_autorun = 1;
                    if(ENCODER_CNT_VALUE <= dwn_rec[r_index].enc_cnt && 
                       ENCODER_CNT_VALUE >= dwn_rec[r_index-1].enc_cnt  ){
                            analog_state_update(dwn_rec[r_index].adc_data);
#if UART_DEBUG
printf("        �Զ��½��� [ r_index = %02d, adc = %04d, enc = %05d | act %05d]\r\n",r_index,dwn_rec[r_index].adc_data,dwn_rec[r_index].enc_cnt,ENCODER_CNT_VALUE);
#endif
                            
                    }
                    if(ENCODER_CNT_VALUE >= dwn_rec[r_index].enc_cnt||
                       ENCODER_CNT_VALUE <200){
                           if(r_index<254 && r_index<=dwn_len)
                            r_index++;
                    }
                    // ֹͣ
                    if(dwn_rec[dwn_len].enc_cnt <= ENCODER_CNT_VALUE ||
                       r_index >= dwn_len ){
                        
                       analog_state_update(0);
                    }
                    // �л�
                    if(!Is_HOPPER_DOWN() || !Is_AUTORUN()){
                        
                        REC_flag |= 0x20;
                        REC_flag &= 0xEF;
                        s_run.sys_autorun = 0;
                        VALVE_B_Off();
                        ctrlr_State = S_Standby;
                    }
                    break;
                /////// ���д��� /////////////////
                default:
#if UART_DEBUG
printf(">>>> ����״̬ת������ ǿ�н������״̬ <<<<\r\n");
#endif
                    ctrlr_State = S_Standby;
                    break;
            }          
/////////////////////////////////////////////////////////////////////////////////////////
#if UART_DEBUG
printf("-------- ������ϵ����� [ %02x%02x - %04d - %05d ]\r\n",receive_value[1],receive_value[2],receive_value[3]*256+receive_value[4],ENCODER_CNT_VALUE);
#endif          



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
