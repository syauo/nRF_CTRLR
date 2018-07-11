
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
uint8_t receive_value[6];       // 存放无线接收值
uint8_t reset_flag;             // 复位标志
uint32_t trans_value;           // 转换值
volatile u8_Bits_t DIO_byte;    // 表示数字端口状态的字节

uint8_t  REC_flag = 0;          // 学习标志
uint16_t up_len = 0, dwn_len = 0;   // 上升、下降学习记录长度

//uint16_t pwm_value=0, acc_value=0;
REC_data_t   up_rec[256], dwn_rec[256]; // 学习记录数组
volatile  LED_Status_t   LED_Status;    // 指示灯状态
volatile error_status_t  s_error;       
volatile run_status_t    s_run;
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
static uint16_t    index = 0;
run_state_t ctrlr_State = S_Standby;
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

lable_check:
    PWM_Init();
    check_nrf();
    HAL_TIM_Base_Start(&htim2);
    ENCODER_CNT_Set(0);

#if UART_DEBUG
printf("GPIO、SPI2、USART1、TIM1-2、PWM、ENCODER 初始化完成。\r\n");
#endif
    
    // 上电检查控制器和遥控器各主要开关是否归位
#if UART_DEBUG
printf("正在检查遥控器接收和各主要开关是否归位...\r\n");
#endif
    while(1==switch_check()); // 归位时结束等待
#if UART_DEBUG
printf("无线接收正常，开关已归位\r\n");
#endif

    reset_flag = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      rcv_nrf_data();     // 接收遥控器数据
      // 若急停按钮触发
      if(Is_EMER()){
          s_error.is_emergency = 1;
          
          TIM1->CCR1 = 0;   // PWM 输出0
          TIM1->CCR4 = 0;
          
          VALVE_A_Off();    // 关闭阀 A 与 B
          VALVE_B_Off();
         
          reset_flag = 1;
#if UART_DEBUG
printf("!!!!!!!!!!!!!! 急停触发 !!!!!!!!!!!!!!!\r\n");
#endif
          goto lable_check;         // 跳出循环，检查开关归位状态（急停开关是否回位）
      }
      // 若急停未触发，则继续执行有效流程
      else{
            s_error.is_emergency = 0;
            digit_state_update(); // 喇叭、照明、发动机启停

/////////////////////////////////////////////////////////////////////////////////////////

#if UART_DEBUG
printf("正常运行...\r\n");
#endif
            if(1 == reset_flag){
                ctrlr_State = S_Standby;
                reset_flag = 0;
            }
            switch(ctrlr_State){
                //////// 待机模式 /////////////////
                case S_Standby: 
#if UART_DEBUG
printf("    >>>>待机状态---------------\r\n");
#endif
                    // 进入状态
                    s_run.sys_autorun = 0;

                    analog_state_update(0);              // 关闭比例阀 AB阀 AO怠速 
                    ENCODER_CNT_Set(0); // 编码器计数清零
                    index = 1;
                    // 判断比较上下学习结果 成功则 REC_flag |= 0x04
                    if(REC_flag == 0x03){
                        if(abs(up_rec[up_len].enc_cnt - dwn_rec[dwn_len].enc_cnt) <= REC_POS_TLR){
                            REC_flag |= 0x24; // 置位学习成功标志，并禁止先自动下降
                            s_run.rec_success = 1;
#if UART_DEBUG
printf("        **** 上下双向学习成功 ****\r\n");
#endif
                        }else{   
                            REC_flag &= 0xFB;
                            s_run.rec_success = 0;
#if UART_DEBUG
printf("        !!!! 上下双向学习未成功 !!!!\r\n");
#endif
                        }
                    }
                    // 切换
                    if(Is_HOPPER_UP() && !Is_AUTORUN()){
                        if(Is_REC_UP()){
                            memset(&up_rec,0,sizeof(REC_data_t)*256); // 清空数组
                        }
                        ctrlr_State = S_Manual_Up;
                    }
                    else 
                        if(Is_HOPPER_DOWN() && !Is_AUTORUN()){
                            if(Is_REC_DWN()){
                                memset(&dwn_rec,0,sizeof(REC_data_t)*256); // 清空数组
                            }
                            ctrlr_State = S_Manual_Down;
                        }
                    else 
                        if(Is_HOPPER_UP() && Is_AUTORUN() && (REC_flag&0x07) == 0x07 &&
                            (REC_flag & 0x10) == 0)
                            ctrlr_State = S_Auto_Up;
                    else 
                        if(Is_HOPPER_DOWN() && Is_AUTORUN() && (REC_flag&0x07) == 0x07 &&
                            (REC_flag & 0x20) == 0)
                            ctrlr_State = S_Auto_Down;
                    break;
                /////// 手动上升 /////////////////
                case S_Manual_Up:
#if UART_DEBUG
printf("    >>>>手动上升状态----------\r\n");
#endif
                    // 进入状态    
                    s_run.sys_manual = 1;
                    if(Is_REC_UP()){
                        s_run.sys_under_rec = 1;
                        if(abs(trans_value - up_rec[index-1].adc_data)>=REC_STEP){
                            up_rec[index].adc_data = trans_value;   // 记录给定值
                            up_rec[index].enc_cnt  = ENCODER_CNT_VALUE; // 记录编码器值
                            index += 1;
                            if(index > 254) index = 0;
                        }
#if UART_DEBUG
printf("        上升学习中 [ index = %02d, adc = %04d, enc = %05d ]\r\n",index,up_rec[index-1].adc_data,up_rec[index-1].enc_cnt);
#endif
                    }
                    else 
                        s_run.sys_under_rec = 0;
                    
                    analog_state_update(trans_value);
                    // 切换
                    if(!Is_HOPPER_UP()){
                        // 判断是否学习成功
                        if(index>=5 &&
                           //up_rec[index-1].adc_data < REC_STEP && 
                           up_rec[index].enc_cnt == 0 &&
                           up_rec[0].enc_cnt == 0 &&
                           rec_increment_check(up_rec, index) == 1){
                               up_rec[index].adc_data = 0;
                               up_rec[index].enc_cnt = ENCODER_CNT_VALUE;
                               up_len = index;
                               REC_flag |= 0x01;
#if UART_DEBUG
printf("        上升单程学习完成，数据 OK。\r\n");
#endif
                        }else{
                            REC_flag &= 0xFE;
#if UART_DEBUG
printf("        上升单程学习 NG !!!\r\n");
#endif
                        }
                        VALVE_A_Off();     
                        ctrlr_State = S_Standby;
                        s_run.sys_manual = 0;
                    }
                    break;
                /////// 手动下降 /////////////////
                case S_Manual_Down:
#if UART_DEBUG
printf("    >>>>手动下降状态----------\r\n");
#endif
                    // 进入状态    
                    s_run.sys_manual = 1;
                    if(Is_REC_DWN()){
                        s_run.sys_under_rec = 1;
                        if(abs(trans_value - dwn_rec[index-1].adc_data)>=REC_STEP){
                            dwn_rec[index].adc_data = trans_value;   // 记录给定值
                            dwn_rec[index].enc_cnt  = ENCODER_CNT_VALUE; // 记录编码器值
                            index += 1;
                            if(index > 254) index = 0;
                        }
#if UART_DEBUG
printf("        下降学习中 [ index = %02d, adc = %04d, enc = %05d ]\r\n",index,dwn_rec[index-1].adc_data,dwn_rec[index-1].enc_cnt);
#endif
                    }
                    else
                        s_run.sys_under_rec = 0;
                    analog_state_update(trans_value);
                    
                    // 切换
                    if(!Is_HOPPER_DOWN()){
                        // 判断是否学习成功
                        if(index>=5 &&
                           //dwn_rec[index-1].adc_data < REC_STEP && 
                           dwn_rec[index].enc_cnt == 0 &&
                           dwn_rec[0].enc_cnt == 0 &&
                           rec_increment_check(dwn_rec, index) == 1){
                               dwn_rec[index].adc_data = 0;
                               dwn_rec[index].enc_cnt = ENCODER_CNT_VALUE;
                               dwn_len = index;
                               REC_flag |= 0x02;
#if UART_DEBUG
printf("        下降单程学习完成，数据 OK。\r\n");
#endif
                        }else{
                            REC_flag &= 0xFD;
#if UART_DEBUG
printf("        下降单程学习 NG !!!\r\n");
#endif
                        }
                        VALVE_B_Off();
                        ctrlr_State = S_Standby;
                        s_run.sys_manual = 0;
                    }
                    break;
                /////// 自动上升 /////////////////
                case S_Auto_Up:
#if UART_DEBUG
printf("    >>>>自动上升状态----------\r\n");
#endif
                    // 进入状态
                    s_run.sys_autorun = 1;
                    if(ENCODER_CNT_VALUE <= up_rec[index].enc_cnt && 
                       ENCODER_CNT_VALUE >= up_rec[index-1].enc_cnt  ){
                           
                            analog_state_update(up_rec[index].adc_data);
#if UART_DEBUG
printf("        自动上升中 [ index = %02d, adc = %04d, enc = %05d | act %05d]\r\n",index,up_rec[index].adc_data,up_rec[index].enc_cnt,ENCODER_CNT_VALUE);
#endif
                            
                    }
                    if(ENCODER_CNT_VALUE >= up_rec[index].enc_cnt||
                       ENCODER_CNT_VALUE <200){
                        index++;
                    }
                    // 停止
                    if(up_rec[up_len].enc_cnt <= ENCODER_CNT_VALUE ||
                       index >= up_len){
                        
                        analog_state_update(0);
                    }
                    // 切换
                    if(!Is_HOPPER_UP() || !Is_AUTORUN()){
                        
                        REC_flag |= 0x10;
                        REC_flag &= 0xDF;
                        s_run.sys_autorun = 0;
                        ctrlr_State = S_Standby;
                    }
                    break;
                /////// 自动下降 /////////////////
                case S_Auto_Down:
#if UART_DEBUG
printf("    >>>>自动下降状态----------\r\n");
#endif
                    // 进入状态
                    s_run.sys_autorun = 1;
                    if(ENCODER_CNT_VALUE <= dwn_rec[index].enc_cnt && 
                       ENCODER_CNT_VALUE >= dwn_rec[index-1].enc_cnt  ){
                            analog_state_update(dwn_rec[index].adc_data);
#if UART_DEBUG
printf("        自动下降中 [ index = %02d, adc = %04d, enc = %05d | act %05d]\r\n",index,dwn_rec[index].adc_data,dwn_rec[index].enc_cnt,ENCODER_CNT_VALUE);
#endif
                            
                    }
                    if(ENCODER_CNT_VALUE >= dwn_rec[index].enc_cnt||
                       ENCODER_CNT_VALUE <200){
                        index++;
                    }
                    // 停止
                    if(dwn_rec[dwn_len].enc_cnt <= ENCODER_CNT_VALUE ||
                       index >= dwn_len ){
                        
                       analog_state_update(0);
                    }
                    // 切换
                    if(!Is_HOPPER_DOWN() || !Is_AUTORUN()){
                        
                        REC_flag |= 0x20;
                        REC_flag &= 0xEF;
                        s_run.sys_autorun = 0;
                        ctrlr_State = S_Standby;
                    }
                    break;
                /////// 运行错误 /////////////////
                default:
#if UART_DEBUG
printf("    >>>> 运行状态转换错误 强行进入待机状态 <<<<\r\n");
#endif
                    ctrlr_State = S_Standby;
                    break;
            }          
/////////////////////////////////////////////////////////////////////////////////////////
#if UART_DEBUG
printf("        处理完毕的数据 [ %02x%02x - %04d - %05d ] 。\r\n",receive_value[1],receive_value[2],receive_value[3]*256+receive_value[4],ENCODER_CNT_VALUE);
printf("---------------------------------------\r\n");
#endif          

      }   // end of "else" Is_EMER()

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
