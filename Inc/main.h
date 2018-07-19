/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ENCODER_ETR_Pin GPIO_PIN_0
#define ENCODER_ETR_GPIO_Port GPIOA
#define VALVE_A1_Pin GPIO_PIN_4
#define VALVE_A1_GPIO_Port GPIOA
#define VALVE_A2_Pin GPIO_PIN_5
#define VALVE_A2_GPIO_Port GPIOA
#define VALVE_B1_Pin GPIO_PIN_6
#define VALVE_B1_GPIO_Port GPIOA
#define VALVE_B2_Pin GPIO_PIN_7
#define VALVE_B2_GPIO_Port GPIOA
#define REC_UP_Pin GPIO_PIN_0
#define REC_UP_GPIO_Port GPIOB
#define ENGINE_STOP_Pin GPIO_PIN_1
#define ENGINE_STOP_GPIO_Port GPIOB
#define ENGINE_START_Pin GPIO_PIN_10
#define ENGINE_START_GPIO_Port GPIOB
#define REC_DWN_Pin GPIO_PIN_11
#define REC_DWN_GPIO_Port GPIOB
#define AO_CSN_Pin GPIO_PIN_12
#define AO_CSN_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOA
#define ACC_RELAY_Pin GPIO_PIN_15
#define ACC_RELAY_GPIO_Port GPIOA
#define HORN_Pin GPIO_PIN_3
#define HORN_GPIO_Port GPIOB
#define LED_LAMP_Pin GPIO_PIN_4
#define LED_LAMP_GPIO_Port GPIOB
#define LED_STA_Pin GPIO_PIN_5
#define LED_STA_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_6
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_7
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOB
#define LED_AUTO_Pin GPIO_PIN_9
#define LED_AUTO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
// 串口打印调试信息
#define UART_DEBUG  1   //1
// 记录数组定义长度
#define REC_LEN    140
// 编码器读数
#define ENC_VAL    ENCODER_CNT_VALUE

// PRINT 超级终端相关
#define PRNT_DFT    "\033[0m"       // 关闭效果
#define PRNT_P_CHK  "\033[3;35H"    // nrf在位检查
#define PRNT_P_RCV  "\033[3;43H"    // nrf接收成功否
#define PRNT_P_UP   "\033[4;28H"    // 上升学习OK？
#define PRNT_P_DN   "\033[4;35H"    // 下降学习OK？
#define PRNT_P_ALL  "\033[4;43H"    // 双向学习OK?
#define PRNT_P_SW   "\033[5;43H"    // 开关归位OK？
#define PRNT_P_DIO  "\033[6;33H"    // 接收的数字量
#define PRNT_P_ADC  "\033[6;41H"    // 接收的电位器
#define PRNT_P_STAT "\033[7;34H"    // 当前状态
#define PRNT_P_REC  "\033[7;41H"    // 是否学习中
#define PRNT_P_IDX  "\033[11;2H"    // 索引处
#define PRNT_P_PWM  "\033[11;34H"   // PWM处
#define PRNT_GR_STR "\033[1;42;37m%s\033[0m"    // 绿底白字
#define PRNT_RD_STR "\033[1;41;37m%s\033[0m"    // 红底白字
#define PRNT_FL_STR "\033[5m%s\033[0m"          // 闪烁-正常格式
//--------------IO 宏定义-------------------------

// 闪灯状态指示
#define CHK_FAIL   500
#define NRF_DELAY  200  // 两次成功接收间隔
#define RCV_FAIL   1000

// 照明灯 
#define LED_LAMP_On()       HAL_GPIO_WritePin(LED_LAMP_GPIO_Port, LED_LAMP_Pin, GPIO_PIN_SET)
#define LED_LAMP_Off()      HAL_GPIO_WritePin(LED_LAMP_GPIO_Port, LED_LAMP_Pin, GPIO_PIN_RESET)

// 状态灯
#define LED_STA_Off()        HAL_GPIO_WritePin(LED_STA_GPIO_Port, LED_STA_Pin, GPIO_PIN_SET)
#define LED_STA_On()       HAL_GPIO_WritePin(LED_STA_GPIO_Port, LED_STA_Pin, GPIO_PIN_RESET)
#define LED_STA_Toggle()    HAL_GPIO_TogglePin(LED_STA_GPIO_Port, LED_STA_Pin)

#define LED_RD_Turn(X)  HAL_GPIO_WritePin(LED_STA_GPIO_Port, LED_STA_Pin, X )
#define LED_RD_Off()    LED_STA_Off()
#define LED_RD_On()     LED_STA_On()
#define LED_RD_Toggle() LED_STA_Toggle()


// 自动运行 LED RD
#define LED_AUTO_Off()       HAL_GPIO_WritePin(LED_AUTO_GPIO_Port, LED_AUTO_Pin, GPIO_PIN_SET)
#define LED_AUTO_On()      HAL_GPIO_WritePin(LED_AUTO_GPIO_Port, LED_AUTO_Pin, GPIO_PIN_RESET)
#define LED_AUTO_Toggle()   HAL_GPIO_TogglePin(LED_AUTO_GPIO_Port, LED_AUTO_Pin)

#define LED_GR_Turn(X)   HAL_GPIO_WritePin(LED_AUTO_GPIO_Port, LED_AUTO_Pin, X )
#define LED_GR_Off()    LED_AUTO_Off()
#define LED_GR_On()     LED_AUTO_On()
#define LED_GR_Toggle() LED_AUTO_Toggle()

// 喇叭
#define HORN_Off()           HAL_GPIO_WritePin(HORN_GPIO_Port, HORN_Pin, GPIO_PIN_RESET)
#define HORN_On()          HAL_GPIO_WritePin(HORN_GPIO_Port, HORN_Pin, GPIO_PIN_SET)

// 油门继电器
#define ACC_RELAY_Off()      HAL_GPIO_WritePin(ACC_RELAY_GPIO_Port, ACC_RELAY_Pin, GPIO_PIN_RESET)
#define ACC_RELAY_On()     HAL_GPIO_WritePin(ACC_RELAY_GPIO_Port, ACC_RELAY_Pin, GPIO_PIN_SET)

// 阀 A 开
#define VALVE_A_On()       {HAL_GPIO_WritePin(VALVE_B1_GPIO_Port, VALVE_B1_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_B2_GPIO_Port, VALVE_B2_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_A1_GPIO_Port, VALVE_A1_Pin, GPIO_PIN_SET);\
                            HAL_GPIO_WritePin(VALVE_A2_GPIO_Port, VALVE_A2_Pin, GPIO_PIN_SET);}
// 阀 A 关
#define VALVE_A_Off()      {HAL_GPIO_WritePin(VALVE_A1_GPIO_Port, VALVE_A1_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_A2_GPIO_Port, VALVE_A2_Pin, GPIO_PIN_RESET);}
// 阀 B 开
#define VALVE_B_On()       {HAL_GPIO_WritePin(VALVE_A1_GPIO_Port, VALVE_A1_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_A2_GPIO_Port, VALVE_A2_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_B1_GPIO_Port, VALVE_B1_Pin, GPIO_PIN_SET);\
                            HAL_GPIO_WritePin(VALVE_B2_GPIO_Port, VALVE_B2_Pin, GPIO_PIN_SET);}
// 阀 B 关
#define VALVE_B_Off()      {HAL_GPIO_WritePin(VALVE_B1_GPIO_Port, VALVE_B1_Pin, GPIO_PIN_RESET);\
                            HAL_GPIO_WritePin(VALVE_B2_GPIO_Port, VALVE_B2_Pin, GPIO_PIN_RESET);}
// 输入 升学习
#define Is_REC_UP()         (GPIO_PIN_RESET == HAL_GPIO_ReadPin(REC_UP_GPIO_Port, REC_UP_Pin))

// 输入 降学习
#define Is_REC_DWN()        (GPIO_PIN_RESET == HAL_GPIO_ReadPin(REC_DWN_GPIO_Port, REC_DWN_Pin))

// 引擎停止
#define ENGINE_STOP_Set()    HAL_GPIO_WritePin(ENGINE_STOP_GPIO_Port, ENGINE_STOP_Pin, GPIO_PIN_SET)
#define ENGINE_STOP_Reset()  HAL_GPIO_WritePin(ENGINE_STOP_GPIO_Port, ENGINE_STOP_Pin, GPIO_PIN_RESET)

// 引擎启动
#define ENGINE_START_Set()   HAL_GPIO_WritePin(ENGINE_START_GPIO_Port, ENGINE_START_Pin, GPIO_PIN_SET)
#define ENGINE_START_Reset() HAL_GPIO_WritePin(ENGINE_START_GPIO_Port, ENGINE_START_Pin, GPIO_PIN_RESET)

// AO 片选
#define AO_CSN_Disable()        HAL_GPIO_WritePin(AO_CSN_GPIO_Port, AO_CSN_Pin, GPIO_PIN_SET)
#define AO_CSN_Enable()      HAL_GPIO_WritePin(AO_CSN_GPIO_Port, AO_CSN_Pin, GPIO_PIN_RESET)
//------------------数据类型-------------------------
// 单字节位变量结构体             
typedef union _u8_Bits_t 
{
    uint8_t Byte_Val;
    struct
    {
        uint8_t bit0        :1;
        uint8_t bit1        :1; 
        uint8_t bit2        :1;
        uint8_t bit3        :1; 
        uint8_t bit4        :1; 
        uint8_t bit5        :1;
        uint8_t bit6        :1;
        uint8_t bit7        :1;
    }bits;
}u8_Bits_t; 
// led状态的定义
typedef struct {
      GPIO_PinState On          ;
      GPIO_PinState Off         ;
      GPIO_PinState Flickering  ;
    //   GPIO_PinState Blinking    ;
      GPIO_PinState SingleFlash ;
    //   GPIO_PinState DoubleFlash ;
    //   GPIO_PinState TripleFlash ;
}LED_Status_t;

#define LED_ON  GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET

// 系统运行主状态定义 
   typedef enum {
       S_Emerg       = 0x00,
       S_Manual_Up   = 0x01,
       S_Manual_Down = 0x02,
       S_Auto_Up     = 0x03,
       S_Auto_Down   = 0x04,
       S_Standby     = 0x05
   }run_state_t;
   
// 错误状态
typedef struct{
    uint8_t nrf_not_exist ;
    uint8_t nrf_rcv_fail  ;
    uint8_t switch_init_fail ;
    uint8_t is_emergency  ;
}error_status_t;
// 运行状态
typedef struct{
    uint8_t sys_under_rec ;
    uint8_t rec_success   ;
    uint8_t sys_autorun   ;
    uint8_t sys_manual;
}run_status_t;
// 学习数据
typedef struct{
   uint16_t enc_cnt;
   uint16_t adc_data;
}REC_data_t;

//--------------------------变量声明------------------   
extern volatile LED_Status_t LED_Status;
extern volatile error_status_t s_error;
extern volatile run_status_t s_run;
extern REC_data_t  up_rec[REC_LEN], dwn_rec[REC_LEN];
// 无线接收字节暂存数组
extern uint8_t receive_value[6];
// 模拟量接收值
extern uint16_t trans_value;
// 数字量控制位
extern volatile u8_Bits_t DIO_byte;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
