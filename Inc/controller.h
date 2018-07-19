#ifndef __controller_H
#define __controller_H

#include "stm32f1xx_hal.h"


#define PWM_INIT_VALUE   0      // 比例阀 PWM 初始开量

// 电位器分段控制定义
#define MIN_VALID        10     // 认为数据有效的最小值
#define PWM_SEG_S        20     // PWM占空比快速变化开始
#define PWM_SEG_QS       50     // PWM占空比快速变化结束 慢速变化开始
#define RELAY_ON_VALUE   500    // 高于此值油门继电器介入
#define PWM_SEG_E        2000   // PWM变化范围结束
#define ACC_SEG_S        2000   // 模拟量划分 ACC_SEG_VALUE~4095 部分控制油门
#define MAX_VALID        4095   // 模拟量范围 0~MAX_SEG_VALUE

#define ADC_START_VAL    140    // 自动运行起始ADC
#define ADC_STOP_VAL     50     // 自动运行结束前的ADC值
// PWM 2kHz D=0~500
#define PWM_MIN          188
#define PWM_MAX          494
#define ACC_IDLE_VALUE   1479   // 怠速DA值（700mV*4096/Vref）
#define REC_STEP         500    //32     // 学习数据记录步长

// 上下学习记录比较阈值
#if UART_DEBUG
#define REC_POS_TLR      5000
#else
#define REC_POS_TLR      100    
#endif
// 数字量定义//////////////////////////////////////////
#define Is_EMER()           (1 == DIO_byte.bits.bit0)
#define Is_HOPPER_UP()      (1 == DIO_byte.bits.bit1)
#define Is_HOPPER_DOWN()    (1 == DIO_byte.bits.bit2)
#define Is_AUTORUN()        (1 == DIO_byte.bits.bit3)
#define Is_ENGINE_STOP()    (1 == DIO_byte.bits.bit4)
#define Is_ENGINE_START()   (1 == DIO_byte.bits.bit5)
#define Is_HORN()           (1 == DIO_byte.bits.bit6)
#define Is_LAMP()           (1 == DIO_byte.bits.bit7)


// MCP4922 相关宏定义///////////////////////////////////
//   bit15: 通道选择 0 = A, 1 = B
//   bit14: Vref 输入缓冲 1 = 缓冲
//   bit13: nGA 输出增益选择 0 = 2x, 1 = 1x
//   bit12: nSHDN 输出关断控制 0 = 关断 DAC 通道
// A通道配置 
#define MCP_CHA		0x70  //0b 0111 0000
// B通道配置 
#define MCP_CHB 	0xF0  //0b 1111 0000
							
// 编码器计数 TIM2 相关/////////////////////////////////
// 计数器装载值
#define ENCODER_CNT_Set(X)    __HAL_TIM_SET_COUNTER(&htim2, X)
// 读取计数器值
#define ENCODER_CNT_VALUE     __HAL_TIM_GET_COUNTER(&htim2)



// 变量声明////////////////////////////////////
//extern volatile LED_Status_t LED_Status;
//extern volatile error_status_t s_error;
//extern volatile run_status_t s_run;

//extern REC_data_t  up_rec[256], dwn_rec[256];


// 函数声明////////////////////////////////////
void delay_ms(volatile unsigned long nms);
void PWM_Init(void);
void check_nrf(void);
uint8_t rcv_nrf_data(void);
// uint8_t switch_check(void);
void digit_state_update(void);
void analog_state_update(uint32_t a_value);
void HAL_SYSTICK_Callback(void);
void MCP_WriteData(uint16_t value);
   
void state_switch(void);
uint8_t rec_increment_check(REC_data_t *i_array, uint16_t len);

#endif /*__ controller_H */
