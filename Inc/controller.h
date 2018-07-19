#ifndef __controller_H
#define __controller_H

#include "stm32f1xx_hal.h"


#define PWM_INIT_VALUE   0      // ������ PWM ��ʼ����

// ��λ���ֶο��ƶ���
#define MIN_VALID        10     // ��Ϊ������Ч����Сֵ
#define PWM_SEG_S        20     // PWMռ�ձȿ��ٱ仯��ʼ
#define PWM_SEG_QS       50     // PWMռ�ձȿ��ٱ仯���� ���ٱ仯��ʼ
#define RELAY_ON_VALUE   500    // ���ڴ�ֵ���ż̵�������
#define PWM_SEG_E        2000   // PWM�仯��Χ����
#define ACC_SEG_S        2000   // ģ�������� ACC_SEG_VALUE~4095 ���ֿ�������
#define MAX_VALID        4095   // ģ������Χ 0~MAX_SEG_VALUE

#define ADC_START_VAL    140    // �Զ�������ʼADC
#define ADC_STOP_VAL     50     // �Զ����н���ǰ��ADCֵ
// PWM 2kHz D=0~500
#define PWM_MIN          188
#define PWM_MAX          494
#define ACC_IDLE_VALUE   1479   // ����DAֵ��700mV*4096/Vref��
#define REC_STEP         500    //32     // ѧϰ���ݼ�¼����

// ����ѧϰ��¼�Ƚ���ֵ
#if UART_DEBUG
#define REC_POS_TLR      5000
#else
#define REC_POS_TLR      100    
#endif
// ����������//////////////////////////////////////////
#define Is_EMER()           (1 == DIO_byte.bits.bit0)
#define Is_HOPPER_UP()      (1 == DIO_byte.bits.bit1)
#define Is_HOPPER_DOWN()    (1 == DIO_byte.bits.bit2)
#define Is_AUTORUN()        (1 == DIO_byte.bits.bit3)
#define Is_ENGINE_STOP()    (1 == DIO_byte.bits.bit4)
#define Is_ENGINE_START()   (1 == DIO_byte.bits.bit5)
#define Is_HORN()           (1 == DIO_byte.bits.bit6)
#define Is_LAMP()           (1 == DIO_byte.bits.bit7)


// MCP4922 ��غ궨��///////////////////////////////////
//   bit15: ͨ��ѡ�� 0 = A, 1 = B
//   bit14: Vref ���뻺�� 1 = ����
//   bit13: nGA �������ѡ�� 0 = 2x, 1 = 1x
//   bit12: nSHDN ����ضϿ��� 0 = �ض� DAC ͨ��
// Aͨ������ 
#define MCP_CHA		0x70  //0b 0111 0000
// Bͨ������ 
#define MCP_CHB 	0xF0  //0b 1111 0000
							
// ���������� TIM2 ���/////////////////////////////////
// ������װ��ֵ
#define ENCODER_CNT_Set(X)    __HAL_TIM_SET_COUNTER(&htim2, X)
// ��ȡ������ֵ
#define ENCODER_CNT_VALUE     __HAL_TIM_GET_COUNTER(&htim2)



// ��������////////////////////////////////////
//extern volatile LED_Status_t LED_Status;
//extern volatile error_status_t s_error;
//extern volatile run_status_t s_run;

//extern REC_data_t  up_rec[256], dwn_rec[256];


// ��������////////////////////////////////////
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
