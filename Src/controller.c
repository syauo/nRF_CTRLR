#include "controller.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"

// ��������
uint16_t pwm_value=0, acc_value=0;

//---------------------------------------------------------
// ��������

///////////////////////////////////////////////////////////


/**
 * PWM ��ʼ��
 */
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	TIM1->CCR1 = PWM_INIT_VALUE;
	TIM1->CCR4 = PWM_INIT_VALUE;
}

/**
 * nRF24L01 ��λ���
 * ����������������ݽ���ģʽ��ʧ����ѭ���ȴ�
 */
void check_nrf(void)
{
  while(NRF24L01_Check())
	{
        #if UART_DEBUG
        printf(PRNT_P_CHK);
        printf(PRNT_RD_STR,"NG");
        #endif
		HAL_Delay(CHK_FAIL);
        //delay_ms(CHK_FAIL);
        s_error.nrf_not_exist = 1;
	}
    NRF24L01_RX_Mode();
    s_error.nrf_not_exist = 0;
    #if UART_DEBUG
    printf(PRNT_P_CHK);
    printf(PRNT_GR_STR,"OK");
    #endif
}


/**
 * ���ݽ��ղ������� main ��������
 */
uint8_t rcv_nrf_data(void)
{
	if(NRF24L01_RxPacket(receive_value)==0){
        
        // �����յ����ݰ��Ƿ���Ч��֡��ʼӦΪ 0x55������ 1 �� 2 ����Ӧ��ͬ
        if(receive_value[0]==0x55 && (receive_value[1] == receive_value[2])){
            
            // �����������ֽ�
            DIO_byte.Byte_Val = receive_value[1];
            // ���յ�ģ����
            trans_value = receive_value[3]*256 + receive_value[4];
     
        }
        //HAL_Delay(NRF_DELAY);
        s_error.nrf_rcv_fail = 0;
        #if UART_DEBUG
        printf(PRNT_P_RCV);
        printf(PRNT_GR_STR,"OK");
        #endif
        return 0;
	}
    else{
        #if UART_DEBUG
        printf(PRNT_P_RCV);
        printf(PRNT_RD_STR,"NG");
        #endif
        s_error.nrf_rcv_fail = 1;
        //HAL_Delay(RCV_FAIL);
        return 1;
    }
}


/**
 * �ɽ��յ������ݿ���Ӳ���������״̬
 */
void digit_state_update(void)
{
    // ����
    if(Is_HORN())     HORN_On();      else HORN_Off();
    // ������
    if(Is_LAMP())     LED_LAMP_On();  else LED_LAMP_Off();
    // ��������
    if(Is_ENGINE_START()) ENGINE_START_Set(); else ENGINE_START_Reset();
    // ����ֹͣ
    if(Is_ENGINE_STOP())  ENGINE_STOP_Set();  else ENGINE_STOP_Reset();
    
    #if 0 //UART_DEBUG
    printf("        ���ȡ�������������ͣ״̬����...\r\n");
    #endif
}

/**
 * �ɽ��յ���ģ�������ݿ��� PWM ��������ż̵����� MCP ��չ���
 * ���� a_value 0~4095 �����巶Χ���Ʊ�����������
 */
void analog_state_update(uint32_t a_value)
{
    // ����ֵ��Ч�Դ���
    a_value = (a_value >= MAX_VALID)? MAX_VALID : a_value; //���������Чֵ
    // a_value = (a_value <= MIN_VALID)? 0         : a_value; //С����С��Чֵ 
    // �ֶο���
    // PWM ռ�ձ�
    if(a_value > PWM_SEG_E){ // ά�����
        pwm_value = PWM_MAX;
    }else if(a_value >= PWM_SEG_QS){    // ���ٵ���
        pwm_value = (a_value - PWM_SEG_QS)*(PWM_MAX - PWM_MIN)/(PWM_SEG_E - PWM_SEG_QS) +PWM_MIN;
    }else if(a_value >= PWM_SEG_S){     // ���ٱ仯
        pwm_value = (a_value - PWM_SEG_S)*PWM_MIN/(PWM_SEG_QS - PWM_SEG_S);
    }else
        pwm_value = 0;

    // pwm_value = (a_value >= PWM_SEG_VALUE)? PWM_SEG_VALUE : a_value;
    // pwm_value = pwm_value*500.0/PWM_SEG_VALUE; // ռ�ձȻ���

    // ����
    acc_value = (a_value > ACC_SEG_S)? ((a_value - ACC_SEG_S)*(4095.0 - ACC_IDLE_VALUE)/(MAX_VALID - ACC_SEG_S) + ACC_IDLE_VALUE) : ACC_IDLE_VALUE;
    // ���ż̵��������ж�
    if(a_value >= RELAY_ON_VALUE)   ACC_RELAY_On(); else ACC_RELAY_Off();
    // AB������
    if(a_value < (MIN_VALID - 4)){
        VALVE_A_Off();     
        VALVE_B_Off();
    }else if(a_value >= MIN_VALID){
        if(Is_HOPPER_UP())  { VALVE_A_On(); }else {VALVE_A_Off()};
        if(Is_HOPPER_DOWN()){ VALVE_B_On(); }else {VALVE_B_Off()};
    }
    // PWM ���
    TIM1->CCR1 = pwm_value;   
    TIM1->CCR4 = pwm_value;

    // �������
    MCP_WriteData(acc_value);
    
    #if UART_DEBUG
    printf(PRNT_P_PWM);
    printf("%04d  %04d",pwm_value, acc_value);
    #endif
}

/**
 * ģ������չ MCP4922 ��������
 * ��nRF24L01���� spi2��AO_CSN ������ΪƬѡ�ź�
 */
void MCP_WriteData(uint16_t value) 
{
  uint8_t v_temp;
  uint8_t val[2];
  val[1] = (uint8_t)(value & 0xFF);
  v_temp = (uint8_t)((value >> 8) & 0x0F);
  val[0] = v_temp | MCP_CHA;
  AO_CSN_Enable();                 //ʹ��Ƭѡ
//  SPIx_ReadWriteByte(&hspi2, mcp_msb|MCP_CHA);
//  status =SPIx_ReadWriteByte(&hspi2,value);
  HAL_SPI_Transmit(&hspi2, val, 2, 50);
  AO_CSN_Disable();                 //��ֹƬѡ	
  __nop();__nop(); __nop();
  val[0] = v_temp | MCP_CHB;
  AO_CSN_Enable();                  //ʹ��Ƭѡ
//  SPIx_ReadWriteByte(&hspi2, mcp_msb|MCP_CHB);     
//  status =SPIx_ReadWriteByte(&hspi2,value);
  HAL_SPI_Transmit(&hspi2, val, 2, 50);
  AO_CSN_Disable();                 //��ֹƬѡ
}


/**
 * ���ѧϰ�����Ƿ�˳�����
 * 0 = ��1 = ����
 */
uint8_t rec_increment_check(REC_data_t *i_array, uint16_t len){
    uint16_t i;
    for(i=1; i<len; i++){
        if(i_array[i].enc_cnt < i_array[i-1].enc_cnt){
            return 0;
        }
    }
    return 1;
}




