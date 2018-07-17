#include "controller.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"

// ��������
uint16_t pwm_value=0, acc_value=0;
//REC_data_t   up_rec[256], dwn_rec[256];
//volatile  LED_Status_t   LED_Status; 
//volatile error_status_t  s_error;
//volatile run_status_t    s_run;



//---------------------------------------------------------
// ��������

///////////////////////////////////////////////////////////

////SysTickʵ����ʱn_ms���жϷ�ʽ
//void delay_ms(volatile unsigned long nms)
//{
//    //SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
//    if (SysTick_Config(HAL_RCC_GetHCLKFreq()/1000))
//    {
//   
//        while (1);
//    }
//    time_delay = nms;//��ȡ��ʱʱ��
//    while(time_delay);
//    SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
//}
/**
 * ԭcube HAL�����жϴ���ص�������д
 * Systick �� main.c ������Ϊ HAL_RCC_GetHCLKFreq()/1000����ÿ 1ms �ж�һ��
 */
//void HAL_SYSTICK_Callback(void)
//{
//    static uint16_t      tLEDflicker=0;
//    static uint16_t      tLEDflash=0;
    
//   if(time_delay)
//		time_delay--;
   
//   LED_Status.On  = LED_ON;
//   LED_Status.Off = LED_OFF;
//   
//   /* LED ��˸λ bits (����� CANopenNode V1.1)*/
//   //10Hz
//   if   (++tLEDflicker == 50)    LED_Status.Flickering = LED_ON;
//   else if(tLEDflicker >= 100){  LED_Status.Flickering = LED_OFF; tLEDflicker = 0;}
//   //
//   if   (++tLEDflash == 800)     LED_Status.SingleFlash = LED_ON;
//   else if(tLEDflash >= 1000){    LED_Status.SingleFlash = LED_OFF; tLEDflash = 0;}
   //flashes
   //tLEDflash = tLEDflash + 1;
//    switch(++tLEDflash){
//       case 200:  LED_Status.TripleFlash = LED_ON;
//                  LED_Status.DoubleFlash = LED_ON;
//                  LED_Status.SingleFlash = LED_ON; break;
//       case 400:  LED_Status.TripleFlash = LED_OFF;
//                  LED_Status.DoubleFlash = LED_OFF;
//                  LED_Status.SingleFlash = LED_OFF; break;
//       case 600:  LED_Status.TripleFlash = LED_ON;
//                  LED_Status.DoubleFlash = LED_ON; break;
//       case 800:  LED_Status.TripleFlash = LED_OFF;
//                  LED_Status.DoubleFlash = LED_OFF; break;
//       case 1000: LED_Status.TripleFlash = LED_ON; break;
//       case 1200: LED_Status.TripleFlash = LED_OFF; break;
//       //case 2000: tLEDflash = 0; break;
//       default:  break;
//    }
//    if(tLEDflash >= 2000) tLEDflash = 0;
   
   // ����״̬/����ָʾ��
//   if(s_error.nrf_not_exist==1 || s_error.is_emergency==1)  LED_RD_Turn(LED_Status.Flickering);
//   else if(s_error.switch_init_fail==1)                     LED_RD_Turn(LED_Status.Off);
////   else if(s_error.nrf_rcv_fail==1 )                        LED_RD_Turn(LED_Status.Off);
//   else                                                     LED_RD_Turn(LED_Status.SingleFlash);
//   
//    // ѧϰ/�Զ�����ָʾ��
//    if(s_run.sys_autorun==1)            LED_GR_Turn(LED_Status.Flickering);
//    else if(s_run.rec_success==1)       LED_GR_Turn(LED_Status.On);
//    else if(s_run.sys_under_rec==1)     LED_GR_Turn(LED_Status.SingleFlash);
//    //else if(s_run.sys_manual==1)        LED_GR_Turn(LED_Status.SingleFlash);
//    else                                LED_GR_Turn(LED_Status.Off);
//}

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
#if UART_DEBUG
printf("����ģ����λ���...\r\n");
#endif
  while(NRF24L01_Check())
	{
#if UART_DEBUG
printf("!!!!!! δ��⵽����ģ�� !!!!!!\r\n");
#endif
		HAL_Delay(CHK_FAIL);
        //delay_ms(CHK_FAIL);
        s_error.nrf_not_exist = 1;
	}
  NRF24L01_RX_Mode();
  s_error.nrf_not_exist = 0;
#if UART_DEBUG
printf("����ģ�������������������ݽ���ģʽ��\r\n");
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
        HAL_Delay(NRF_DELAY);
        //delay_ms(NRF_DELAY);
        s_error.nrf_rcv_fail = 0;
        return 0;
	}
    else{
#if UART_DEBUG
printf("        !!!!!!���߽���ʧ��!!!!!!\r\n");
#endif
        s_error.nrf_rcv_fail = 1;
        HAL_Delay(RCV_FAIL);
        //delay_ms(RCV_FAIL);
        return 1;
    }
}

/**
 * ������Ҫ�����Ƿ��λ
 * ���� 1=δ��λ��0=�ѹ�λ
 */
// uint8_t switch_check(void)
// {   
//     uint8_t rcv_rslt;
//     rcv_rslt = rcv_nrf_data();
    
//     if(rcv_rslt==1 || (DIO_byte.Byte_Val & 0x3F)!=0 || trans_value>MIN_VALID){
//         s_error.switch_init_fail = 1;
// #if UART_DEBUG
// printf("        ����δ��λ����\r\n");
// #endif
//         return 1;
//     }
//     else{
//         s_error.switch_init_fail = 0;
//         return 0;
//     }
// }

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
    
#if UART_DEBUG
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
printf("        �����������Ÿ��� [ PWM = %04d   ACC = %04d ]\r\n",pwm_value, acc_value);
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




