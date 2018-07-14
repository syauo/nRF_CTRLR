#include "controller.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"

// 变量定义
uint16_t pwm_value=0, acc_value=0;
//REC_data_t   up_rec[256], dwn_rec[256];
//volatile  LED_Status_t   LED_Status; 
//volatile error_status_t  s_error;
//volatile run_status_t    s_run;



//---------------------------------------------------------
// 函数声明

///////////////////////////////////////////////////////////

////SysTick实现延时n_ms，中断方式
//void delay_ms(volatile unsigned long nms)
//{
//    //SYSTICK分频--1ms的系统时钟中断
//    if (SysTick_Config(HAL_RCC_GetHCLKFreq()/1000))
//    {
//   
//        while (1);
//    }
//    time_delay = nms;//读取定时时间
//    while(time_delay);
//    SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
//}
/**
 * 原cube HAL库中中断处理回调函数重写
 * Systick 在 main.c 中配置为 HAL_RCC_GetHCLKFreq()/1000，即每 1ms 中断一次
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
//   /* LED 闪烁位 bits (借鉴自 CANopenNode V1.1)*/
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
   
   // 连接状态/错误指示灯
//   if(s_error.nrf_not_exist==1 || s_error.is_emergency==1)  LED_RD_Turn(LED_Status.Flickering);
//   else if(s_error.switch_init_fail==1)                     LED_RD_Turn(LED_Status.Off);
////   else if(s_error.nrf_rcv_fail==1 )                        LED_RD_Turn(LED_Status.Off);
//   else                                                     LED_RD_Turn(LED_Status.SingleFlash);
//   
//    // 学习/自动运行指示灯
//    if(s_run.sys_autorun==1)            LED_GR_Turn(LED_Status.Flickering);
//    else if(s_run.rec_success==1)       LED_GR_Turn(LED_Status.On);
//    else if(s_run.sys_under_rec==1)     LED_GR_Turn(LED_Status.SingleFlash);
//    //else if(s_run.sys_manual==1)        LED_GR_Turn(LED_Status.SingleFlash);
//    else                                LED_GR_Turn(LED_Status.Off);
//}

/**
 * PWM 初始化
 */
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	TIM1->CCR1 = PWM_INIT_VALUE;
	TIM1->CCR4 = PWM_INIT_VALUE;
}

/**
 * nRF24L01 在位检查
 * 如果正常，进入数据接收模式，失败则循环等待
 */
void check_nrf(void)
{
#if UART_DEBUG
printf("无线模块在位检查...\r\n");
#endif
  while(NRF24L01_Check())
	{
#if UART_DEBUG
printf("!!!!!! 未检测到无线模块 !!!!!!\r\n");
#endif
		HAL_Delay(CHK_FAIL);
        //delay_ms(CHK_FAIL);
        s_error.nrf_not_exist = 1;
	}
  NRF24L01_RX_Mode();
  s_error.nrf_not_exist = 0;
#if UART_DEBUG
printf("无线模块连接正常，进入数据接收模式。\r\n");
#endif
}


/**
 * 数据接收并处理，由 main 函数调用
 */
uint8_t rcv_nrf_data(void)
{
	if(NRF24L01_RxPacket(receive_value)==0){
        
        // 检查接收的数据包是否有效，帧起始应为 0x55，索引 1 和 2 数据应相同
        if(receive_value[0]==0x55 && (receive_value[1] == receive_value[2])){
            
            // 数字量控制字节
            DIO_byte.Byte_Val = receive_value[1];
            // 接收的模拟量
            trans_value = receive_value[3]*256 + receive_value[4];
     
        }
        HAL_Delay(NRF_DELAY);
        //delay_ms(NRF_DELAY);
        s_error.nrf_rcv_fail = 0;
        return 0;
	}
    else{
#if UART_DEBUG
printf("        !!!!!!无线接收失败!!!!!!\r\n");
#endif
        s_error.nrf_rcv_fail = 1;
        HAL_Delay(RCV_FAIL);
        //delay_ms(RCV_FAIL);
        return 1;
    }
}

/**
 * 检查各主要开关是否归位
 * 返回 1=未归位，0=已归位
 */
// uint8_t switch_check(void)
// {   
//     uint8_t rcv_rslt;
//     rcv_rslt = rcv_nrf_data();
    
//     if(rcv_rslt==1 || (DIO_byte.Byte_Val & 0x3F)!=0 || trans_value>MIN_VALID){
//         s_error.switch_init_fail = 1;
// #if UART_DEBUG
// printf("        开关未归位！！\r\n");
// #endif
//         return 1;
//     }
//     else{
//         s_error.switch_init_fail = 0;
//         return 0;
//     }
// }

/**
 * 由接收到的数据控制硬件数字输出状态
 */
void digit_state_update(void)
{
    // 喇叭
    if(Is_HORN())     HORN_On();      else HORN_Off();
    // 照明灯
    if(Is_LAMP())     LED_LAMP_On();  else LED_LAMP_Off();
    // 引擎启动
    if(Is_ENGINE_START()) ENGINE_START_Set(); else ENGINE_START_Reset();
    // 引擎停止
    if(Is_ENGINE_STOP())  ENGINE_STOP_Set();  else ENGINE_STOP_Reset();
    
#if UART_DEBUG
printf("        喇叭、照明、引擎启停状态更新...\r\n");
#endif
}

/**
 * 由接收到的模拟量数据控制 PWM 输出、油门继电器和 MCP 扩展输出
 * 输入 a_value 0~4095 按定义范围控制比例阀和油门
 */
void analog_state_update(uint32_t a_value)
{
    // 传入值有效性处理
    a_value = (a_value >= MAX_VALID)? MAX_VALID : a_value; //大于最大有效值
    // a_value = (a_value <= MIN_VALID)? 0         : a_value; //小于最小有效值 
    // 分段控制
    // PWM 占空比
    if(a_value > PWM_SEG_E){ // 维持最大
        pwm_value = PWM_MAX;
    }else if(a_value >= PWM_SEG_QS){    // 慢速调节
        pwm_value = (a_value - PWM_SEG_QS)*(PWM_MAX - PWM_MIN)/(PWM_SEG_E - PWM_SEG_QS) +PWM_MIN;
    }else if(a_value >= PWM_SEG_S){     // 快速变化
        pwm_value = (a_value - PWM_SEG_S)*PWM_MIN/(PWM_SEG_QS - PWM_SEG_S);
    }else
        pwm_value = 0;

    // pwm_value = (a_value >= PWM_SEG_VALUE)? PWM_SEG_VALUE : a_value;
    // pwm_value = pwm_value*500.0/PWM_SEG_VALUE; // 占空比换算

    // 油门
    acc_value = (a_value > ACC_SEG_S)? ((a_value - ACC_SEG_S)*(4095.0 - ACC_IDLE_VALUE)/(MAX_VALID - ACC_SEG_S) + ACC_IDLE_VALUE) : ACC_IDLE_VALUE;
    // 油门继电器介入判断
    if(a_value >= RELAY_ON_VALUE)   ACC_RELAY_On(); else ACC_RELAY_Off();
    // AB阀控制
    if(a_value < (MIN_VALID - 4)){
        VALVE_A_Off();     
        VALVE_B_Off();
    }else if(a_value >= MIN_VALID){
        if(Is_HOPPER_UP())  { VALVE_A_On(); }else {VALVE_A_Off()};
        if(Is_HOPPER_DOWN()){ VALVE_B_On(); }else {VALVE_B_Off()};
    }
    // PWM 输出
    TIM1->CCR1 = pwm_value;   
    TIM1->CCR4 = pwm_value;

    // 油门输出
    MCP_WriteData(acc_value);
    
#if UART_DEBUG
printf("        比例阀及油门更新 [ PWM = %04d   ACC = %04d ]\r\n",pwm_value, acc_value);
#endif
}

/**
 * 模拟量扩展 MCP4922 发送数据
 * 与nRF24L01共用 spi2，AO_CSN 引脚作为片选信号
 */
void MCP_WriteData(uint16_t value) 
{
  uint8_t v_temp;
  uint8_t val[2];
  val[1] = (uint8_t)(value & 0xFF);
  v_temp = (uint8_t)((value >> 8) & 0x0F);
  val[0] = v_temp | MCP_CHA;
  AO_CSN_Enable();                 //使能片选
//  SPIx_ReadWriteByte(&hspi2, mcp_msb|MCP_CHA);
//  status =SPIx_ReadWriteByte(&hspi2,value);
  HAL_SPI_Transmit(&hspi2, val, 2, 50);
  AO_CSN_Disable();                 //禁止片选	
  __nop();__nop(); __nop();
  val[0] = v_temp | MCP_CHB;
  AO_CSN_Enable();                  //使能片选
//  SPIx_ReadWriteByte(&hspi2, mcp_msb|MCP_CHB);     
//  status =SPIx_ReadWriteByte(&hspi2,value);
  HAL_SPI_Transmit(&hspi2, val, 2, 50);
  AO_CSN_Disable();                 //禁止片选
}


/**
 * 检查学习数据是否按顺序递增
 * 0 = 否；1 = 递增
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




