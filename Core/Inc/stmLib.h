#ifndef __STM_LIB_H__
#define __STM_LIB_H__



#ifdef __cplusplus
extern "C"{
#endif


void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ETH_Init(void);
void MX_USART3_UART_Init(void);
void MX_DMA_Init(void);
void MX_UART4_Init(void);
void MX_WWDG_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_ADC1_Init(void);


void initialize_stm(void);



#ifdef __cplusplus
}
#endif



#endif
