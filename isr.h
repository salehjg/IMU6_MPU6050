#ifndef _ISR_HPP_
#define _ISR_HPP_

#include "LPC17xx.H" 
#include <stdint.h>
#include "eisr_def.h"
#include "eisr.h"


//#if EMCU_ADC
//	#include "adc.h"
//	extern ADC adc; 
//#endif

//#if EMCU_DAC
//	#include "dac.h"
//	extern DAC dac; 
//#endif

//#if EMCU_EEPROM
//	#include "eeprom.h"
//	extern EEPROM eeprom;
//#endif

//#if EMCU_EXTINT
//	#include "extint.h"
//	extern EXTINT eint;
//#endif

//#if EMCU_GPIO
//	#include "gpio.h"
//	extern GPIO io;
//#endif

//#if EMCU_FIO
//	#include "fio.h"
//	extern FIO io;
//#endif

//#if EMCU_I2C
//	#include "i2c.h"
//	extern I2C i2c;
//#endif

//#if EMCU_MPWM
//	#include "mpwm.h"
//	extern MPWM mpwm;
//#endif

//#if EMCU_PWM
//	#include "pwm.h"
//	extern PWM pwm;
//#endif

//#if EMCU_RIT
//	#include "rit.h"
//	extern RIT rit; 
//#endif

//#if EMCU_SSP
//	#include "ssp.h"
//	extern SSP spi; 
//#endif

//#if EMCU_TIMER
//	#include "timer.h"
//	extern TIMER timer;
//#endif

//#if EMCU_USART
//	#include "usart.h"
//	extern USART usart;
//#endif

//#if EMCU_DELAY
//	#include "delay.h"
//	extern DELAY delay;
//#endif



#ifdef __cplusplus
extern "C" {
#endif
	
void RIT_IRQHandler(void); 
void TIMER0_IRQHandler(void);		
void TIMER1_IRQHandler(void);	
void TIMER2_IRQHandler(void);		
void TIMER3_IRQHandler(void);	
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);	
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void I2C2_IRQHandler(void);
	
void RTC_IRQHandler (void);
	
/*------------------------------------*/
//extern __inline void SysTick_ISR(void);	
	
/*------------------------------------*/
	
#ifdef __cplusplus
}
#endif	

#endif
