#ifndef __CONFIG_H__
#define __CONFIG_H__


#define F_CLK_72MHz 		0
#define F_CLK_100MHz 		1		

#define EMCU_DEBUG			0				// Not supported yet
#define EMCU_WDT				0				// Not supported yet
#define IS_NOISY				0				
#define EMCU_SLEEP			0				// Not supported yet

#define USB_CONN				0
#define _USB_HOST				0				// Not supported yet
#define _USB_CDC 				0
#define _USB_HID				0				// Not supported yet
#define _USB_MSC				0				// Not supported yet
#define _USB_AUDIO			0				// Not supported yet
#define CDC_BUF_SIZE  	128

#define EMCU_EMAC				0				// Not supported yet

#define EMCU_CAN				0				// Not supported yet
#define EMCU_SSP				0
#define EMCU_SPI				0
#define EMCU_I2C				1
#define EMCU_USART			1

#define EMCU_EEPROM			1
#define EEPROM_SIZE		(2*1024)		//  Bytes
#define EMCU_ADC				1
#define	EMCU_DAC				0
#define EMCU_EXTINT			0
#define EMCU_FIO				1
#define EMCU_MPWM				0
#define EMCU_PWM				1			
#define EMCU_RIT				0
#define EMCU_TIMER			1
#define EMCU_RTC				0
#define EMCU_DELAY			1


#endif

