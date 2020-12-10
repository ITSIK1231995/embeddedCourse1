

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*base adresses of flash and SRAM memories */
#define FLASH_BASEADDR 0x08000000U // main memory base address  // U means unsigned integer
#define SRAM1_BASEADDR 0x20000000U // ram base address
#define SRAM2_BASEADDR 0x20001C00U // ram2 base address (follows ram1 in memory
#define ROM_BASEADDR   0x1FFF0000U // system memory base address
#define SRAM		   SRAM1_BASEADDR

/* AHBx and APBx bus peripheral base addresses*/

#define PERIPH_BASE 		0x4000000U
#define APB1PERIPH_BASEADDR PERIPH_BASE
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

/*
 * base addresses of peripherals wich are hanging on AHB1
 * */
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * base addresses of peripherals wich are hanging on APB1
 * */

#define I2C1_BASEADDR 	(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 	(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 	(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR	    (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR	    (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR	    (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR	    (APB1PERIPH_BASEADDR + 0x5000)

/*
 * base addresses of peripherals wich are hanging on APB2
 * */

#define EXTI_BASEADDR 	 (APB2PERIPH_BASEADDR + 0x3c00)
#define SPI1_BASEADDR	 (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR	 (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR	 (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR	 (APB2PERIPH_BASEADDR + 0x1400)

/*******************************peripheral register definition structures ***************************/
typedef struct{
	__vo uint32_t MODER;			//GPIO port mode register
	__vo uint32_t OTYPER;		//GPIO port output type register
	__vo uint32_t OSPEEDR;		//GPIO port output speed register
	__vo uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			//GPIO port input data register
	__vo uint32_t ODR;			//GPIO port output data register
	__vo uint32_t BSRR;			//GPIO port bit set/reset register
	__vo uint32_t LCKR;			//GPIO port configuration lock register
	__vo uint32_t AFR[2];		//AFR[0] : GPIO alternate function low register | AFR[1] : GPIO alternate function high register

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			//RCC clock control register
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register
	__vo uint32_t CFGR;			//RCC clock configuration register
	__vo uint32_t CIR;			//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register
	uint32_t RESERVED0;			//Reserved
	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];		//Reserved
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;			//Reserved
	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];		//Reserved
	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;			//Reserved
	__vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];		//Reserved
	__vo uint32_t BDCR;			//RCC Backup domain control register
	__vo uint32_t CSR;			//RCC clock control & status register
	uint32_t RESERVED6[2];		//Reserved
	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register




} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */

} SYSCFG_RegDef_t;




/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RefDef_t
 * */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 8))

/*
 * clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/*
 * clock enable macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))
/*
 * clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 8))
/*
 * clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DE() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DE() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DE() (RCC->APB1ENR &= ~(1 << 23))

/*
 * clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DE() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DE() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DE() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DE() (RCC->APB2ENR &= ~(1 << 13))

/*
 * clock disable macros for USARTx peripherals
 */
#define USART1_PCCK_DE() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCCK_DE() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DE() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCCK_DE()  (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCCK_DE()  (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCCK_DE() (RCC->APB1ENR &= ~(1 << 5))
/*
 * clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DE() (RCC->APB2ENR &= ~(1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI1    1
#define NVIC_IRQ_PRI2    2
#define NVIC_IRQ_PRI3    3
#define NVIC_IRQ_PRI4    4
#define NVIC_IRQ_PRI5    5
#define NVIC_IRQ_PRI6    6
#define NVIC_IRQ_PRI7    7
#define NVIC_IRQ_PRI8    8
#define NVIC_IRQ_PRI9    9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14

#define NVIC_IRQ_PRI15    15


// some generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f407xx_gpio_driver.h"



#endif /* INC_STM32F407XX_H_ */
