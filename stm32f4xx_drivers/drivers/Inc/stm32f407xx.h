/*
 * stm32f407xx.h
 *
 *  Created on: May 18, 2020
 *      Author: Shashank
 */


//MCU SPECIFIC HEADER FILE

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stdint.h>
#define __vo volatile

/*base addressses of SRAM,System memory and Flash memory*/

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM             SRAM1_BASEADDR
#define SRAM2_BASEADDR    0x20001C00U
#define ROM              0x1FFF0000U

/*base addresses of Peripheral base addresses of AHB1/AHB2 and APB1/APB2 bus*/

#define PERIPH_BASEADDR              0x40000000U
#define APB1PERIPH_BASEADDR          PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR          0x40010000U
#define AHB1PERIPH_BASEADDR          0x40020000U
#define AHB2PERIPH_BASEADDR          0x50000000U

/*Base address of peripherals which are hanging on AHB1 bus*/

#define GPIOA_BASEADDR               (AHB1PERIPH_BASEADDR+0X0000)
#define GPIOB_BASEADDR               (AHB1PERIPH_BASEADDR+0X0400)
#define GPIOC_BASEADDR               (AHB1PERIPH_BASEADDR+0X0800)
#define GPIOD_BASEADDR               (AHB1PERIPH_BASEADDR+0X0C00)
#define GPIOE_BASEADDR               (AHB1PERIPH_BASEADDR+0X1000)
#define GPIOF_BASEADDR               (AHB1PERIPH_BASEADDR+0X1400)
#define GPIOG_BASEADDR               (AHB1PERIPH_BASEADDR+0X1800)
#define GPIOH_BASEADDR               (AHB1PERIPH_BASEADDR+0X1C00)
#define GPIOI_BASEADDR               (AHB1PERIPH_BASEADDR+0X2000)
#define GPIOJ_BASEADDR               (AHB1PERIPH_BASEADDR+0X2400)
#define GPIOK_BASEADDR               (AHB1PERIPH_BASEADDR+0X2800)
#define RCC_BASEADDR                 (AHB1PERIPH_BASEADDR+0X3800)

/*Base address of peripherals which are hanging on APB1 bus*/

#define I2C1_BASEADDR                 (APB1PERIPH_BASEADDR+0X5400)
#define I2C2_BASEADDR                 (APB1PERIPH_BASEADDR+0X5BFF)
#define I2C3_BASEADDR                 (APB1PERIPH_BASEADDR+0X5FFF)
#define USART2_BASEADDR               (APB1PERIPH_BASEADDR+0X47FF)
#define USART3_BASEADDR               (APB1PERIPH_BASEADDR+0X4BFF)
#define UART4_BASEADDR                (APB1PERIPH_BASEADDR+0X4FFF)
#define UART5_BASEADDR                (APB1PERIPH_BASEADDR+0X53FF)
#define SPI2_BASEADDR                (APB1PERIPH_BASEADDR+0X3BFF)
#define SPI3_BASEADDR                (APB1PERIPH_BASEADDR+0X3FFF)

/*Base address of peripherals which are hanging on APB2 bus*/

#define EXTI_BASEADDR                 (APB2PERIPH_BASEADDR+0X3FFF)
#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR+0X33FF)
#define USART1_BASEADDR               (APB2PERIPH_BASEADDR+0X13FF)
#define USART6_BASEADDR               (APB2PERIPH_BASEADDR+0X17FF)





/*Register of a Peripheral are specific to MCU
 * The register of the SPI peripheral of stm32fx MCU may be different(more or less)
 * compared to that of SPI peripheral of stm32lx MCU
 * Please check your device RM*/

/*GPIO peripheral configuration Registers*/

typedef struct
{
	__vo uint32_t MODER;                    /*GPIO port mode register           Address offset: 0x00*/
	__vo uint32_t OTYPER;                   /*GPIO port output type register    Address offset: 0x04*/
	__vo uint32_t OSPEEDR;                  /*GPIO port output speed register   Address offset: 0x08*/
	__vo uint32_t PUPDR;               /*GPIO port pull-up/pull-down register   Address offset: 0x0C*/
	__vo uint32_t IDR;                      /*GPIO port input data register     Address offset: 0x10*/
	__vo uint32_t ODR;                      /*GPIO port output data register    Address offset: 0x14*/
	__vo uint32_t BSRR;                     /*GPIO port bit set/reset register  Address offset: 0x18*/
	__vo uint32_t LCKR;                /*GPIO port configuration lock register  Address offset: 0x18*/
	__vo uint32_t AFR[2];              /*AFR[1]=GPIO alternate function low register   Address offset: 0x20
	                                AFR[2]=GPIO alternate function high register  Address offset: 0x24*/
}GPIO_RegDef_t;

/*SPI peripheral configuration structure*/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


/*I2C peripheral configuration structure*/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/*USART peripheral configuration structure*/

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


/*RCC peripheral configuration Registers*/

typedef struct
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
	uint32_t RESERVED;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
	uint32_t RESERVED3;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
	uint32_t RESERVED5;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
}RCC_RegDef_t;

/*
peripheral definations (peripheral base address typecasted to xxx_RegDef_t)
*/

#define GPIOA      ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB      ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF      ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG      ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH      ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI      ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define SPI1       ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2       ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3       ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1       ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASEADDR)

#define RCC        ((RCC_RegDef_t*)RCC_BASEADDR)

#define USART1     ((USART_RegDef_t*)USART1_BASEADDR)
#define USART6     ((USART_RegDef_t*)USART6_BASEADDR)
#define USART3     ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4      ((USART_RegDef_t*)UART4_BASEADDR)

/*CLOCK ENABLE MICROS FOR GPIOx PERIPHERALS*/

#define GPIOA_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN       (RCC->RCC_AHB1ENR |= ( 1 << 8 ))

/*CLOCK ENABLE MACROS FOR SPI PERIPHERAL*/

#define SPI1_PCLK_EN         (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN         (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN         (RCC->RCC_APB1ENR |= (1 << 15))

/*CLOCK ENABLE MACROS FOR I2C PERIPHERALS*/

#define I2C1_PCLK_EN         (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN         (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN         (RCC->RCC_APB1ENR |= (1 << 23))

/*CLOCK ENABLE MACROS FOR I2C PERIPHERALS*/

#define USART1_PCLK_EN       (RCC->RCC_APB2ENR |= (1 << 4))
#define USART6_PCLK_EN       (RCC->RCC_APB2ENR |= (1 << 5))
#define USART3_PCLK_EN       (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN        (RCC->RCC_APB1ENR |= (1 << 19))

/*CLOCK DISABLE MICROS FOR GPIOx PERIPHERALS*/

#define GPIOA_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI       (RCC->RCC_AHB1ENR &= ~( 1 << 8 ))

/*CLOCK DISABLE MACROS FOR SPIx PERIPHERALS*/

#define SPI1_PCLK_DI         (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI         (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI         (RCC->RCC_APB1ENR &= ~(1 << 15))

/*CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS*/
#define I2C1_PCLK_DI         (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI         (RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI         (RCC->RCC_APB1ENR &= ~(1 << 23))

/*CLOXK DISBALE MACROS FOR USARTx PERIPHERALS*/
#define USART1_PCLK_DI       (RCC->RCC_APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI       (RCC->RCC_APB2ENR &= ~(1 << 5))
#define USART3_PCLK_DI       (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI        (RCC->RCC_APB1ENR &= ~(1 << 19))


/*Macros to reset GPIO peripherals*/
#define GPIOA_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<0));   (RCC-> RCC_AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<1));   (RCC-> RCC_AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<2));   (RCC-> RCC_AHB1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<3));   (RCC-> RCC_AHB1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<4));   (RCC-> RCC_AHB1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<5));   (RCC-> RCC_AHB1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<6));   (RCC-> RCC_AHB1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<7));   (RCC-> RCC_AHB1RSTR &=~(1<<7));}while(0)
#define GPIOI_REG_RESET()   do{ (RCC-> RCC_AHB1RSTR |=(1<<8));   (RCC-> RCC_AHB1RSTR &=~(1<<8));}while(0)



/*Macros to reset SPI peripherals*/

#define SPI1_REG_RESET()    do{ (RCC-> RCC_APB2RSTR |=(1<<12));   (RCC-> RCC_APB1RSTR &=~(1<<12));}while(0)
#define SPI2_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<14));   (RCC-> RCC_APB1RSTR &=~(1<<14));}while(0)
#define SPI3_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<15));   (RCC-> RCC_APB1RSTR &=~(1<<15));}while(0)

/*Macros to reset I2C peripherals*/

#define I2C1_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<21));   (RCC-> RCC_APB1RSTR &=~(1<<21));}while(0)
#define I2C2_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<22));   (RCC-> RCC_APB1RSTR &=~(1<<22));}while(0)
#define I2C3_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<23));   (RCC-> RCC_APB1RSTR &=~(1<<23));}while(0)

/*Macros to reset GPIO peripherals*/

#define USART1_REG_RESET()    do{ (RCC-> RCC_APB2RSTR |=(1<<4));   (RCC-> RCC_APB1RSTR &=~(1<<4));}while(0)
#define USART6_REG_RESET()    do{ (RCC-> RCC_APB2RSTR |=(1<<5));   (RCC-> RCC_APB1RSTR &=~(1<<5));}while(0)
#define USART3_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<18));   (RCC-> RCC_APB1RSTR &=~(1<<18));}while(0)
#define UART4_REG_RESET()    do{ (RCC-> RCC_APB1RSTR |=(1<<19));   (RCC-> RCC_APB1RSTR &=~(1<<19));}while(0)



/*Bit Definations of USARTx_SR register*/

#define PE      (1<<0)
#define FE      (1<<1)
#define NF      (1<<2)
#define ORE     (1<<3)
#define IDLE    (1<<4)
#define RXNE    (1<<5)
#define TC      (1<<6)
#define TXE     (1<<7)
#define LBD     (1<<8)
#define CTS     (1<<9)

/*some generic macros*/

#define ENABLE     1
#define DISABLE    0
#define SET        ENABLE
#define RESET      DISABLE
#define Flag_Reset()   RESET
#define Flag_Set()     SET

#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
