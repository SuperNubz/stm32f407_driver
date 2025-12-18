/*
 * stm32f407xx.h
 *
 *  Created on: Dec 2, 2025
 *      Author: preus
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#define FLASH_BASEADDR
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x20000000U + 0x2001C001U
#define	ROM_BASEADDR			0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

#define APB1_BASEADDR			0x40000000U
#define APB2_BASEADDR			0x40010000U
#define AHB1_BASEADDR			0x40020000U
#define AHB2_BASEADDR			0x50000000U

/*
 * Base addresses of peripherals which are hanging on APB1 bus
*/
#define TIM2_BASEADDR					APB1_BASEADDR + 0x0000U
#define TIM3_BASEADDR					APB1_BASEADDR + 0x0400U
#define TIM4_BASEADDR					APB1_BASEADDR + 0x0800U
#define TIM5_BASEADDR					APB1_BASEADDR + 0x0C00U
#define TIM6_BASEADDR					APB1_BASEADDR + 0x1000U
#define TIM7_BASEADDR					APB1_BASEADDR + 0x1400U
#define TIM12_BASEADDR					APB1_BASEADDR + 0x1800U
#define TIM13_BASEADDR					APB1_BASEADDR + 0x1C00U
#define TIM14_BASEADDR					APB1_BASEADDR + 0x2000U
#define RTK_BKP_BASEADDR				APB1_BASEADDR + 0x2800U
#define WWDG_BASEADDR					APB1_BASEADDR + 0x2C00U
#define IWDG_BASEADDR					APB1_BASEADDR + 0x3000U
#define I2S2EXT_BASEADDR				APB1_BASEADDR + 0x3400U
#define SPI2_I2S2_BASEADDR				APB1_BASEADDR + 0x3800U
#define SPI3_I2S3_BASEADDR				APB1_BASEADDR + 0x3C00U
#define I2S3EXT_BASEADDR				APB1_BASEADDR + 0x4000U
#define USART2_BASEADDR					APB1_BASEADDR + 0x4400U
#define USART3_BASEADDR					APB1_BASEADDR + 0x4800U
#define UART4_BASEADDR					APB1_BASEADDR + 0x4C00U
#define UART5_BASEADDR					APB1_BASEADDR + 0x5000U
#define I2C1_BASEADDR					APB1_BASEADDR + 0x5400U
#define I2C2_BASEADDR					APB1_BASEADDR + 0x5800U
#define I2C3_BASEADDR					APB1_BASEADDR + 0x5C00U
#define CAN1_BASEADDR					APB1_BASEADDR + 0x6400U
#define CAN2_BASEADDR					APB1_BASEADDR + 0x6800U
#define PWR_BASEADDR					APB1_BASEADDR + 0x7000U
#define DAC_BASEADDR					APB1_BASEADDR + 0x7400U
#define UART7_BASEADDR					APB1_BASEADDR + 0x7800U
#define UART8_BASEADDR					APB1_BASEADDR + 0x7C00U

/*
 * Base addresses of peripherals which are hanging on APB2 bus
*/
#define TIM1_BASEADDR					APB2_BASEADDR + 0x0000U
#define TIM8_BASEADDR					APB2_BASEADDR + 0x0400U
#define USART1_BASEADDR					APB2_BASEADDR + 0x1000U
#define USART6_BASEADDR					APB2_BASEADDR + 0x1400U
#define ADC1_ADC2_ADC3_BASEADDR			APB2_BASEADDR + 0x2000U
#define SDIO_BASEADDR					APB2_BASEADDR + 0x2C00U
#define SPI1_BASEADDR					APB2_BASEADDR + 0x3000U
#define SPI4_BASEADDR					APB2_BASEADDR + 0x3400U
#define SYSCFG_BASEADDR					APB2_BASEADDR + 0x3800U
#define EXTI_BASEADDR					APB2_BASEADDR + 0x3C00U
#define TIM9_BASEADDR					APB2_BASEADDR + 0x4000U
#define TIM10_BASEADDR					APB2_BASEADDR + 0x4400U
#define TIM11_BASEADDR					APB2_BASEADDR + 0x4800U
#define SPI5_BASEADDR					APB2_BASEADDR + 0x5000U
#define SPI6_BASEADDR					APB2_BASEADDR + 0x5400U
#define SAI1_BASEADDR					APB2_BASEADDR + 0x5800U
#define LCD_TFT_BASEADDR				APB2_BASEADDR + 0x6800U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
*/
#define GPIOA_BASEADDR					AHB1_BASEADDR + 0x0000U
#define GPIOB_BASEADDR					AHB1_BASEADDR + 0x0400U
#define GPIOC_BASEADDR					AHB1_BASEADDR + 0x0800U
#define GPIOD_BASEADDR					AHB1_BASEADDR + 0x0C00U
#define GPIOE_BASEADDR					AHB1_BASEADDR + 0x1000U
#define GPIOF_BASEADDR					AHB1_BASEADDR + 0x1400U
#define GPIOG_BASEADDR					AHB1_BASEADDR + 0x1800U
#define GPIOH_BASEADDR					AHB1_BASEADDR + 0x1C00U
#define GPIOI_BASEADDR					AHB1_BASEADDR + 0x2000U
#define GPIOJ_BASEADDR					AHB1_BASEADDR + 0x2400U
#define GPIOK_BASEADDR					AHB1_BASEADDR + 0x2800U
#define CRC_BASEADDR					AHB1_BASEADDR + 0x3000U
#define RCC_BASEADDR					AHB1_BASEADDR + 0x3800U
#define FLASHINTREG_BASEADDR			AHB1_BASEADDR + 0x3C00U
#define BKPSRAM_BASEADDR				AHB1_BASEADDR + 0x4000U
#define DMA1_BASEADDR					AHB1_BASEADDR + 0x6000U
#define DMA2_BASEADDR					AHB1_BASEADDR + 0x6400U
#define ETHERNET_MAC_BASEADDR			AHB1_BASEADDR + 0x8000U
#define DMA2D_BASEADDR					AHB1_BASEADDR + 0xB000U
#define USBOTGHS_BASEADDR				AHB1_BASEADDR + 0x2800U

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
*/
#define USBOTGFS						AHB2_BASEADDR + 0x00000
#define DCMI							AHB2_BASEADDR + 0x50000
#define CRYP							AHB2_BASEADDR + 0x60000
#define HASH							AHB2_BASEADDR + 0x60400
#define RNG								AHB2_BASEADDR + 0x60800

/*
*   Peripheral Registers Struct Definition for GPIO
*/
typedef struct 
{
    uint32_t MODER;
    uint32_t OTYPER;
    uint32_t OSPEEDR;
    uint32_t PUPDR;
    uint32_t IDR;
    uint32_t ODR;
    uint32_t BSRR;
    uint32_t LCKR;
    uint32_t AF[2];

} GPIO_RegDef_t;


/*
* Peripheral Register Struct Definition for RCC
*/
typedef struct
{
    uint32_t CR;
    uint32_t PLLCFGR;
    uint32_t CFGR;
    uint32_t CIR;
    uint32_t AHB1RSTR;
    uint32_t AHB2RSTR;
    uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    uint32_t APB1RSTR;
    uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    uint32_t AHB1ENR;
    uint32_t AHB2ENR;
    uint32_t AHB3ENR;
    uint32_t RESERVED2;
    uint32_t APB1ENR;
    uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    uint32_t AHB1LPENR;
    uint32_t AHB2LPENR;
    uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    uint32_t APB1LPENR;
    uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    uint32_t BDCR;
    uint32_t CSR;
    uint32_t SSCGR;
    uint32_t PLLI2SCFGR;
    uint32_t PLLSAICFGR;
    uint32_t DCKCFGR;

} RCC_RegDef_t;

/*
* Macros for GPIO Peripherals
*/
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ  ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK  ((GPIO_RegDef_t*)GPIOK_BASEADDR)

/*
* Macro for RCC
*/
#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

/*
* Macros for Enabling GPIOx peripheral clock
*/
#define GPIOA_PCLK_EN   (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN   (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN   (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN   (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN   (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN   (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN   (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN   (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN   (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN   (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN   (RCC->AHB1ENR |= (1 << 10))

/*
* Macros for Enabling I2Cx peripheral clock
*/
#define I2C1_PCLK_EN   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN   (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN   (RCC->APB1ENR |= (1 << 23))

/*
* Macros for Enabling SPIx peripheral clock
*/
#define SPI1_PCLK_EN   (RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN   (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN   (RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN   (RCC->APB2ENR |= (1 << 21))
#define SPI2_PCLK_EN   (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN   (RCC->APB1ENR |= (1 << 15))

/*
* Macros for Enabling USARTx peripheral clock
*/
#define USART2_PCLK_EN   (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN   (RCC->APB1ENR |= (1 << 18))
#define USART1_PCLK_EN   (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN   (RCC->APB2ENR |= (1 << 6))

/*
* Macros for Enabling SYSCFG peripheral clock
*/
#define SYSCFG_PCLK_EN   (RCC->APB2ENR |= (1 << 6))

/*
* Macros for DISABLING GPIOx peripheral clock
*/
#define GPIOA_PCLK_DIS   (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS   (RCC->AHB1ENR &= (1 << 1))
#define GPIOC_PCLK_DIS   (RCC->AHB1ENR &= (1 << 2))
#define GPIOD_PCLK_DIS   (RCC->AHB1ENR &= (1 << 3))
#define GPIOE_PCLK_DIS   (RCC->AHB1ENR &= (1 << 4))
#define GPIOF_PCLK_DIS   (RCC->AHB1ENR &= (1 << 5))
#define GPIOG_PCLK_DIS   (RCC->AHB1ENR &= (1 << 6))
#define GPIOH_PCLK_DIS   (RCC->AHB1ENR &= (1 << 7))
#define GPIOI_PCLK_DIS   (RCC->AHB1ENR &= (1 << 8))
#define GPIOJ_PCLK_DIS   (RCC->AHB1ENR &= (1 << 9))
#define GPIOK_PCLK_DIS   (RCC->AHB1ENR &= (1 << 10))


#endif /* STM32F407XX_H_ */
