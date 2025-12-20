#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include "stm32f407xx_gpio.h"

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t En){
    if (En == ENABLE){
        if (pGPIOx  == GPIOA){
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB){
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC){
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD){
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE){
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF){
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOG){
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOH){
            GPIOH_PCLK_EN();
        } else if (pGPIOx == GPIOI){
            GPIOI_PCLK_EN();
        } else if (pGPIOx == GPIOJ){
            GPIOJ_PCLK_EN();
        } else if (pGPIOx == GPIOK){
            GPIOK_PCLK_EN();
        }
    } else if (En == DISABLE){
        if (pGPIOx  == GPIOA){
            GPIOA_PCLK_DIS();
        } else if (pGPIOx == GPIOB){
            GPIOB_PCLK_DIS();
        } else if (pGPIOx == GPIOC){
            GPIOC_PCLK_DIS();
        } else if (pGPIOx == GPIOD){
            GPIOD_PCLK_DIS();
        } else if (pGPIOx == GPIOE){
            GPIOE_PCLK_DIS();
        } else if (pGPIOx == GPIOF){
            GPIOF_PCLK_DIS();
        } else if (pGPIOx == GPIOG){
            GPIOG_PCLK_DIS();
        } else if (pGPIOx == GPIOH){
            GPIOH_PCLK_DIS();
        } else if (pGPIOx == GPIOI){
            GPIOI_PCLK_DIS();
        } else if (pGPIOx == GPIOJ){
            GPIOJ_PCLK_DIS();
        } else if (pGPIOx == GPIOK){
            GPIOK_PCLK_DIS();
        }
    }
} 