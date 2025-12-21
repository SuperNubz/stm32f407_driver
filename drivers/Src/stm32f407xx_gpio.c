#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include "stm32f407xx_gpio.h"

/*
* GPIO Pin Modes
*/
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTFN = 2,
    GPIO_MODE_ANALOG = 3

} GPIO_PinMode_t;

/*
* GPIO Output Type 
*/
typedef enum{

    GPIO_OPTYPE_PUSHPULL = 0,
    GPIO_OPTYPE_OPENDRAIN = 1,
}GPIO_OutputType_t;

/*
* GPIO Output Type 
*/
typedef enum{

    GPIO_OPSPEED_LOW = 0,
    GPIO_OPSPEED_MEDIUM = 1,
    GPIO_OPSPEED_HIGH = 2,
    GPIO_OPSPEED_VERYHIGH = 3,
}GPIO_OutputSpeed_t;

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

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
    uint32_t temp = 0;
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
        // Non-Interrupt Mode
        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2  * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER = temp;
    } else {
        // Interrupt Mode.
    }

    temp = 0;
    
    //2 Configure Speed Here

    //3 Configure pupd settings

    //4 Configure optype 

    //5 Configure the alt functionality
}