#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include "stm32f407xx_gpio.h"
#include <stdint.h>

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
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2  * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);    // Clear BitFields
        pGPIOHandle->pGPIOx->MODER |= temp;                                                     // Set BitFields
    } else {
        // Interrupt Mode.
    }

    temp = 0;
    
    //2 Configure Speed Here
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;

    //3 Configure pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    //4 Configure optype 
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER;
    temp = 0;

    //5 Configure the alt functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
        
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        pGPIOHandle->pGPIOx->AF[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
    if (pGPIOx == GPIOA) GPIOA_REG_RST();
    else if (pGPIOx == GPIOB) GPIOB_REG_RST();
    else if (pGPIOx == GPIOC) GPIOC_REG_RST();
    else if (pGPIOx == GPIOD) GPIOD_REG_RST();
    else if (pGPIOx == GPIOE) GPIOE_REG_RST();
    else if (pGPIOx == GPIOF) GPIOF_REG_RST();
    else if (pGPIOx == GPIOG) GPIOG_REG_RST();
    else if (pGPIOx == GPIOH) GPIOH_REG_RST();
    else if (pGPIOx == GPIOI) GPIOI_REG_RST();
    else if (pGPIOx == GPIOJ) GPIOJ_REG_RST();
    else if (pGPIOx == GPIOK) GPIOK_REG_RST();

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}


#endif
