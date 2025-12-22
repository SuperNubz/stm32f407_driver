#include "stm32f407xx.h"

/*
* This is a Configuration structure for a GPIO pin
*/
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFuncMode;

} GPIO_PinConfig_t;

/*
* Handle structure for a GPIO Pin
*/
typedef struct 
{
    GPIO_RegDef_t *pGPIOx;               
    GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;

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

/***********************************************
* APIs supported by this driver
* For more information about the APIs check the function definitions
***************************************************/

/*
* Peripheral Clock Setup
*/
void GPIO_PeriClkCtrl(void);
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t En);

/*
* Init and De-init
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle);

/*
* Data read and write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t PinNUmber);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
* IRQ Configuration and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En);
void GPIO_IRQHandling(uint8_t PinNumber);

