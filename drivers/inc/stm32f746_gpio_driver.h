#ifndef STM32F746XX_H
#define STM32F746XX_H

#include "stm32f746xx.h"


enum class Port {
	A, B, C, D, E, F, G, H, I, J, K
};

enum class PinPullUpMode {
	None, PullUp, PullDown
};

enum class PinOutputType {
	PushPull,
	OpenDrain
};

enum class PinNumber {
	Pin0,
	Pin1,
	Pin2,
	Pin3,
	Pin4,
	Pin5,
	Pin6,
	Pin7,
	Pin8,
	Pin9,
	Pin10,
	Pin11,
	Pin12,
	Pin13,
	Pin14,
	Pin15,
	Pin16
};


GPIO_RegDef_t* fromPort(Port port);
uint8_t toPinPullUp(PinPullUpMode mode);
uint8_t toPinNumber(PinNumber pin);
uint8_t toOutputType(PinOutputType outputType);

enum EnabledOrDisabledEnum {
	Disabled = 0, Enabled = 1
};

enum PinStateEnum {
	Set = Enabled, Reset = Disabled
};

enum PortNumberEnum {
	GpioA = 0,
	GpioB = 1,
	GpioC = 2,
	GpioD = 3,
	GpioE = 4,
	GpioF = 5,
	GpioG = 6,
	GpioH = 7,
	GpioI = 8,
	GpioJ = 9,
	GpioK = 10,
};

enum PinNumberEnum {
	Pin0 = 0,
	Pin1,
	Pin2,
	Pin3,
	Pin4,
	Pin5,
	Pin6,
	Pin7,
	Pin8,
	Pin9,
	Pin10,
	Pin11,
	Pin12,
	Pin13,
	Pin14,
	Pin15
};

enum PinModeEnum {
	Input = 0,
	Output = 1,
	AlternateFunction = 2,
	Analog = 3,
	InterruptFallingEdge = 4,
	InterruptRisingEdge = 5,
	InterruptRisingEdgeFallingEdgeTrigger = 6

};

enum PinSpeedEnum {
	Low = 0, Medium = 1, High = 2, VeryHigh = 3
};

enum PullUpPullDownControlEnum {
	NoPullUpNoPullDown = 0, PullUp = 1, PullDown = 2
};

enum PinOutputTypeEnum {
	PushPull = 0, OpenDrain = 1
};

typedef struct {
	uint8_t pinNumber;
	uint8_t pinMode;
	uint8_t pinSpeed;
	uint8_t pinPuPdControl;
	uint8_t pinOType;
	uint8_t pinAltFunMode;

} GPIO_PinConfig_t;

typedef struct {
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t pinConfig;

} GPIO_Handle_t;


#define GPIOA_REG_RESET() do { RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RESET() do { RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while(0)
#define GPIOC_REG_RESET() do { RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while(0)
#define GPIOD_REG_RESET() do { RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while(0)
#define GPIOE_REG_RESET() do { RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while(0)
#define GPIOF_REG_RESET() do { RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); } while(0)
#define GPIOG_REG_RESET() do { RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); } while(0)
#define GPIOH_REG_RESET() do { RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while(0)
#define GPIOI_REG_RESET() do { RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); } while(0)
#define GPIOJ_REG_RESET() do { RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9); } while(0)
#define GPIOK_REG_RESET() do { RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10); } while(0)

/**
 * Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enabled);

/**
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * Sets the register to its default state (reset value)
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/**
 * Data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t pinValue);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t portValue);

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif
