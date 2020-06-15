#include "stm32f746_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enabled) {
	if (enabled == Enabled) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DI();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	// 3 = 0b0011
	uint32_t pinTwoBitMask = ~(3 << (2 * pGPIOHandle->pinConfig.pinNumber));
	uint32_t pinTwoBitShift = (2 * pGPIOHandle->pinConfig.pinNumber);
	uint32_t pinOneBitMask = ~(1 << pGPIOHandle->pinConfig.pinNumber);
	uint32_t pinOneBitShift = 1 << pGPIOHandle->pinConfig.pinNumber;

	if (pGPIOHandle->pinConfig.pinMode <= Analog) {
		// 1. configure the mode of GPIO pin
		pGPIOHandle->pGPIOx->MODER &= pinTwoBitMask;
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->pinConfig.pinMode
				<< pinTwoBitShift;
	} else {
		// TODO: implement IT modes
	}

	// 2. configure the output speed
	pGPIOHandle->pGPIOx->OSPEEDR &= pinTwoBitMask;
	pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->pinConfig.pinSpeed
			<< pinTwoBitShift;

	// 3. configure the PUPD settings
	pGPIOHandle->pGPIOx->PUPDR &= pinTwoBitMask;
	pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->pinConfig.pinPuPdControl
			<< pinTwoBitShift;

	// 4. configure the OPTYPE
	pGPIOHandle->pGPIOx->OTYPER &= pinOneBitMask;
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->pinConfig.pinOPType
			<< pinOneBitShift;

	// 5. configure the alternate functionality
	if (pGPIOHandle->pinConfig.pinMode == AlternateFunction) {
		uint8_t index = pGPIOHandle->pinConfig.pinNumber >> 3U;
		uint32_t pinFourBitMask = ~(2
				<< (4 * (pGPIOHandle->pinConfig.pinNumber & 7)));
		uint32_t pinFourBitShift = (4 * (pGPIOHandle->pinConfig.pinNumber & 7));

		pGPIOHandle->pGPIOx->AFR[index] &= pinFourBitMask;
		pGPIOHandle->pGPIOx->AFR[index] |= pGPIOHandle->pinConfig.pinAltFunMode
				<< pinFourBitShift;
	}
}

void GPIO_Deinit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	} else if (pGPIOx == GPIOJ) {
		GPIOJ_REG_RESET();
	} else if (pGPIOx == GPIOK) {
		GPIOK_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t value = (pGPIOx->IDR >> pinNumber) & 1;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value = pGPIOx->IDR & 0xffff;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t pinValue) {
	if (pinValue == Set) {
		pGPIOx->ODR |= 1 << pinNumber;
	} else {
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t portValue) {
	pGPIOx->ODR = portValue;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= 1 << pinNumber;
}

void GPIO_IRQConfig(void) {
}

void GPIO_IRQHandling(void) {
}
