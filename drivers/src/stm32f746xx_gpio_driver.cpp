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
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->pinConfig.pinMode << pinTwoBitShift;
	} else {
		// TODO: implement IT modes
		if (pGPIOHandle->pinConfig.pinMode == InterruptFallingEdge) {
			// configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);
		} else if (pGPIOHandle->pinConfig.pinMode == InterruptRisingEdge) {
			// configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		} else {
			// configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t exticr_index = pGPIOHandle->pinConfig.pinNumber >> 2;
		uint8_t port_index = portToIndex(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		uint32_t value = port_index << (4 * (pGPIOHandle->pinConfig.pinNumber & 3));
		SYSCFG->EXTICR[exticr_index] = value;

		// 3. enable the EXTI interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= (1 << pGPIOHandle->pinConfig.pinNumber);
	}

	// 2. configure the output speed
	pGPIOHandle->pGPIOx->OSPEEDR &= pinTwoBitMask;
	pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->pinConfig.pinSpeed << pinTwoBitShift;

	// 3. configure the PUPD settings
	pGPIOHandle->pGPIOx->PUPDR &= pinTwoBitMask;
	pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->pinConfig.pinPuPdControl << pinTwoBitShift;

	// 4. configure the OTYPE
	pGPIOHandle->pGPIOx->OTYPER &= pinOneBitMask;
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->pinConfig.pinOType << pinOneBitShift;

	// 5. configure the alternate functionality
	if (pGPIOHandle->pinConfig.pinMode == AlternateFunction) {
		uint8_t index = pGPIOHandle->pinConfig.pinNumber >> 3U;
		uint32_t pinFourBitMask = ~(2 << (4 * (pGPIOHandle->pinConfig.pinNumber & 7)));
		uint32_t pinFourBitShift = (4 * (pGPIOHandle->pinConfig.pinNumber & 7));

		pGPIOHandle->pGPIOx->AFR[index] &= pinFourBitMask;
		pGPIOHandle->pGPIOx->AFR[index] |= pGPIOHandle->pinConfig.pinAltFunMode << pinFourBitShift;
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

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t pinValue) {
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

void GPIO_IRQHandling(uint8_t pinNumber) {
	// clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << pinNumber)) {
		// clear
		EXTI->PR |= (1 << pinNumber);
	}
}

GPIO_RegDef_t* fromPort(Port port) {
	switch (port) {
	case Port::A:
		return GPIOA;
	case Port::B:
		return GPIOB;
	case Port::C:
		return GPIOC;
	case Port::D:
		return GPIOD;
	case Port::E:
		return GPIOE;
	case Port::F:
		return GPIOF;
	case Port::G:
		return GPIOG;
	case Port::H:
		return GPIOH;
	case Port::I:
		return GPIOI;
	case Port::J:
		return GPIOJ;
	case Port::K:
		return GPIOK;
	default:
		return GPIOA;
	}
}

uint8_t toPinPullUp(PinPullUpMode mode) {
	switch (mode) {
	case PinPullUpMode::PullUp:
		return PullUp;
	case PinPullUpMode::PullDown:
		return PullDown;
	default:
		return NoPullUpNoPullDown;
	}
}

uint8_t toPinNumber(PinNumber pin) {
	switch (pin) {
	case PinNumber::Pin0:
		return Pin0;
	case PinNumber::Pin1:
		return Pin1;
	case PinNumber::Pin2:
		return Pin2;
	case PinNumber::Pin3:
		return Pin3;
	case PinNumber::Pin4:
		return Pin4;
	case PinNumber::Pin5:
		return Pin5;
	case PinNumber::Pin6:
		return Pin6;
	case PinNumber::Pin7:
		return Pin7;
	case PinNumber::Pin8:
		return Pin8;
	case PinNumber::Pin9:
		return Pin9;
	case PinNumber::Pin10:
		return Pin10;
	case PinNumber::Pin11:
		return Pin11;
	case PinNumber::Pin12:
		return Pin12;
	case PinNumber::Pin13:
		return Pin13;
	case PinNumber::Pin14:
		return Pin14;
	default:
		return Pin15;
	}
}

uint8_t toOutputType(PinOutputType outputType) {
	switch (outputType) {
	case PinOutputType::PushPull:
		return PushPull;
	case PinOutputType::OpenDrain:
		return OpenDrain;
	default:
		return 0; // should never happen
	}
}

uint8_t portToIndex(GPIO_RegDef_t *port) {
	if (port == GPIOA)
		return 0;
	if (port == GPIOB)
		return 1;
	if (port == GPIOC)
		return 2;
	if (port == GPIOD)
		return 3;
	if (port == GPIOE)
		return 4;
	if (port == GPIOF)
		return 5;
	if (port == GPIOG)
		return 6;
	if (port == GPIOH)
		return 7;
	if (port == GPIOI)
		return 8;
	if (port == GPIOJ)
		return 9;
	if (port == GPIOK)
		return 10;

	return 0;
}

void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t enabled) {
	if (enabled) {
		NVIC_ISER[irqNumber / 32] |= 1 << (irqNumber & 31);
	} else {
		NVIC_ICER[irqNumber / 32] |= 1 << (irqNumber & 31);
	}
}

void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority) {
	NVIC_IPR[irqNumber / 4] &= ~(0xff << (8 * (irqNumber & 3)));
	// only priorities 0-15 are implemented and they are stored in the upper nibble of each NVIC_IPR priority slot
	uint8_t shiftAmount = (8 * irqNumber & 3) + (8 - NO_PR_BITS_IMPLEMENTED);
	NVIC_IPR[irqNumber / 4] |= ((irqPriority & 0xf) << shiftAmount);
}
