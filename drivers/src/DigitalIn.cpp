#include "DigitalIn.h"

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

uint8_t toPinNumber(PinNumberEnum pin) {
	switch (pin) {
	case PinNumberEnum::Pin0:
		return Pin0;
	case PinNumberEnum::Pin1:
		return Pin1;
	case PinNumberEnum::Pin2:
		return Pin2;
	case PinNumberEnum::Pin3:
		return Pin3;
	case PinNumberEnum::Pin4:
		return Pin4;
	case PinNumberEnum::Pin5:
		return Pin5;
	case PinNumberEnum::Pin6:
		return Pin6;
	case PinNumberEnum::Pin7:
		return Pin7;
	case PinNumberEnum::Pin8:
		return Pin8;
	case PinNumberEnum::Pin9:
		return Pin9;
	case PinNumberEnum::Pin10:
		return Pin10;
	case PinNumberEnum::Pin11:
		return Pin11;
	case PinNumberEnum::Pin12:
		return Pin12;
	case PinNumberEnum::Pin13:
		return Pin13;
	case PinNumberEnum::Pin14:
		return Pin14;
	default:
		return Pin15;
	}
}

DigitalIn::DigitalIn(Port port, PinNumberEnum pin, PinPullUpMode pullUpMode) {
	handle.pGPIOx = fromPort(port);
	handle.pinConfig.pinNumber = toPinNumber(pin);
	handle.pinConfig.pinMode = Input;
	handle.pinConfig.pinSpeed = High;
	handle.pinConfig.pinPuPdControl = toPinPullUp(pullUpMode);

	GPIO_PeriClockControl(handle.pGPIOx, Enabled);
	GPIO_Init(&handle);
}

bool DigitalIn::read() {
	return GPIO_ReadFromInputPin(handle.pGPIOx, handle.pinConfig.pinNumber)
			== Enabled;
}
