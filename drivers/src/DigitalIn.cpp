#include "DigitalIn.h"

DigitalIn::DigitalIn(Port port, PinNumber pin, PinPullUpMode pullUpMode) {
	handle.pGPIOx = fromPort(port);
	handle.pinConfig.pinNumber = toPinNumber(pin);
	handle.pinConfig.pinMode = Input;
	handle.pinConfig.pinSpeed = High;
	handle.pinConfig.pinPuPdControl = toPinPullUp(pullUpMode);

	GPIO_PeriClockControl(handle.pGPIOx, Enabled);
	GPIO_Init(&handle);
}

bool DigitalIn::read() {
	return GPIO_ReadFromInputPin(handle.pGPIOx, handle.pinConfig.pinNumber) == Enabled;
}
