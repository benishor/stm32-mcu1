#include "DigitalOut.h"

DigitalOut::DigitalOut(Port port, PinNumber pin, PinOutputType outputType) {
	handle.pGPIOx = fromPort(port);
	handle.pinConfig.pinNumber = toPinNumber(pin);
	handle.pinConfig.pinMode = Output;
	handle.pinConfig.pinSpeed = High;
	handle.pinConfig.pinOType = toOutputType(outputType);

	GPIO_PeriClockControl(handle.pGPIOx, Enabled);
	GPIO_Init(&handle);
}

void DigitalOut::toggle() {
	GPIO_ToggleOutputPin(handle.pGPIOx, handle.pinConfig.pinNumber);
}
