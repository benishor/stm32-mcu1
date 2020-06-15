#ifndef DIGITAL_OUT_H
#define DIGITAL_OUT_H

#include "stm32f746_gpio_driver.h"

class DigitalOut {
public:
	DigitalOut(Port port, PinNumber pinNumber, PinOutputType outputType=PinOutputType::PushPull);

	void write(bool enabled);
	void toggle();

private:
	GPIO_Handle_t handle;
};

#endif
