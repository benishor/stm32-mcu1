#ifndef DIGITAL_IN_H_
#define DIGITAL_IN_H_

#include "stm32f746_gpio_driver.h"


class DigitalIn {
public:

	DigitalIn(Port port, PinNumber pin, PinPullUpMode pullUpMode =
			PinPullUpMode::PullUp);
	bool read();

private:
	GPIO_Handle_t handle;
};

#endif
