#ifndef DIGITAL_IN_H_
#define DIGITAL_IN_H_

#include "stm32f746_gpio_driver.h"

enum class Port {
	A, B, C, D, E, F, G, H, I, J, K
};

enum class PinPullUpMode {
	None, PullUp, PullDown
};

enum class PinNumberEnum {
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

class DigitalIn {
public:

	DigitalIn(Port port, PinNumberEnum pin, PinPullUpMode pullUpMode =
			PinPullUpMode::PullUp);
	bool read();

private:
	GPIO_Handle_t handle;
};

#endif
