#include "DigitalIn.h"
#include "DigitalOut.h"

void delay() {
	for (uint32_t i = 0; i < 500000; i++) {
	}
}

// need to enable FPU access, otherwise FP operations will fail
#define CPACR (uint32_t *)0xE000ED88

//__attribute__((constructor)) void initializeFpu(void) {
//	*CPACR |= ((3UL << 20) | (3UL << 22)); // set CP10 and CP11 to full access
//}

[[gnu::constructor(0)]] void initializeFpu(void) {
	*CPACR |= ((3UL << 20) | (3UL << 22)); // set CP10 and CP11 to full access
}

//extern "C" {
//
//	void SystemInit() {
//		*CPACR |= ((3UL << 20) | (3UL << 22)); // set CP10 and CP11 to full access
//	}
//}

int main(void) {

	float a = 1.2345f;
	float b = 2.3456f;
	float c = a / b;

	DigitalIn button(Port::I, PinNumber::Pin11, PinPullUpMode::None);
	DigitalOut led(Port::I, PinNumber::Pin1, PinOutputType::PushPull);

	while (1) {
		if (button.read()) {
			delay();
			led.toggle();
			c += 1.1f;
		}
	}
}
