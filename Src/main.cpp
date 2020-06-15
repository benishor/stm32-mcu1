#include "DigitalIn.h"
#include "DigitalOut.h"

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

void delay() {
	for (uint32_t i = 0; i < 500000; i++) {
	}
}

int main(void) {

	DigitalIn button(Port::I, PinNumber::Pin11, PinPullUpMode::None);
	DigitalOut led(Port::I, PinNumber::Pin1, PinOutputType::PushPull);

	while (1) {
		if (button.read()) {
			delay();
			led.toggle();
		}
	}
}
