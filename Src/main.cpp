#include "stm32f746xx.h"
#include "stm32f746_gpio_driver.h"
#include "DigitalIn.h"

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

void delay() {
	for (uint32_t i = 0; i < 500000; i++) {
	}
}

int main(void) {

	GPIO_PeriClockControl(GPIOI, Enabled);

	GPIO_Handle_t ledHandle;
	ledHandle.pGPIOx = GPIOI;
	ledHandle.pinConfig.pinNumber = Pin1;
	ledHandle.pinConfig.pinMode = Output;
	ledHandle.pinConfig.pinSpeed = High;
	ledHandle.pinConfig.pinOPType = PushPull;

	GPIO_Init(&ledHandle);

	DigitalIn button(Port::I, PinNumberEnum::Pin11, PinPullUpMode::None);

	while (1) {
		if (button.read()) {
			delay();
			GPIO_ToggleOutputPin(ledHandle.pGPIOx,
					ledHandle.pinConfig.pinNumber);
		}
	}
}
