#include "stm32f746xx.h"
#include "stm32f746_gpio_driver.h"

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

	GPIO_Handle_t btnHandle;
	btnHandle.pGPIOx = GPIOI;
	btnHandle.pinConfig.pinNumber = Pin11;
	btnHandle.pinConfig.pinMode = Input;
	btnHandle.pinConfig.pinSpeed = High;
	btnHandle.pinConfig.pinPuPdControl = NoPullUpNoPullDown;

	GPIO_Init(&btnHandle);

	while (1) {
		if (GPIO_ReadFromInputPin(btnHandle.pGPIOx,
				btnHandle.pinConfig.pinNumber) == Enabled) {
			delay();
			GPIO_ToggleOutputPin(ledHandle.pGPIOx,
					ledHandle.pinConfig.pinNumber);
		}
	}
}
