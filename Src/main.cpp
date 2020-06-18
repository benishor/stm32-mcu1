#include "DigitalIn.h"
#include "DigitalOut.h"

void delay() {
	for (uint32_t i = 0; i < 500000; i++) {
	}
}

extern "C" {
// need to enable FPU access, otherwise FP operations will fail
#define CPACR (uint32_t *)0xE000ED88
void SystemInit() {
	*CPACR |= ((3UL << 20) | (3UL << 22)); // set CP10 and CP11 to full access
}
}

GPIO_Handle_t led, btn;

int main(void) {

	// button: I_11
	// led: I_1

	led.pGPIOx = GPIOI;
	led.pinConfig.pinNumber = 1;
	led.pinConfig.pinMode = Output;
	led.pinConfig.pinSpeed = Low;
	led.pinConfig.pinOType = PushPull;
	led.pinConfig.pinPuPdControl = NoPullUpNoPullDown;

	GPIO_PeriClockControl(led.pGPIOx, Enabled);
	GPIO_Init(&led);

	btn.pGPIOx = GPIOI;
	btn.pinConfig.pinNumber = 11;
	btn.pinConfig.pinMode = InterruptRisingEdge;
	btn.pinConfig.pinSpeed = Low;
	btn.pinConfig.pinPuPdControl = NoPullUpNoPullDown;

	GPIO_PeriClockControl(btn.pGPIOx, Enabled);
	GPIO_Init(&btn);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, Enabled);

	while (1) {
	}

//	float a = 1.2345f;
//	float b = 2.3456f;
//	float c = a / b;
//
//	DigitalIn button(Port::I, PinNumber::Pin11, PinPullUpMode::None);
//	DigitalOut led(Port::I, PinNumber::Pin1, PinOutputType::PushPull);
//
//	while (1) {
//		if (button.read()) {
//			delay();
//			led.toggle();
//			c += 1.1f;
//		}
//	}
}

extern "C" {
void EXTI15_10_IRQHandler(void) {
	// handle the interrupt
	GPIO_IRQHandling(btn.pinConfig.pinNumber);

	GPIO_ToggleOutputPin(led.pGPIOx, led.pinConfig.pinNumber);
}
}
