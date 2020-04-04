#include "WS2812.h"

#include "stm32f4xx_hal.h"

// Based on devararendy library on mbed.com

void sendReset() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	//wait_us(60);
}

void send0() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

//--------------hitung manual
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

//-----------------hitung manual
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
}
void send1() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

	//-----------------------------------
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	//-----------------hitung manual
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
}

void writeByte(uint8_t data) {
	for (int x = 7; x >= 0; x--) {
		if (data & (1 << x)) {
			//bit is high
			send1();
		} else {
			//bit is low
			send0();
		}
	}
}

void writeColor(uint32_t RGB) {
	writeByte((RGB & 0xFF00) >> 8); //green
	writeByte((RGB & 0xFF0000) >> 16); //red
	writeByte(RGB & 0xFF); //blue
}

void send1Color(uint32_t RGB) {
	writeColor(RGB);
	sendReset();
}
void sendColors(uint32_t * const colorBuffer, size_t len) {
	for (int x = 0; x < len; x++) {
		writeColor(colorBuffer[x]);
	}
	sendReset();
}
