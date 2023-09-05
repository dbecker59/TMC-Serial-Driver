/*
 Name:		TMC_Serial_Driver_0.ino
 Created:	9/2/2023 7:58:50 PM
 Author:	danie
*/

#include "TMC_Serial.h"

TMC_Serial TMC2209(USART0, 115200);
TMC_Serial TMC2209_1(USART0, 115200);

// the setup function runs once when you press reset or power the board
void setup() {
	//Serial1.begin(115200);
	Serial.begin(115200);
	Serial.print("\n Master initiallized...");
	Serial.print("\n USART default priority: ");
	Serial.print(NVIC_GetPriority(USART1_IRQn));
	Serial.print("\n USART new priority: ");
	Serial.print(NVIC_GetPriority(USART0_IRQn));
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint32_t lastTransmit = -4000;
	micros();

	Serial.print("\n Setting RPM to 1");
	TMC2209.write(0, TMC_Serial::VACTUAL, 1600);
	TMC2209.write(0, TMC_Serial::GCONF, (1 << 3) | (1 << 1));
	delay(1000);

	Serial.print("\n Setting RPM to 2");
	TMC2209.write(0, TMC_Serial::VACTUAL, 3200);
	TMC2209_1.write(0, TMC_Serial::GCONF, (0 << 3) | (1 << 1));
	delay(1000);

	Serial.print("\n Setting RPM to 3");
	TMC2209_1.write(0, TMC_Serial::VACTUAL, 4800);
	TMC2209_1.write(0, TMC_Serial::GCONF, (1 << 3) | (1 << 1));
	delay(1000);

	Serial.print("\n Setting RPM to 4");
	TMC2209_1.write(0, TMC_Serial::VACTUAL, 6400);
	TMC2209.write(0, TMC_Serial::GCONF, (0 << 3) | (1 << 1));
	delay(1000);

	Serial.print("\n Setting RPM to 0");
	TMC2209.write(0, TMC_Serial::VACTUAL, 0);
	delay(1000);


	static uint32_t lastUpdate = 0;
	if (millis() - lastUpdate > 4000)
	{
		lastUpdate = millis();
		Serial.print("\n Master is active");
	}
}