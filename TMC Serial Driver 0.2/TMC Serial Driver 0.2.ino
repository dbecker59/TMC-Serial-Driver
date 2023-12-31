/*
 Name:		TMC_Serial_Driver_0.ino
 Created:	9/2/2023 7:58:50 PM
 Author:	danie
*/

#include "TMC_Serial.h"

TMC_Serial TMC2209(USART0, 460800);
// the setup function runs once when you press reset or power the board
void setup() {
	//Serial1.begin(115200);
	Serial.begin(115200);
	Serial.print("\n Master initiallized...");
}

// the loop function runs over and over again until power down or reset
void loop() {
	TMC2209.write(0, TMC_Serial::GCONF, (1 << 1) | (1 << 7));
	TMC2209.write(0, TMC_Serial::CHOPCONF, 0b00010110000000000000000001010011);
	TMC2209.write(0, TMC_Serial::IHOLD_IRUN, (2 << 0) | (31 << 8) | (1 << 16));
	TMC2209.write(0, TMC_Serial::TPWMTHRS, 200);

	uint32_t read_data;
	volatile TMC_Serial::read_ticket* r_ticket = TMC2209.read(0, TMC_Serial::IOIN, TMC_Serial::storeRegisterAt, &read_data);
	while (!(r_ticket->transfer_complete()))
	{ /* wait for transfer */ }

	Serial.print("\n\n ENA: ");
	Serial.print(read_data & (1 << 0) ? "Disabled" : "Enabled");
	Serial.print("\n Status: ");
	Serial.print(r_ticket->status);
	Serial.print("\n Data: ");
	Serial.print(read_data, BIN);
	delete r_ticket;

	for (int32_t i = 0000; i < 20000; )
	{
		TMC2209.write(0, TMC_Serial::VACTUAL, i);
		i += 20;
		delay(10);
	}
	delay(2000);
	for (int32_t i = 20000; i > 0; )
	{
		i -= 200;
		TMC2209.write(0, TMC_Serial::VACTUAL, i);
	}

	TMC2209.write(0, TMC_Serial::VACTUAL, 0);
	delay(2000);
}