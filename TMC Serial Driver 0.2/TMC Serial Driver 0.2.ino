/*
 Name:		TMC_Serial_Driver_0.ino
 Created:	9/2/2023 7:58:50 PM
 Author:	danie
*/

#include "TMC_Serial.h"

TMC_Serial TMC2209(USART0, 115200);

// the setup function runs once when you press reset or power the board
void setup() {
	//Serial1.begin(115200);
	Serial.begin(115200);
	Serial.print("\n Master initiallized...");
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint32_t lastTransmit = 0;
	if (millis() - lastTransmit > 1000)
	{
		lastTransmit = millis();

		TMC_Serial::read_ticket* ticket = TMC2209.read(0, TMC_Serial::IOIN);

		delay(100);	// wait for transaction

		Serial.print("\n\n        SYNC      | D ADDRESS | R ADDRESS | DATA 0    | DATA 1    | DATA 2    | DATA 3    | SYNC");
		Serial.print("\n IOIN: ");
		uint8_t* reply = (uint8_t*)&ticket->request.read_data.memory.reply;

		for (size_t i = 0; i < 8; i++)
		{
			uint8_t& byte = reply[i];
			for (size_t b = 0; b < 8; b++)
			{
				if (b % 4 == 0)
					Serial.print(' ');
				Serial.print((byte & (1 << b)) >> b, BIN);
			}
			Serial.print(" |");
		}
		

		uint32_t bad_data = ticket->request.read_data.memory.reply.get_data();
		uint32_t data;
		((uint8_t*)&data)[0] = ticket->request.read_data.memory.reply.data3;
		((uint8_t*)&data)[1] = ticket->request.read_data.memory.reply.data2;
		((uint8_t*)&data)[2] = ticket->request.read_data.memory.reply.data1;
		((uint8_t*)&data)[3] = ticket->request.read_data.memory.reply.data0;

		Serial.print("\n DATA:                                     ");
		for (size_t i = 0; i < 4; i++)
		{
			uint8_t& byte = ((uint8_t*)&data)[i];
			for (size_t b = 0; b < 8; b++)
			{
				if (b % 4 == 0)
					Serial.print(' ');
				Serial.print((byte & (1 << b)) >> b, BIN);
			}
			Serial.print(" |");
		}
		Serial.print("\n        SYNC      | D ADDRESS | R ADDRESS | DATA 0    | DATA 1    | DATA 2    | DATA 3    | SYNC");

		delete ticket;
	}


	static uint32_t lastUpdate = 0;
	if (millis() - lastUpdate > 4000)
	{
		lastUpdate = millis();
		Serial.print("\n Master is active");
	}
}
