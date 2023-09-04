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
	if (millis() - lastTransmit > 4000)
	{
		lastTransmit = millis();
		Serial.print("\n ========================================================================================================");

		// Get VACTUAL before write =====================================================================================================================
		TMC_Serial::read_ticket* read_ticket0 = TMC2209.read(0, TMC_Serial::TSTEP);

		delay(100);	// wait for transaction

		Serial.print("\n\n Field:    SYNC      | D ADDRESS | R ADDRESS | DATA 3    | DATA 2    | DATA 1    | DATA 0    | CRC\n          ");
		uint8_t* reply = (uint8_t*)&read_ticket0->request.read_data.memory.reply;

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


		uint32_t data = read_ticket0->request.read_data.memory.reply.get_data();

		Serial.print("\n                                              ");
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
		Serial.print("\n  Data:                                        DATA 0    | DATA 1    | DATA 2    | DATA 3    ");

		delete read_ticket0;

		// Write to VACTUAL =====================================================================================================================
		

		static bool driven = true;
		static bool dir = true;

		TMC_Serial::write_ticket* write_ticket = TMC2209.write(0, TMC_Serial::GCONF, 0x00000008 * dir);
		if (driven)
			dir = !dir;

		delay(100);

		Serial.print("\n\n FIELD:    SYNC      | D ADDRESS | R ADDRESS | DATA 3    | DATA 2    | DATA 1    | DATA 0    | CRC\n          ");
		uint8_t* transmission = (uint8_t*)&write_ticket->request.write_data.transmission;

		for (size_t i = 0; i < 8; i++)
		{
			uint8_t& byte = transmission[i];
			for (size_t b = 0; b < 8; b++)
			{
				if (b % 4 == 0)
					Serial.print(' ');
				Serial.print((byte & (1 << b)) >> b, BIN);
			}
			Serial.print(" |");
		}

		delete write_ticket;


		write_ticket = TMC2209.write(0, TMC_Serial::VACTUAL, 1600 * driven);
		driven = !driven;
		delay(100);

		Serial.print("\n\n FIELD:    SYNC      | D ADDRESS | R ADDRESS | DATA 3    | DATA 2    | DATA 1    | DATA 0    | CRC\n          ");
		transmission = (uint8_t*)&write_ticket->request.write_data.transmission;

		for (size_t i = 0; i < 8; i++)
		{
			uint8_t& byte = transmission[i];
			for (size_t b = 0; b < 8; b++)
			{
				if (b % 4 == 0)
					Serial.print(' ');
				Serial.print((byte & (1 << b)) >> b, BIN);
			}
			Serial.print(" |");
		}
		
		delete write_ticket;






		// Get VACTUAL after write =====================================================================================================================


		TMC_Serial::read_ticket* read_ticket1 = TMC2209.read(0, TMC_Serial::TSTEP);

		delay(500);	// wait for transaction

		uint32_t b = 0;

		Serial.print("\n\n FIELD:    SYNC      | D ADDRESS | R ADDRESS | DATA 3    | DATA 2    | DATA 1    | DATA 0    | CRC\n          ");
		reply = (uint8_t*)&read_ticket1->request.read_data.memory.reply;

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


		data = read_ticket1->request.read_data.memory.reply.get_data();

		Serial.print("\n                                              ");
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
		Serial.print("\n  Data:                                        DATA 0    | DATA 1    | DATA 2    | DATA 3    ");

		Serial.print("\n TSTEP: ");
		Serial.print(data);
		delete read_ticket1;


	}


	static uint32_t lastUpdate = 0;
	if (millis() - lastUpdate > 4000)
	{
		lastUpdate = millis();
		Serial.print("\n Master is active");
	}
}
