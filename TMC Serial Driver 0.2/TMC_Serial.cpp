#include "TMC_Serial.h"

wrap<volatile TMC_Serial::access_ticket*> TMC_Serial::messageQueues[4];
uint8_t TMC_Serial::idleTimes[4] = { -1, -1, -1, -1 };

TMC_Serial::TMC_Serial(Usart* _Serial, uint32_t Baudrate) :
	serial(_Serial),
	message_queue(messageQueues[serial - USART0])
{
	// ===== Configure GPIO pins ===============================================================================
	Pio* _pio;					// Pio Bank
	uint8_t rx_bit, tx_bit;		// pin bits
	uint8_t ab_select;			// multiplexing channel

	// Determine the values based on which USART were working with
	switch ((uint32_t)serial)
	{
	case (uint32_t)USART0:	// USART0 uses RXD0 at PA10, and TXD0 at PA11, both multiplexed on channel A
		_pio = PIOA;
		rx_bit = 10;
		tx_bit = 11;
		ab_select = 0;
		break;

	case (uint32_t)USART1:	// USART1 uses RXD1 at PA12, and TXD1 at PA13, both multiplexed on channel A
		_pio = PIOA;
		rx_bit = 12;
		tx_bit = 13;
		ab_select = 0;
		break;

	case (uint32_t)USART2:	// USART2 uses RXD2 at PB21, and TXD2 at PB20, both multiplexed on channel A
		_pio = PIOB;
		rx_bit = 21;
		tx_bit = 20;
		ab_select = 0;
		break;

	case (uint32_t)USART3:	// USART3 uses RXD3 at PD5, and TXD3 at PD4, both multiplexed on channel B
		_pio = PIOD;
		rx_bit = 5;
		tx_bit = 4;
		ab_select = 1;
		break;
	default:
		break;
	}

	_pio->PIO_PDR = (1 << rx_bit) | (1 << tx_bit);						// Grant access to the pins to the peripherals
	_pio->PIO_ABSR &= ~((1 << 10) | (1 << 11));							// Clear the current multiplexing selection
	_pio->PIO_ABSR |= (ab_select << rx_bit) | (ab_select << tx_bit);	// Set the multiplexing selection


	// Enable Peripheral Clock for USART, necessary for configuration
	REG_PMC_PCER0 = 1 << (serial - USART0 + ID_USART0);

	// ===== Configure USART peripheral ===============================================================================
	serial->US_MR =					// USART Mode Register (Configures the behavior/protocol used)
		US_MR_USART_MODE_NORMAL |	// We want normal UART communication behavior
		US_MR_USCLKS_MCK |			// Use the chips Master Clock as clock source (from PMC)
		US_MR_CHRL_8_BIT |			// Set the length of each character (byte) of data
		US_MR_PAR_NO |				// Set the parity type
		US_MR_NBSTOP_1_BIT |		// Set the number of stop bits
		US_MR_CHMODE_NORMAL;		// Normal communication operation (can configure to loopback)

	serial->US_BRGR = SystemCoreClock / Baudrate / 16;	// Configure the baudrate

	NVIC_EnableIRQ((IRQn_Type)(serial - USART0 + USART0_IRQn));		// Enable interrupts for 'serial'
}

// Function to calculate the CRC bytes
uint8_t TMC_Serial::calc_CRC(uint8_t* datagram, uint8_t datagram_size)
{
	uint8_t crc = 0;
	uint8_t byte;
	for (uint8_t i = 0; i < (datagram_size - 1); ++i)
	{
		byte = (*(uint64_t*)datagram >> (i * 8)) & 0xFF;
		for (uint8_t j = 0; j < 8; ++j)
		{
			if ((crc >> 7) ^ (byte & 0x01))
			{
				crc = (crc << 1) ^ 0x07;
			}
			else
			{
				crc = crc << 1;
			}
			byte = byte >> 1;
		}
	}
	return crc;
}


TMC_Serial::read_access_datagram::read_access_datagram(uint8_t s_address, reg_address r_address) :
	sync(0b0101),
	reserved(0x0),
	device_address(s_address),
	register_address(r_address),
	rw_access(0),
	CRC(calc_CRC((uint8_t*)this, datagram_length))
{}


TMC_Serial::data_transfer_datagram::data_transfer_datagram() :	// initiallize this to 0, will be written to by serial
	sync(0b0101),
	reserved(0x0),
	device_address(0),
	register_address(0),
	rw_access(0),
	data3(0),
	data2(0),
	data1(0),
	data0(0),
	CRC(0)
{}


TMC_Serial::data_transfer_datagram::data_transfer_datagram(uint32_t s_address, reg_address r_address, const uint32_t& data) :
	sync(0b0101),
	reserved(0x0),
	device_address(s_address),
	register_address(r_address),
	rw_access(1),
	data3(((uint8_t*)&data)[3]),
	data2(((uint8_t*)&data)[2]),
	data1(((uint8_t*)&data)[1]),
	data0(((uint8_t*)&data)[0]),
	CRC(calc_CRC((uint8_t*)this, datagram_length))
{}

uint32_t TMC_Serial::data_transfer_datagram::get_data() volatile
{
	uint8_t data[] = { data0, data1, data2, data3 };	// create a 4 byte array
	return *(uint32_t*)data;							// return the array as if its a full 32 bit integer
}


TMC_Serial::read_request::read_request(uint32_t s_address, reg_address r_address) :
	memory{ read_access_datagram(s_address, r_address) }
{}


TMC_Serial::write_request::write_request(uint32_t s_address, reg_address r_address, uint32_t data) :
	transmission(s_address, r_address, data)
{}


TMC_Serial::access_ticket::_request::_request(uint32_t s_address, reg_address r_address) :
	read_data(s_address, r_address)
{}


TMC_Serial::access_ticket::_request::_request(uint32_t s_address, reg_address r_address, uint32_t data) :
	write_data(s_address, r_address, data)
{}


TMC_Serial::access_ticket::access_ticket(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*)) :
	request(s_address, r_address),
	status(state::pending_transmission),
	callback(Callback)
{}


TMC_Serial::access_ticket::access_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*)) :
	request(s_address, r_address, data),
	status(state::pending_transmission),
	callback(Callback)
{}


TMC_Serial::read_ticket::read_ticket(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*)) :
	access_ticket(s_address, r_address, Callback)
{}


TMC_Serial::write_ticket::write_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*)) :
	access_ticket(s_address, r_address, data, Callback)
{}

volatile  TMC_Serial::read_ticket* TMC_Serial::read(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*))
{
	noInterrupts();

	volatile read_ticket* ticket = new volatile read_ticket(s_address, r_address, Callback);
	message_queue.push(ticket);
	if (message_queue.size() == 1)
		begin_transfers(serial, ticket);

	interrupts();

	return ticket;
}

volatile TMC_Serial::write_ticket* TMC_Serial::write(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*))
{
	noInterrupts();

	volatile write_ticket* ticket = new volatile write_ticket(s_address, r_address, data, Callback);
	message_queue.push(ticket);
	if (message_queue.size() == 1)
		begin_transfers(serial, ticket);

	interrupts();

	return ticket;
}

void TMC_Serial::deleteTicketCallback(volatile access_ticket* ticket)
{
	Serial.print(ticket->status);
	delete ticket;
}

void TMC_Serial::begin_transfers(Usart* serial, volatile  access_ticket* ticket)
{
	bool access_type = ticket->request.write_data.transmission.rw_access;

	if (access_type)
	{
		volatile data_transfer_datagram& transmission = ticket->request.write_data.transmission;

		serial->US_RPR = (uint32_t)&transmission;
		serial->US_RNPR = 0;
		serial->US_TPR = (uint32_t)&transmission;
		serial->US_TNPR = 0;

		serial->US_RCR = transmission.datagram_length;
		serial->US_RNCR = 0;
		serial->US_TCR = transmission.datagram_length;
		serial->US_TNCR = 0;

		serial->US_RTOR = 20; // Trigger a timeout if the start of two recieved characters excededs 20 bit-times

		serial->US_CR = US_CR_TXEN | US_CR_RXEN;
		serial->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN | US_CR_RETTO;
		serial->US_IER = US_IER_RXBUFF | US_IER_TIMEOUT;
	}
	else
	{
		volatile read_access_datagram& transmission = ticket->request.read_data.memory.transmission;
		volatile data_transfer_datagram& reply = ticket->request.read_data.memory.reply;

		serial->US_RPR = (uint32_t)&reply;
		serial->US_RNPR = (uint32_t)&reply;
		serial->US_TPR = (uint32_t)&transmission;
		serial->US_TNPR = 0;

		serial->US_RCR = transmission.datagram_length;
		serial->US_RNCR = reply.datagram_length;
		serial->US_TCR = transmission.datagram_length;
		serial->US_TNCR = 0;

		serial->US_RTOR = 20; // Trigger a timeout if the start of two recieved characters excededs 20 bit-times

		serial->US_CR = US_CR_TXEN | US_CR_RXEN | US_CR_RETTO;
		serial->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;
		serial->US_IER = US_IER_RXBUFF | US_IER_TIMEOUT;
	}
}

void USART0_Handler() {
	uint32_t status = USART0->US_CSR;

	if (status & US_CSR_RXBUFF)
	{
		wrap<volatile TMC_Serial::access_ticket*>& message_queue = TMC_Serial::messageQueues[0];
		volatile TMC_Serial::access_ticket* ticket = message_queue.pull((bool)true);

		if (ticket->request.read_data.memory.reply.CRC == TMC_Serial::calc_CRC((uint8_t*)&ticket->request.read_data.memory.reply, ticket->request.read_data.memory.reply.datagram_length))
			ticket->status = TMC_Serial::access_ticket::state::completed_successfully;
		else
			ticket->status = TMC_Serial::access_ticket::state::crc_error;

		if (ticket->callback != nullptr)	// execute the ticket's callback function if one was provided
			ticket->callback(ticket);

		USART0->US_RTOR = 0; // Trigger a timeout if the start of two recieved characters excededs 20 bit-times
		USART0->US_CR = US_CR_TXDIS | US_CR_RXDIS | US_CR_RETTO;
		USART0->US_PTCR = US_PTCR_TXTDIS | US_PTCR_RXTDIS;
		USART0->US_IDR = US_IDR_RXBUFF | US_IDR_TIMEOUT;
		if (!message_queue.empty())
			TMC_Serial::idleTimes[0] = 0;
	}
	else if (status & US_CSR_TIMEOUT)
	{
		wrap<volatile TMC_Serial::access_ticket*>& message_queue = TMC_Serial::messageQueues[0];
		volatile TMC_Serial::access_ticket* ticket = message_queue.pull((bool)true);

		ticket->status = TMC_Serial::access_ticket::state::timedout;

		if (ticket->callback != nullptr)	// execute the ticket's callback function if one was provided
			ticket->callback(ticket);

		USART0->US_RTOR = 0; // Trigger a timeout if the start of two recieved characters excededs 20 bit-times
		USART0->US_CR = US_CR_TXDIS | US_CR_RXDIS | US_CR_RETTO;
		USART0->US_PTCR = US_PTCR_TXTDIS | US_PTCR_RXTDIS;
		USART0->US_IDR = US_IDR_RXBUFF | US_IDR_TIMEOUT;
		if (!message_queue.empty())
			TMC_Serial::idleTimes[0] = 0;
	}
}


extern "C" {int sysTickHook() {
	for (size_t i = 0; i < 4; i++)
	{
		uint8_t& idle_time = TMC_Serial::idleTimes[i];
		if (idle_time == 0xFF)
			continue;	// this message queue is not idle

		wrap<volatile TMC_Serial::access_ticket*>& message_queue = TMC_Serial::messageQueues[i];
		++idle_time;	// add 1ms to the idle time

		if (idle_time > 1) {
			TMC_Serial::begin_transfers(&(USART0[i]), message_queue.pull((bool)false));
			Serial.print("\n Idle Finished");
			idle_time = 0xFF;
		}
	}
	return 0;
}}