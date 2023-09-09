#pragma once
#include <Arduino.h>
#include "Ring_Buffer.h"

class TMC_Serial
{
public:
	// The TMC2209 register addresses
	enum reg_address {
		GCONF			= 0x00,	// Global Configuration Flags
		GSTAT			= 0x01,	// Global Status Flags
		IFCNT			= 0x02,	// Interface Transmission Counter
		SLAVECONF		= 0x03,	// SENDDELAY
		OTP_PROG		= 0x04,	// One-Time-Programming Programming
		OTP_READ		= 0x05,	// One-Time-Programming Read
		IOIN			= 0x06,	// Input Output Pin States
		// FACTORT_CONF	= 0x07,	// Factory Configuration, DON'T TOUCH!
		IHOLD_IRUN		= 0x10,	// Driver Current Control
		TPOWERDOWN		= 0x11,	// Delay time for standstill detection
		TSTEP			= 0x12,	// Measured time between microsteps
		TPWMTHRS		= 0x13,	// Minimum period threshold of StealthChop
		VACTUAL			= 0x22,	// Set velocity of motor, µstep/s
		TCOOLTHRS		= 0x14,	// Upper period threshold of CoolStep and StallGuard
		SGTHRS			= 0x40,	// Detection threshold for a stall
		SG_RESULT		= 0x41,	// StallGuard Result
		COOL_CONF		= 0x42,	// CoolStep Configuration
		MSCNT			= 0x6A,	// Microstep Table Counter
		MSCURACT		= 0x6B,	// Microstep Phase Currents for A and B
		CHOPCONF		= 0x6C,	// Chopper and driver configuration
		DRV_STATUS		= 0x6F,	// Driver Status flags
		PWMCONF			= 0x70,	// Stealthchop PWM chopper configuration
		PWM_SCALE		= 0x71,	// Results of StealthChop Amplitute regulator
		PWM_AUTO		= 0x72	// PWM offset and gradient values
	};



	// used to calculate the CRC byte for datagrams
	//    datagram: pointer to the first byte of the datagram
	//    datagram_size: length in bytes of the datagram
	static uint8_t calc_CRC(uint8_t* datagram, uint8_t datagram_size);
	


	// This is the structure of a datagram that requests a read access from the drivers registers
	struct read_access_datagram {
		const uint8_t sync : 4;					// Sequence of bits (1010) which allows the TMC2209 to synchronize its baud rate.
		const uint8_t reserved : 4;				// Reserved: The TMC2209 does nothing with these bits, but they are included in the CRC calculation
		const uint8_t device_address : 8;		// The slave address of the driver we're communicating with (selected using the ms1, ms2 pins)
		const uint8_t register_address : 7;		// The address of the register we're writing to/reading from
		const uint8_t rw_access : 1;			// The read/write access bit to signal to the TMC2209 whether we want to write to or read from a register
		const uint8_t CRC : 8;					// This is the CRC byte used to validate each datagram

		static const uint8_t datagram_length = 4;	// All read access datagrams are 4 bytes (datasheet 15)
		read_access_datagram(uint8_t s_address, uint32_t r_address);
	};



	// This is the structure of a datagram that is used to transfer data between the controller and driver.
	//    When we write to the driver, we use this datagram, and when the driver responds to read access
	//    requests, we store the data_transfer as this struct
	struct data_transfer_datagram {
		const uint8_t sync : 4;					// Sequence of bits (1010) which allows the TMC2209 to synchronize its baud rate.
		const uint8_t reserved : 4;				// Reserved: The TMC2209 does nothing with these bits, but they are included in the CRC calculation
		const uint8_t device_address : 8;		// The slave address of the driver we're communicating with (selected using the ms1, ms2 pins)
		const uint8_t register_address : 7;		// The address of the register we're writing to/reading from
		const uint8_t rw_access : 1;			// The read/write access bit to signal to the TMC2209 whether we want to write to or read from a register
		const uint8_t data3 : 8;				// The TMC2209 reqiures the data bytes are sent in reverse order
		const uint8_t data2 : 8;				//    Thus, we send the 4th byte first, then the 3rd, 2nd, and lastly
		const uint8_t data1 : 8;				//    the first.
		const uint8_t data0 : 8;				//
		const uint8_t CRC : 8;					// This is the CRC byte used to validate each datagram

		static const uint8_t datagram_length = 8;	// All read data_transfer datagrams are 8 bytes (datasheet 15)

		data_transfer_datagram();
		data_transfer_datagram(uint32_t s_address, uint32_t r_address, const uint32_t& data);
	};


	// This is the base class for communication tickets, it can act as either a read_request, or a write_request
	//    via the untion.
	struct access_ticket {
		union _request {
			read_access_datagram read_request;
			data_transfer_datagram data_transfer;
			_request(uint32_t s_address, uint32_t r_address);
			_request(uint32_t s_address, uint32_t r_address, uint32_t data);
		} datagram;	// The datagram information

		enum state
		{
			pending = 0,	// Waiting to transmit
			completed_successfully = 1,	// Communication finished without error
			crc_error = 2,				// Reply was corrupted
			timedout = 3				// Communication timedout on data_transfer
		};

		uint8_t status;																	// current state of this ticket
		void(* const callback)(volatile access_ticket*, void* additional_parameters);	// callback to be called when the ticket has completed or failed
		void* callback_parameters;

		access_ticket(uint32_t s_address, uint32_t r_address, void(*Callback)(volatile access_ticket*, void*), void* Callback_parameters);
		access_ticket(uint32_t s_address, uint32_t r_address, uint32_t data, void(*Callback)(volatile access_ticket*, void*), void* Callback_parameters);

		bool transfer_complete() const volatile;
		bool validate_crc() const volatile;
		uint32_t get_data() volatile const;
	};


	// Used to handle read tickets that read registers from the driver.
	struct read_ticket : public access_ticket {
		read_ticket(uint32_t s_address, uint32_t r_address, void(*Callback)(volatile access_ticket*, void* additional_parameters), void* Callback_parameters);
	};

	// Used to handle tickets that write to the drivers registers
	struct write_ticket : public access_ticket {
		write_ticket(uint32_t s_address, uint32_t r_address, uint32_t data, void(*Callback)(volatile access_ticket*, void* additional_parameters), void* Callback_parameters);
	};

	static void begin_transfers(Usart* serial, volatile access_ticket* ticket);

	static void deleteTicketCallback(volatile access_ticket* ticket, void*);
	static void storeRegisterAt(volatile access_ticket* ticket, void*);

	TMC_Serial(Usart* _Serial, uint32_t Baudrate);
	
	// Read from the 's_address' driver's 'r_register' register
	//    s_address: The slave address of the driver to read from
	//    r_address: Address of the register to read from
	//     Callback: A callback function that'll be called whenever the transfer has compeleted
	volatile read_ticket* read(uint32_t s_address, uint32_t r_address, void(*Callback)(volatile access_ticket*, void* additional_parameters) = nullptr, void* Callback_parameters = nullptr);


	// Read from the 's_address' driver's 'r_register' register
	//	s_address: The slave address of the driver to read from
	//	r_address: Address of the register to read from
	//	data: The data to write to the register
	//	Callback: A callback function that'll be called whenever the transfer has compeleted
	//	Callback_parameters: A pointer to the parameters the callback function will utilize
	volatile write_ticket* write(uint32_t s_address, uint32_t r_address, uint32_t data, void(*Callback)(volatile access_ticket*, void* additional_parameters) = deleteTicketCallback, void* Callback_parameters = nullptr);


protected:
	Usart* serial;											// The USART peripheral we're transmitting over
	static Ring_Buffer<volatile access_ticket*> messageQueues[];	// The queues of messages to transmit over each USART
	static uint8_t idleTimes[];								// How long each queue has been idle for
	Ring_Buffer<volatile access_ticket*>& message_queue;			// The message queue this instance will work with

	friend void USART_Handler(Usart* serial, uint32_t status);
	friend void USART0_Handler();
	friend void USART1_Handler();
	friend void USART2_Handler();
	friend void USART3_Handler();
	friend void message_queue_idle_handler();
};