#pragma once
#include <Arduino.h>
#include "wrap.h"


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
		//FACTORT_CONF	= 0x07,	// Factory Configuration, DON'T TOUCH!
		IHOLD_IRUN		= 0x10,	// Driver Current Control
		TPOWERDOWN		= 0x11,	// Delay time for standstill detection
		TSTEP			= 0x12,	// Measured time between microsteps
		TPWMTHRS		= 0x13,	// Lower period threshold of StealthChop
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
		const uint8_t sync : 4;
		const uint8_t reserved : 4;
		const uint8_t device_address : 8;
		const uint8_t register_address : 7;
		const uint8_t rw_access : 1;
		const uint8_t CRC : 8;

		static const uint8_t datagram_length = 4;	// All read access datagrams are 4 bytes (datasheet 15)
		read_access_datagram(uint8_t s_address, reg_address r_address);
	};


	// This is the structure of a datagram that is used to transfer data between the controller and driver.
	//    When we write to the driver, we use this datagram, and when the driver responds to read access
	//    requests, we store the reply as this struct
	struct data_transfer_datagram {
		const uint8_t sync : 4;
		const uint8_t reserved : 4;
		const uint8_t device_address : 8;
		const uint8_t register_address : 7;
		const uint8_t rw_access : 1;
		const uint8_t data3 : 8;
		const uint8_t data2 : 8;
		const uint8_t data1 : 8;
		const uint8_t data0 : 8;
		const uint8_t CRC : 8;

		static const uint8_t datagram_length = 8;	// All read reply datagrams are 8 bytes (datasheet 15)

		data_transfer_datagram();
		data_transfer_datagram(uint32_t s_address, reg_address r_address, const uint32_t& data);

		// Used to read the data field in the datagram with the bytes in the correct order, this is necessary
		//    when reading data from the driver since the receiver transmitts data bytes in reverse order.
		uint32_t get_data() volatile;
	};


	// This struct is used to handle data relating to reading from the driver, during the 'request' phase the
	//    struct can act as a read_access_datagram via the union, and during the 'reply' phase it can act as a
	//    data_transfer_datagram
	struct read_request {
		union {
			read_access_datagram transmission;
			data_transfer_datagram reply;
		} memory;

		read_request(uint32_t s_address, reg_address r_address);
	};

	
	// This struct is used to request writes to the drivers registers.
	struct write_request {
		data_transfer_datagram transmission;

		write_request(uint32_t s_address, reg_address r_address, uint32_t data);
	};


	// This is the base class for communication tickets, it can act as either a read_request, or a write_request
	//    via the untion.
	struct access_ticket {
		union _request {
			read_request read_data;
			write_request write_data;
			_request(uint32_t s_address, reg_address r_address);
			_request(uint32_t s_address, reg_address r_address, uint32_t data);
		} request;	// The datagram information

		enum state
		{
			pending_transmission = 0,	// Waiting to transmit
			completed_successfully = 1,	// Communication finished without error
			crc_error = 2,				// Reply was corrupted
			timedout = 3				// Communication timedout on reply
		};

		uint8_t status;										// current state of this ticket
		void(* const callback)(volatile access_ticket*);	// callback to be called when the ticket has completed or failed

		access_ticket(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*) = nullptr);
		access_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*) = nullptr);

		// TODO:
		//  - Add getData() function
		//  - Add checkCRC() function
	};


	// Used to handle read tickets that read registers from the driver.
	struct read_ticket : public access_ticket {
		read_ticket(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*) = nullptr);
	};

	// Used to handle tickets that write to the drivers registers
	struct write_ticket : public access_ticket {
		write_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*) = deleteTicketCallback);
	};

	static void begin_transfers(Usart* serial, volatile access_ticket* ticket);

	static void deleteTicketCallback(volatile access_ticket* ticket);

	TMC_Serial(Usart* _Serial, uint32_t Baudrate);
	
	// Read from the 's_address' driver's 'r_register' register
	//    s_address: The address of the driver to read from
	//    r_address: Address of the register to read from
	//     Callback: Function pointer to the function to execute when the ticket completes or fails
	volatile read_ticket* read(uint32_t s_address, reg_address r_address, void(*Callback)(volatile access_ticket*) = nullptr);


	// Read from the 's_address' driver's 'r_register' register
	//    s_address: The address of the driver to read from
	//    r_address: Address of the register to read from
	//         data: The data to write to the register
	//     Callback: Function pointer to the function to execute when the ticket completes or fails
	volatile write_ticket* write(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(volatile access_ticket*) = deleteTicketCallback);


	Usart* serial;											// The USART peripheral we're transmitting over
	static wrap<volatile access_ticket*> messageQueues[];	// The queues of messages to transmit over each USART
	static uint8_t idleTimes[];								// How long each queue has been idle for
	wrap<volatile access_ticket*>& message_queue;			// The message queue this instance will work with
};