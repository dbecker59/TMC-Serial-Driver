#pragma once
#include <Arduino.h>
#include <queue>

class TMC_Serial
{
public:
	enum reg_address {
		GCONF = 0x00,
		GSTAT = 0x01,
		IFCNT = 0x02,
		SLAVECONF = 0x03,
		OTP_PROG = 0x04,
		OTP_READ = 0x05,
		IOIN = 0x06,
		FACTORT_CONF = 0x07,
		IHOLD_IRUN = 0x10,
		TPOWERDOWN = 0x11,
		TSTEP = 0x12,
		TPWMTHRS = 0x13,
		VACTUAL = 0x22,
		TCOOLTHRS = 0x14,
		SGTHRS = 0x40,
		SG_RESULT = 0x41,
		COOL_CONF = 0x42,
		MSCNT = 0x6A,
		MSCURACT = 0x6B,
		CHOPCONF = 0x6C,
		DRV_STATUS = 0x6F,
		PWMCONF = 0x70,
		PWM_SCALE = 0x71,
		PWM_AUTO = 0x72
	};


public:	// public for now, will be protected later
	Usart* serial;
	static uint8_t calc_CRC(uint8_t* datagram, uint8_t datagram_size);


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

		uint32_t get_data();
	};


	struct read_request {
		union {
			read_access_datagram transmission;
			data_transfer_datagram reply;
		} memory;

		read_request(uint32_t s_address, reg_address r_address);
	};


	struct write_request {
		data_transfer_datagram transmission;

		write_request(uint32_t s_address, reg_address r_address, uint32_t data);
	};

public:
	struct access_ticket {
		union _request {
			read_request read_data;
			write_request write_data;
			_request(uint32_t s_address, reg_address r_address);
			_request(uint32_t s_address, reg_address r_address, uint32_t data);
		} request;

		enum state
		{
			pending_reply = -1,			// Waiting for reply from driver
			pending_transmission = 0,	// Waiting to transmit
			completed_successfully = 1,	// Communication finished without error
			timedout = 2,				// Communication timedout on reply
			crc_error = 3				// Reply was corrupted
		};

		state status;								// current state of this ticket
		void(* const callback)(access_ticket*);	// constant function pointer

		access_ticket(uint32_t s_address, reg_address r_address, void(*Callback)(access_ticket*) = nullptr);
		access_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(access_ticket*) = nullptr);
	};

	struct read_ticket : public access_ticket {
		read_ticket(uint32_t s_address, reg_address r_address);
	};

	struct write_ticket : public access_ticket {
		write_ticket(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(access_ticket*) = nullptr);
	};

	TMC_Serial(Usart* _Serial, uint32_t Baudrate);
	read_ticket* read(uint32_t s_address, reg_address r_address, void(*Callback)(access_ticket*) = nullptr);
	write_ticket* write(uint32_t s_address, reg_address r_address, uint32_t data, void(*Callback)(access_ticket*) = nullptr);
};

