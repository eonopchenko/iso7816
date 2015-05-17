// ----------------------------------------------------------------------------

/**
   File: iso7816.h
   
   System:
   Component Name: ISO7816 driver module
   Status:         Version 1.0 Release 1 

   Language: C++

   Address:
	St.Petersburg, Russia
	  
   Author: Evgeny Onopchenko
   
   E-Mail: jake.spb@gmail.com
   
   Description: Header file for ISO7816 smartcard driver implementation
                This file contains the defined types

*/ 
// ----------------------------------------------------------------------------

#ifndef __ISO7816_H
#define __ISO7816_H

#include "timer.h"
#include "ring_buffer.h"
#include <stdint.h>
#include <stm32f4xx.h>


#define SC_USART					USART6
#define SC_USART_CLK				RCC_APB2Periph_USART6
#define SC_USART_APB_PERIPH_CLOCK	RCC_APB2PeriphClockCmd
#define SC_USART_IRQn				USART6_IRQn
#define SC_USART_IRQHandler			USART6_IRQHandler

#define SC_USART_TX_PIN				GPIO_Pin_6
#define SC_USART_TX_GPIO_PORT		GPIOC
#define SC_USART_TX_GPIO_CLK		RCC_AHB1Periph_GPIOC
#define SC_USART_TX_SOURCE			GPIO_PinSource6
#define SC_USART_TX_AF				GPIO_AF_USART6

#define SC_USART_CK_PIN				GPIO_Pin_8
#define SC_USART_CK_GPIO_PORT		GPIOC
#define SC_USART_CK_GPIO_CLK		RCC_AHB1Periph_GPIOC
#define SC_USART_CK_SOURCE			GPIO_PinSource8
#define SC_USART_CK_AF				GPIO_AF_USART6

#define SC_VCC_PIN					GPIO_Pin_5
#define SC_VCC_GPIO_PORT			GPIOG
#define SC_VCC_GPIO_CLK				RCC_AHB1Periph_GPIOG

#define SC_RST_PIN					GPIO_Pin_7
#define SC_RST_GPIO_PORT			GPIOC
#define SC_RST_GPIO_CLK				RCC_AHB1Periph_GPIOC


class ISO7816
{
	public:
		/// Bit convention
		enum BitConvention
		{
			BIT_CONVETNTION_DIRECT = 0x3B	///< Direct convention
		};
		
		/// Protocol (physical layer)
		enum Protocol_t
		{
			///< T0 protocol (asynchronous, half-duplex, character)
			PROTOCOL_T0 = 0x00
		};
		
		/// Error codes
		enum SW_t
		{
			SW_NO_ERROR  = 0x9000,
			SW_BYTES_REMAINING_00 = 0x6100,
		};
		
#pragma pack(1)
		/// ATR (physical layer)
		struct ATR_t
		{
			uint8_t TS;
			uint8_t T0;
			uint8_t TA1;
			uint8_t TB1;
			uint8_t TC1;
			uint8_t TD1;
			uint8_t TA2;
			uint8_t TB2;
			uint8_t TC2;
			uint8_t TD2;
			uint8_t Hlength;
			uint8_t H[20];
		};
		
		/// PTS (physical layer)
		struct PTS_t
		{
			uint8_t PTSS;
			uint8_t PTS0;
			uint8_t PTS1;
			uint8_t PTS2;
			uint8_t PTS3;
			uint8_t PCK;
		};
		
		/// TPDU (link layer) (P3 will be transmit separated)
		struct TPDU_t
		{
			uint8_t CLA;
			uint8_t INS;
			uint8_t P1;
			uint8_t P2;
		};
#pragma pack()
		
		/// Exchange cases
		enum Case_t
		{
			CASE_1,	///< No data in command, no data in response
			CASE_2,	///< No data in command, data in response
			CASE_3,	///< Data in command, no data in response
			CASE_4,	///< Data in command, data in response
			
		};
		
		/// Instructions list
		enum INS_t
		{
			VERIFY					= 0x20,
			SELECT_FILE				= 0xA4,
			GET_RESPONSE			= 0xC0,
			TERMINATE_CARD_USAGE	= 0xFE,
			HANG_CARD				= 0xFF,
		};
		
		/// Card activation
		bool ActivateCard();
		
		/// "Cold" reset
		bool ColdReset();
		
		/// Card deactivation
		void DeactivateCard();
		
		/// Select file
		bool SelectFile(uint8_t P1, uint8_t P2, const char* fileName, uint8_t nameLength);
		
		/// Verify
		bool Verify(uint8_t P1, uint8_t P2, const char* pin, uint8_t pinLength);
		
		/// Send TPDU
		uint16_t SendTPDU(const TPDU_t* tpdu, const void* data, uint8_t dataSize, Case_t exchangeCase);
		
		/// Get data
		int GetResponse(void* buffer, uint8_t dataSize);
		
		/// Interrupts handler
		void IrqHandler();
		
		/// Constructor
		ISO7816();
		
	private:
		SoftTimer Timer;					///< Timer
		
		char RxBuffer[128];					///< Receive buffer
		RingBuffer* ReceiveBuffer;			///< Ring buffer
		
		uint8_t DataByte;					///< Last transmitted byte
		
		bool CardActivateFlag;				///< Card activation flag
		
		/// VCC on/off
		void VccHigh()	{ GPIO_ResetBits(SC_VCC_GPIO_PORT, SC_VCC_PIN); };
		void VccLow()	{ GPIO_SetBits(SC_VCC_GPIO_PORT, SC_VCC_PIN); };
		
		/// RST high/low
		void RstHigh()	{ GPIO_SetBits(SC_RST_GPIO_PORT, SC_RST_PIN); };
		void RstLow()	{ GPIO_ResetBits(SC_RST_GPIO_PORT, SC_RST_PIN); };
		
		/// Transmit data
		int8_t Transmit(const char* data, uint8_t size);
		
		/// Receive data
		int8_t Receive(char* buffer, uint8_t size, uint32_t timeout, bool remove = true);
		
		/// Calculate set bit count
		uint8_t GetBitsCount(uint8_t byte);
		
		/// Calculate TCK and PCK
		uint8_t CalcCK(char* data, uint8_t length);
};

extern ISO7816 Smartcard;

#endif
