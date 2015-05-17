/**
 * @file iso7816.cpp
 *
 * @author Evgeny Onopchenko
 * @version 1.0
 */


#include "iso7816.h"
#include "stm32f4xx_rcc.h"
#include <string.h>

ISO7816 Smartcard;

enum ISO7816_Constants
{
	CLK_FREQ = 3500000,				///< USART clock frequency (Hz)
	ETU = 372,						///< Elementary Time Unit (ticks) (iso/iec7816-3 8.1)
	fb_TICKS = 400,					///< Delay before reset (fb (iso/iec7816-3 6.2.2))
	fa_TICKS = 40000,				///< ATR response expectation (fa (iso/iec7816-3 6.2.2))
	ATR_MAX_LENGTH = 33,			///< ATR max length (Smartcard Tutorial Part 4)
	
	ATR_PART_TO = 100,				///< ATR timeout
	PTS_TO = 20,					///< PTS timeout
	T0_PROCBYTE_TO = 500,			///< Procedure byte timeout
	T0_SW_TO = 500,					///< SW response timeout
	T0_GETRESPONSE_TO = 500,		///< GET RESPONSE response timeout
};


/// Fi table
const uint16_t ISO7816_FI[] = 
{
	0,
	512,
	768,
	1024,
	1536,
	2048,
	0,
	0,
};


/// Di table
const uint8_t ISO7816_DI[] = 
{
	0,
	0,
	2,
	4,
	8,
	16,
	32,
	64,
};


/**
 * \brief Constructor
 * 
 */
ISO7816::ISO7816()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/// Configure VCC and RST
	RCC_AHB1PeriphClockCmd(SC_VCC_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SC_RST_GPIO_CLK, ENABLE);
	
	GPIO_ResetBits(SC_VCC_GPIO_PORT, SC_VCC_PIN);
	GPIO_ResetBits(SC_RST_GPIO_PORT, SC_RST_PIN);
	
	GPIO_InitStructure.GPIO_Pin = SC_VCC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SC_VCC_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SC_RST_PIN;
	GPIO_Init(SC_RST_GPIO_PORT, &GPIO_InitStructure);
	
	ReceiveBuffer = new RingBuffer(RxBuffer, sizeof(RxBuffer));
	
	CardActivateFlag = false;
}


/**
 * \brief Interrupts handler
 * 
 */
void ISO7816::IrqHandler()
{
	/// Transmit Data Register Empty TXE
	if(USART_GetITStatus(SC_USART, USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_TXE);
	}
	/// Received Data Ready to be Read RXNE
	if(USART_GetITStatus(SC_USART, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_RXNE);
		ReceiveBuffer->PutChar(USART_ReceiveData(SC_USART));
	}
	/// Transmission Complete TC
	if(USART_GetITStatus(SC_USART, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_TC);
	}
	/// IDLE Line Detected IDLE
	if(USART_GetITStatus(SC_USART, USART_IT_IDLE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_IDLE);
	}
	/// Noise Error NE
	if(USART_GetITStatus(SC_USART, USART_IT_NE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_NE);
	}
	/// Framing Error FE
	if(USART_GetITStatus(SC_USART, USART_IT_FE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_FE);
	}
	/// Parity Error PE
	if(USART_GetITStatus(SC_USART, USART_IT_PE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_PE);
		USART_SendData(SC_USART, DataByte);
		while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
		{
		}
	}
	/// Overrun Error ORE
	if(USART_GetITStatus(SC_USART, USART_IT_ORE) != RESET)
	{
		USART_ClearITPendingBit(SC_USART, USART_IT_ORE);
	}
}


/**
 * \brief Card activation
 * 
 */
bool ISO7816::ActivateCard()
{
	ColdReset();
	return true;
}


/**
 * \brief Card deactivation
 * 
 */
void ISO7816::DeactivateCard()
{
	RstLow();
	USART_DeInit(SC_USART);
	VccLow();
}


/**
 * \brief "Cold" reset
 * 
 */
bool ISO7816::ColdReset()
{
	/// Power up and reset low
	VccHigh();
	RstLow();
	
	RCC_ClocksTypeDef RCC_Clocks;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/// Get clock values
	RCC_GetClocksFreq(&RCC_Clocks);
	
	/// Enable GPIO AHB clock
	RCC_AHB1PeriphClockCmd(SC_USART_TX_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SC_USART_CK_GPIO_CLK, ENABLE);
	
	/// Enable USART APB2 clock
	SC_USART_APB_PERIPH_CLOCK(SC_USART_CLK, ENABLE);
	
	/// Select TX and CK alternative purpose
	GPIO_PinAFConfig(SC_USART_TX_GPIO_PORT, SC_USART_TX_SOURCE, SC_USART_CK_AF);
	GPIO_PinAFConfig(SC_USART_CK_GPIO_PORT, SC_USART_CK_SOURCE, SC_USART_CK_AF);
	
	/// TX Out Open-Drain
	GPIO_InitStructure.GPIO_Pin = SC_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SC_USART_TX_GPIO_PORT, &GPIO_InitStructure);
	
	/// CK Out Push-Pull
	GPIO_InitStructure.GPIO_Pin = SC_USART_CK_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(SC_USART_CK_GPIO_PORT, &GPIO_InitStructure);
	
	/// USART bus frequency is 84 MHz, select prescaler 24 (GTPR = 12),
	/// get frequency 84 / 24 = 3.5 MHz, calculate baudrate = 3500000 / 372 (ETU) = 9409 baud,
	/// basic baudrate is 9600 (if using frequency 3.5712 MHz), but some deviation allowed (±20%),
	/// our deviation will be 8064 / 9600 = +2%
	USART_SetPrescaler(SC_USART, (RCC_Clocks.PCLK2_Frequency / CLK_FREQ) / 2);
	
	/// Select Guard Time 16 bits
	/// (during this time, receiver can generate parity error (PF))
	USART_SetGuardTime(SC_USART, 16);
	
	/// Configure synchronous part (polarity low, phase on rising edge, last bit clocking)
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInit(SC_USART, &USART_ClockInitStructure);
	
	/// Configure asynchronous part (frame length 9 bit (8 data bits + 1 parity))
	USART_InitStructure.USART_BaudRate = CLK_FREQ / ETU;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(SC_USART, &USART_InitStructure);
	
	/// Enable USART
	USART_Cmd(SC_USART, ENABLE);
	
	/// Enable NACK transmission
	USART_SmartCardNACKCmd(SC_USART, ENABLE);
	
	/// Enable Smartcard mode
	USART_SmartCardCmd(SC_USART, ENABLE);
	
	/// Allow interrupts (receive, parity error)
	USART_ITConfig(SC_USART, USART_IT_RXNE, ENABLE);
	USART_ITConfig(SC_USART, USART_IT_PE, ENABLE);
	
	/// Allow global interrupts
	NVIC_InitStructure.NVIC_IRQChannel = SC_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/// Hold RST low during fb
	uint32_t timeout = (fb_TICKS * 1E3) / CLK_FREQ;
	Timer.Delay(timeout < 2 ? 2 : timeout);
	
	/// Set RST high level
	RstHigh();
	
	/// Wait for ATR response during fa
	bool response = false;
	Timer.Start((fa_TICKS * 1E3) / CLK_FREQ);
	while(!Timer.Match())
	{
		if(!ReceiveBuffer->Empty())
		{
			response = true;
			break;
		}
	}
	
	if(!response)
	{
		/// No response during fa
		return false;
	}
	
	ATR_t atr;
	char buffer[64];
	
	/// Expected data size
	/// (ATR have to contain at least TS and T0 fields)
	uint8_t expected = 2;
	
	bool parsed = false;
	
	while(!parsed)
	{
		int8_t count = Receive(buffer, expected, ATR_PART_TO, false);
		if(count == -1)
		{
			return false;
		}
		
		expected = 2;
		
		char* ptr = buffer;
		
		atr.TS = *ptr++;
		atr.T0 = *ptr++;
		atr.Hlength = atr.T0 & 0x0F;
		
		expected += GetBitsCount(atr.T0 & 0xF0) + atr.Hlength;
		
		if(count < expected)
		{
			continue;
		}
		
		if(atr.T0 & (1 << 4)) atr.TA1 = *ptr++;
		if(atr.T0 & (1 << 5)) atr.TB1 = *ptr++;
		if(atr.T0 & (1 << 6)) atr.TC1 = *ptr++;
		if(atr.T0 & (1 << 7)) atr.TD1 = *ptr++;
		
		expected += GetBitsCount(atr.TD1 & 0xF0);
		
		if(count < expected)
		{
			continue;
		}
		
		if(atr.TD1 & (1 << 4)) atr.TA2 = *ptr++;
		if(atr.TD1 & (1 << 5)) atr.TB2 = *ptr++;
		if(atr.TD1 & (1 << 6)) atr.TC2 = *ptr++;
		if(atr.TD1 & (1 << 7)) atr.TD2 = *ptr++;
		
		if(atr.Hlength)
		{
			memcpy(atr.H, ptr, atr.Hlength);
			ptr += atr.Hlength;
		}
		
		/// Some data remained (TCK)
		if(count > ptr - buffer)
		{
			uint8_t tck = *ptr++;
		}
		
		ReceiveBuffer->Delete(count);
		parsed = true;
	}
	
	/// If no ATR received, or unsupported convention, or unsupported protocol
	if(!parsed || (atr.TS != BIT_CONVETNTION_DIRECT) || ((atr.TD1 & 0x0F) != PROTOCOL_T0))
	{
		/// Return error
		return false;
	}
	
	/// No TA2 in ATR (4th digit in TD1)
	/// Negotiable mode
	if(!(atr.TD1 & (1 << 4)))
	{
		char buffer1[4];
		/*	PTSS	*/	buffer1[0] = 0xFF;
		/*	PTS0	*/	buffer1[1] = (1 << 4) | (atr.T0 & (1 << 7) ? (atr.TD1 & 0x0F) : 0x00);
		/*	PTS1	*/	buffer1[2] = atr.TA1;
		/*	PCK		*/	buffer1[3] = CalcCK(buffer1, 3);
		
		Transmit(buffer1, 4);
		
		char buffer2[4];
		int8_t count = Receive(buffer2, 4, PTS_TO);
		
		/// Compare transmitted and received data
		if(memcmp((char *)&buffer1, buffer2, 4))
		{
			/// If data is different, return error
			return false;
		}
		
		/// Get Fi and Di parameters
		uint16_t fi = ISO7816_FI[(atr.TA1 >> 4) & 0x07];
		uint8_t di = ISO7816_DI[atr.TA1 & 0x07];
		
		/// Calculate and set new baudrate
		USART_InitStructure.USART_BaudRate = CLK_FREQ / (fi / di);
		USART_Init(SC_USART, &USART_InitStructure);
		
//		Timer.Delay(400);
	}
	/// TA2 present in ATR
	/// Specific mode
	else
	{
		/// Not supported
		return false;
	}
	
	CardActivateFlag = true;
	
	return true;
}


/**
 * \brief Send TDPU
 * 
 */
uint16_t ISO7816::SendTPDU(const TPDU_t* tpdu, const void* data, uint8_t dataSize, Case_t exchangeCase)
{
	/// Check card state
	if(!CardActivateFlag)
	{
		return 0;
	}
	
	/// Choose case, assemble frame and transmit
	uint16_t result = 0;
	switch(exchangeCase)
	{
		case CASE_1:
		case CASE_2:
		{
			Transmit((const char *)tpdu, sizeof(*tpdu));
			Transmit((const char *)&dataSize, 1);
			
			/// Get status word
			uint8_t sw[2];
			if(Receive((char *)sw, 2, T0_SW_TO) == -1)
			{
				return 0;
			}
			
			result = (int16_t)((sw[0] << 8) | sw[1]);
			break;
		}
		
		case CASE_3:
		case CASE_4:
		{
			Transmit((const char *)tpdu, sizeof(*tpdu));
			Transmit((const char *)&dataSize, 1);
			
			/// Get procedure byte
			uint8_t pb = 0;
			int8_t count = Receive((char *)&pb, 1, T0_PROCBYTE_TO);
			if(count == -1)
			{
				return 0;
			}
			
			/// Wrong procedure byte
			if(pb != tpdu->INS)
			{
				/// Return error
				return 0;
			}
			
			Transmit((const char *)data, dataSize);
			
			/// Get status word
			uint8_t sw[2];
			if(Receive((char *)sw, 2, T0_SW_TO) == -1)
			{
				return 0;
			}
			
			result = (int16_t)((sw[0] << 8) | sw[1]);
			break;
		}
		
		default:
		{
			return 0;
		}
	}
	
	return result;
}


/**
 * \brief Select file
 * 
 */
bool ISO7816::SelectFile(uint8_t P1, uint8_t P2, const char* fileName, uint8_t nameLength)
{
	TPDU_t tpdu;
	
	tpdu.CLA = 0x00;
	tpdu.INS = SELECT_FILE;
	tpdu.P1 = P1;
	tpdu.P2 = P2;
	
	uint16_t result = SendTPDU(&tpdu, fileName, nameLength, CASE_3);
	
	if(result != SW_NO_ERROR)
	{
		return false;
	}
	
	return true;
}


/**
 * \brief Verify
 * 
 */
bool ISO7816::Verify(uint8_t P1, uint8_t P2, const char* pin, uint8_t pinLength)
{
	TPDU_t tpdu;
	tpdu.CLA = 0x00;
	tpdu.INS = VERIFY;
	tpdu.P1 = P1;
	tpdu.P2 = P2;
	
	uint16_t result = SendTPDU(&tpdu, pin, pinLength, CASE_3);
	
	if(result != SW_NO_ERROR)
	{
		return false;
	}
	
	return true;
}


/**
 * \brief Get response
 * 
 */
int ISO7816::GetResponse(void* buffer, uint8_t dataSize)
{
	TPDU_t tpdu;
	tpdu.CLA = 0x00;
	tpdu.INS = GET_RESPONSE;
	tpdu.P1 = 0x00;
	tpdu.P2 = 0x00;
	
	Transmit((const char *)&tpdu, sizeof(tpdu));
	Transmit((const char *)&dataSize, 1);
	
	/// Get procedure byte
	uint8_t pb = 0;
	int8_t count = Receive((char *)&pb, 1, T0_PROCBYTE_TO);
	if(count == -1)
	{
		return -1;
	}
	
	/// Wrong procedure byte
	if(pb != tpdu.INS)
	{
		/// Return error
		return -1;
	}
	
	int size = Receive((char *)buffer, dataSize, T0_GETRESPONSE_TO);
	if(count == -1)
	{
		return -1;
	}
	
	/// Get status word
	uint8_t sw[2];
	if(Receive((char *)sw, 2, T0_SW_TO) == -1)
	{
		return -1;
	}
	
	uint16_t result = (int16_t)((sw[0] << 8) | sw[1]);
	
	if(result != SW_NO_ERROR)
	{
		return -1;
	}
	
	return size;
}


/**
 * \brief Transmit data
 * 
 */
int8_t ISO7816::Transmit(const char* data, uint8_t size)
{
	/// Put data to DR register
	for(uint8_t index = 0; index < size; index++)
	{
		/// Remember last transmitted byte (if parity error will occur)
		DataByte = data[index];
		USART_SendData(SC_USART, DataByte);
		while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
		{
		}
	}
	
	/// Remove transmitted data from buffer
	while(ReceiveBuffer->GetByteCount() < size)
	{
	}
	ReceiveBuffer->Delete(size);
	
	return size;
}


/**
 * \brief Receive data
 * 
 */
int8_t ISO7816::Receive(char* buffer, uint8_t size, uint32_t timeout, bool remove)
{
	SoftTimer timer;
	timer.Start(timeout);
	
	while(ReceiveBuffer->GetByteCount() < size)
	{
		if(timer.Match())
		{
			return -1;
		}
	}
	
	if(remove)
	{
		ReceiveBuffer->Get(buffer, size);
	}
	else
	{
		ReceiveBuffer->Copy(buffer, size);
	}
	
	return size;
}


/**
 * \brief Calculate set bit count
 * 
 */
uint8_t ISO7816::GetBitsCount(uint8_t byte)
{
	byte = byte - ((byte >> 1) & 0x55);
	byte = (byte & 0x33) + ((byte >> 2) & 0x33);
	
	return((byte + (byte >> 4)) & 0x0F);
}


/**
 * \brief Calculate TCK and PCK
 * 
 */
uint8_t ISO7816::CalcCK(char* data, uint8_t length)
{
	char* ptr = data;
	uint8_t ck = *ptr++;
	
	while(ptr - data < length)
	{
		ck ^= *ptr++;
	}
	
	return ck;
}


/**
 * \brief Interrupts handler
 * 
 */
extern "C" void USART6_IRQHandler()
{
	Smartcard.IrqHandler();
}
