/*
SoftwareSerialWithHalfDuplex.cpp (formerly SoftwareSerial.cpp) -
Multi-instance software serial with half duplex library for Arduino/Wiring

By default the library works the same as the SoftwareSerial library, but by adding a couple of additional arguments
it can be configured for half-duplex. In that case, the transmit pin is set by default to an input, with the pull-up set.
When transmitting, the pin temporarily switches to an output until the byte is sent, then flips back to input. When a module is
receiving it should not be able to transmit, and vice-versa. This library probably won't work as is if you need inverted-logic.

This is a first draft of the library and test programs. It appears to work, but has only been tested on a limited basis.
The library also works to communicate with Robotis Bioloid AX-12 motors.
Seems fairly reliable up to 57600 baud. As with all serial neither error checking, nor addressing are implemented,
so it is likely that you will need to do this yourself. Also, you can make use of other protocols such as i2c.
I am looking for any feedback, advice and help at this stage.
Changes from SoftwareSerial have been noted with a comment of "//NS" for your review. Only a few were required.
Contact me at n.stedman@steddyrobots.com, or on the arduino forum.
----
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) -
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
	 (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
	 multi-instance support, porting to 8MHz processors,
	 various optimizations, PROGMEM delay tables, inverse logic and
	 direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerialWithHalfDuplex.h>
#include <util/delay_basic.h>
#include <math.h>

//Lookup table taken from CustomSoftwareSerial
//
// Lookup table
//
typedef struct _DELAY_TABLE
{
	long baud;
	unsigned short rx_delay_centering;
	unsigned short rx_delay_intrabit;
	unsigned short rx_delay_stopbit;
	unsigned short tx_delay;
} DELAY_TABLE;

//#if F_CPU == 16000000
//
//static const DELAY_TABLE PROGMEM table[] =
//{
//	//  baud    rxcenter   rxintra    rxstop    tx
//	{ 115200,   1,         17,        17,       12,    },
//	{ 57600,    10,        37,        37,       33,    },
//	{ 38400,    25,        57,        57,       54,    },
//	{ 31250,    31,        70,        70,       68,    },
//	{ 28800,    34,        77,        77,       74,    },
//	{ 19200,    54,        117,       117,      114,   },
//	{ 14400,    74,        156,       156,      153,   },
//	{ 9600,     114,       236,       236,      233,   },
//	{ 4800,     233,       474,       474,      471,   },
//	{ 2400,     471,       950,       950,      947,   },
//	{ 1200,     947,       1902,      1902,     1899,  },
//	{ 600,      1902,      3804,      3804,     3800,  },
//	{ 300,      3804,      7617,      7617,     7614,  },
//};
//
//const int XMIT_START_ADJUSTMENT = 5;
//
//#elif F_CPU == 8000000
//
//static const DELAY_TABLE table[] PROGMEM =
//{
//	//  baud    rxcenter    rxintra    rxstop  tx
//	{ 115200,   1,          5,         5,      3,      },
//	{ 57600,    1,          15,        15,     13,     },
//	{ 38400,    2,          25,        26,     23,     },
//	{ 31250,    7,          32,        33,     29,     },
//	{ 28800,    11,         35,        35,     32,     },
//	{ 19200,    20,         55,        55,     52,     },
//	{ 14400,    30,         75,        75,     72,     },
//	{ 9600,     50,         114,       114,    112,    },
//	{ 4800,     110,        233,       233,    230,    },
//	{ 2400,     229,        472,       472,    469,    },
//	{ 1200,     467,        948,       948,    945,    },
//	{ 600,      948,        1895,      1895,   1890,   },
//	{ 300,      1895,       3805,      3805,   3802,   },
//};
//
//const int XMIT_START_ADJUSTMENT = 4;
//
//#elif F_CPU == 20000000
//
//// 20MHz support courtesy of the good people at macegr.com.
//// Thanks, Garrett!
//
//static const DELAY_TABLE PROGMEM table[] =
//{
//	//  baud    rxcenter    rxintra    rxstop  tx
//	{ 115200,   3,          21,        21,     18,     },
//	{ 57600,    20,         43,        43,     41,     },
//	{ 38400,    37,         73,        73,     70,     },
//	{ 31250,    45,         89,        89,     88,     },
//	{ 28800,    46,         98,        98,     95,     },
//	{ 19200,    71,         148,       148,    145,    },
//	{ 14400,    96,         197,       197,    194,    },
//	{ 9600,     146,        297,       297,    294,    },
//	{ 4800,     296,        595,       595,    592,    },
//	{ 2400,     592,        1189,      1189,   1186,   },
//	{ 1200,     1187,       2379,      2379,   2376,   },
//	{ 600,      2379,       4759,      4759,   4755,   },
//	{ 300,      4759,       9523,      9523,   9520,   },
//};
//
//const int XMIT_START_ADJUSTMENT = 6;
//
//#else
//
//#error This version of CustomSoftwareSerial supports only 20, 16 and 8MHz processors
//
//#endif
//
// Statics
//
SoftwareSerialWithHalfDuplex* SoftwareSerialWithHalfDuplex::active_object = 0;
char SoftwareSerialWithHalfDuplex::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t SoftwareSerialWithHalfDuplex::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerialWithHalfDuplex::_receive_buffer_head = 0;


//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DEBUG 
inline void DebugPulse(uint8_t pin, uint8_t count)
{
	volatile uint8_t* pport = portOutputRegister(digitalPinToPort(pin));

	uint8_t val = *pport;
	while (count--)
	{
		*pport = val | digitalPinToBitMask(pin);
		*pport = val;
	}
}
#endif
//
// Private methods
//

/* static */
inline void SoftwareSerialWithHalfDuplex::tunedDelay(uint16_t delay) {
	_delay_loop_2(delay);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerialWithHalfDuplex::listen()
{
	if (!_rx_delay_stopbit)
	{
		//DEBUG("SoftSerial: Listening not started(!_rx_delay_stopbit):false");
		setError(sswhdErrors::sswhdRXDelayStopBitNotSet);
		return false;
	}
	if (active_object != this)
	{
		//    DEBUG("SoftSerial: Listening started");

		if (active_object)
		{
			active_object->stopListening();
		}
		_buffer_overflow = false;
		_receive_buffer_head = _receive_buffer_tail = 0;
		active_object = this;
		//DEBUG("SoftSerial: Listening started:true");

		setRxIntMsk(true);

		return true;
	}
	//  DEBUG("SoftSerial: Listening started:false");
	return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerialWithHalfDuplex::stopListening()
{
	if (active_object == this)
	{
		setRxIntMsk(false);
		active_object = NULL;
		//DEBUG("SoftSerial: Listening stopped:true");

		return true;
	}
	//DEBUG("SoftSerial: Listening stopped:false");
	return false;
}

//
// The receive routine called by the interrupt handler
//
void SoftwareSerialWithHalfDuplex::recv()
{

#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Preserve the registers that the compiler misses
	// (courtesy of Arduino forum user *etracer*)
	asm volatile(
		"push r18 \n\t"
		"push r19 \n\t"
		"push r20 \n\t"
		"push r21 \n\t"
		"push r22 \n\t"
		"push r23 \n\t"
		"push r26 \n\t"
		"push r27 \n\t"
		::);
#endif  

	uint8_t d = 0;

	// If RX line is high, then we don't see any start bit
	// so interrupt is probably not for us
	if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
	{
#if DEBUGRECV //start of receiving
		digitalWrite(DEBUGRW_PIN, HIGH);
#endif
		//DEBUG("SoftSerial: Data Received");
		// Disable further interrupts during reception, this prevents
		// triggering another interrupt directly after we return, which can
		// cause problems at higher baudrates.
		setRxIntMsk(false);
		// Wait approximately 1/2 of a bit width to "center" the sample
		tunedDelay(_rx_delay_centering);
#if _DEBUG 
		DebugPulse(_DEBUG_PIN2, 1);
#endif
		// Read each of the 8 bits
		/*for (uint8_t i = 8; i > 0; --i)
		{
			tunedDelay(_rx_delay_intrabit);
			d >>= 1;
#ifdef _DEBUG 
			DebugPulse(_DEBUG_PIN2, 1);
#endif
			if (rx_pin_read())
				d |= 0x80;
		}*/
		//start taken from CustomSoftwareSerial
		for (uint8_t i = 0x1; i && i <= this->_maxValueOfDataBit; i <<= 1)
		{
			tunedDelay(_rx_delay_intrabit);
#ifdef _DEBUG 
			DebugPulse(_DEBUG_PIN2, 1);
#endif			
			uint8_t noti = ~i;
			if (rx_pin_read()) 
			{
				d |= i;
			}
			else 
			{ // else clause added to ensure function timing is ~balanced				
				d &= noti;
			}
		}

		// skip the parity bit
		if (this->_parityBit != NONE) 
		{
			tunedDelay(_rx_delay_stopbit);
#ifdef _DEBUG 
			DebugPulse(_DEBUG_PIN2, 1);
#endif
		}
		
		// skip the stop bit
		for (uint8_t i = 0; i < this->_numberOfStopBit; i++) 
		{
			tunedDelay(_rx_delay_stopbit);
#ifdef _DEBUG 
			DebugPulse(_DEBUG_PIN2, 1);
#endif
		}
		//end taken from CustomSoftwareSerial

		if (_inverse_logic)
			d = ~d;

		// if buffer full, set the overflow flag and return
		uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
		if (next != _receive_buffer_head)
		{
			// save new data in buffer: tail points to where byte goes
			_receive_buffer[_receive_buffer_tail] = d; // save new byte
			_receive_buffer_tail = next;
		}
		else
		{
#if _DEBUG // for scope: pulse pin as overflow indictator
			DebugPulse(_DEBUG_PIN1, 1);
#endif
			_buffer_overflow = true;
		}
#if DEBUGRECV //end of receiving
		digitalWrite(DEBUGRW_PIN, LOW);
#endif

		// original version - now skip stop bits are above (skip the stop bit)
		//commented according to CustomSoftwareSerial logic of parity, stop bits, tunedDelay(_rx_delay_stopbit);
#if _DEBUG 
		DebugPulse(_DEBUG_PIN1, 1);
#endif

		// Re-enable interrupts when we're sure to be inside the stop bit
		setRxIntMsk(true);
	}

#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Restore the registers that the compiler misses
	asm volatile(
		"pop r27 \n\t"
		"pop r26 \n\t"
		"pop r23 \n\t"
		"pop r22 \n\t"
		"pop r21 \n\t"
		"pop r20 \n\t"
		"pop r19 \n\t"
		"pop r18 \n\t"
		::);
#endif
}

void SoftwareSerialWithHalfDuplex::tx_pin_write(uint8_t pin_state)
{
	if (pin_state == LOW)
		*_transmitPortRegister &= ~_transmitBitMask;
	else
		*_transmitPortRegister |= _transmitBitMask;
}

uint8_t SoftwareSerialWithHalfDuplex::rx_pin_read()
{
	return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerialWithHalfDuplex::handle_interrupt()
{
	//DEBUG("SoftSerial: handle_interrupt, 1");
	if (active_object)
	{
		active_object->recv();
		//DEBUG("SoftSerial: handle_interrupt, 2");
	}
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
	SoftwareSerialWithHalfDuplex::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif
//#if defined(PCINT4_vect)
//ISR(PCINT4_vect, ISR_ALIASOF(PCINT0_vect));
//#endif
//#if defined(PCINT5_vect)
//ISR(PCINT5_vect, ISR_ALIASOF(PCINT0_vect));
//#endif
//#if defined(PCINT6_vect)
//ISR(PCINT6_vect, ISR_ALIASOF(PCINT0_vect));
//#endif
//#if defined(PCINT7_vect)
//ISR(PCINT7_vect, ISR_ALIASOF(PCINT0_vect));
//#endif
//
// Constructor
//
SoftwareSerialWithHalfDuplex::SoftwareSerialWithHalfDuplex(uint8_t receivePin, uint8_t transmitPin, 
	bool inverse_logic  /*= false*/, bool full_duplex /* = true */) :
	_rx_delay_centering(0),
	_rx_delay_intrabit(0),
	_rx_delay_stopbit(0),
	_tx_delay(0),
	_buffer_overflow(false),
	_inverse_logic(inverse_logic)
{
	// @micooke - passing half_duplex is fairly pointless as you can determine it from the tx and rx pin chose.
	// Im inclined to remove full_duplex as an argument.
	// This change allows the user to test half-duplex with different or the same pins
	_full_duplex = (transmitPin == receivePin) ? false : full_duplex;				//NS Added 
	
	setTX(transmitPin);
	setRX(receivePin);
	//taken from CustomSoftwareSerial
	setPort(CSERIAL_8N1);
	DEBUG("SoftwareSerial: Created with ReceivePin: " + (String)receivePin);
}

//
// Destructor
//
SoftwareSerialWithHalfDuplex::~SoftwareSerialWithHalfDuplex()
{
	end();
}

void SoftwareSerialWithHalfDuplex::setTX(uint8_t tx)
{
	// First write, then set output. If we do this the other way around,
	// the pin would be output low for a short while before switching to
	// output high. Now, it is input with pullup for a short while, which
	// is fine. With inverse logic, either order is fine.
	digitalWrite(tx, _inverse_logic ? LOW : HIGH);
	if (_full_duplex)
		pinMode(tx, OUTPUT);					//NS Added
	else
		pinMode(tx, INPUT);									//NS Added
	_transmitPin = tx;  										//NS Added  

	_transmitBitMask = digitalPinToBitMask(tx);
	uint8_t port = digitalPinToPort(tx);
	_transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerialWithHalfDuplex::setRX(uint8_t rx)
{
	pinMode(rx, INPUT);
	if (!_inverse_logic)
		digitalWrite(rx, HIGH);  // pullup for normal logic!
	_receivePin = rx;
	_receiveBitMask = digitalPinToBitMask(rx);
	uint8_t port = digitalPinToPort(rx);
	_receivePortRegister = portInputRegister(port);
}

uint16_t SoftwareSerialWithHalfDuplex::subtract_cap(uint16_t num, uint16_t sub) {
	if (num > sub)
		return num - sub;
	else
		return 1;
}

//
// Public methods
//

void SoftwareSerialWithHalfDuplex::begin(long speed)
{
	_rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;
	/*bool foundInLookupTable = false;
	for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); ++i)
	{
		long baud = pgm_read_dword(&table[i].baud);
		if (baud == speed)
		{
			_rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
			_rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
			_rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
			_tx_delay = pgm_read_word(&table[i].tx_delay);
			foundInLookupTable = true;
			break;
		}
	}*/
	//if we haven't found the baud rate from table - using code from SoftSerWithHalfDuplex, else - from CustomSoftSer
	//if (!foundInLookupTable)
	//{ //->code from SoftSerWithHalfDuplex
		// Precalculate the various delays, in number of 4-cycle delays
		uint16_t bit_delay = (F_CPU / speed) / 4;

		// 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
		// 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
		// 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
		// These are all close enough to just use 15 cycles, since the inter-bit
		// timings are the most critical (deviations stack 8 times)
		_tx_delay = subtract_cap(bit_delay, 15 / 4);
	
	
	// Only setup rx when we have a valid PCINT for this pin
		if (digitalPinToPCICR(_receivePin))
		{
#if GCC_VERSION > 40800
			// Timings counted from gcc 4.8.2 output. This works up to 115200 on
			// 16Mhz and 57600 on 8Mhz.
			//
			// When the start bit occurs, there are 3 or 4 cycles before the
			// interrupt flag is set, 4 cycles before the PC is set to the right
			// interrupt vector address and the old PC is pushed on the stack,
			// and then 75 cycles of instructions (including the RJMP in the
			// ISR vector table) until the first delay. After the delay, there
			// are 17 more cycles until the pin value is read (excluding the
			// delay in the loop).
			// We want to have a total delay of 1.5 bit time. Inside the loop,
			// we already wait for 1 bit time - 23 cycles, so here we wait for
			// 0.5 bit time - (71 + 18 - 22) cycles.
			_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

			// There are 23 cycles in each loop iteration (excluding the delay)
			_rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

			// There are 37 cycles from the last bit read to the start of
			// stopbit delay and 11 cycles from the delay until the interrupt
			// mask is enabled again (which _must_ happen during the stopbit).
			// This delay aims at 3/4 of a bit time, meaning the end of the
			// delay will be at 1/4th of the stopbit. This allows some extra
			// time for ISR cleanup, which makes 115200 baud at 16Mhz work more
			// reliably
			_rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
#else // Timings counted from gcc 4.3.2 output
			// Note that this code is a _lot_ slower, mostly due to bad register
			// allocation choices of gcc. This works up to 57600 on 16Mhz and
			// 38400 on 8Mhz.
			_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
			_rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
			_rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
#endif


			// Enable the PCINT for the entire port here, but never disable it
			// (others might also need it, so we disable the interrupt by using
			// the per-pin PCMSK register).
			* digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
			// Precalculate the pcint mask register and value, so setRxIntMask
			// can be used inside the ISR without costing too much time.
			_pcint_maskreg = digitalPinToPCMSK(_receivePin);
			_pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

			tunedDelay(_tx_delay); // if we were low this establishes the end
		}						
	//}
	//else if (_rx_delay_stopbit)//taken from customSoftwareSerial // Set up RX interrupts, but only if we have a valid RX baud rate
	//{
	//	//->code from CustomSoftwareSerial
	//	if (digitalPinToPCICR(_receivePin))
	//	{
	//		*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
	//		*digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
	//	}
	//	tunedDelay(_tx_delay); // if we were low this establishes the end
	//}
	clearErrors();
	//DEBUG("SoftwareSerial: Setup RX in begin: Successfull");
#if _DEBUG
	pinMode(_DEBUG_PIN1, OUTPUT);
	pinMode(_DEBUG_PIN2, OUTPUT);
#endif
#if SSWHD_DEBUG
	pinMode(DEBUGRW_PIN, OUTPUT);
#endif
	_speed = speed;//register current speed/baud rate
	DEBUG("SoftwareSerial.begin: Speed:"+(String)_speed);
	listen();
}

void SoftwareSerialWithHalfDuplex::begin(long speed, uint16_t configuration) 
{
	this->setPort(configuration);
	this->begin(speed);
}
//taken from CustomSoftwareSerial
void SoftwareSerialWithHalfDuplex::clearErrors()
{
	errors = sswhdErrors::sswhdNoErrors;
}

sswhdErrors SoftwareSerialWithHalfDuplex::getErrors()
{
	return errors;
}

void SoftwareSerialWithHalfDuplex::setError(sswhdErrors error)
{
	errors |= error;
}

void SoftwareSerialWithHalfDuplex::setRxIntMsk(bool enable)
{
	if (enable)
		*_pcint_maskreg |= _pcint_maskvalue;
	else
		*_pcint_maskreg &= ~_pcint_maskvalue;
}

void SoftwareSerialWithHalfDuplex::end()
{
	stopListening();
}

#if SSWHD_DEBUG
void SoftwareSerialWithHalfDuplex::dumpReceiveBuffer()
{
	for (int i = _receive_buffer_head; i < _receive_buffer_tail; i++)
	{
		Serial.print((uint8_t)_receive_buffer[i] < 0x10 ? " 0" : " ");
		Serial.print((uint8_t)_receive_buffer[i], HEX);
	}
	Serial.println("");
}
#endif

// Read data from buffer
int SoftwareSerialWithHalfDuplex::read()
{
	if (!isListening())
	{
		setError(sswhdErrors::sswhdIsNotListening);
		return -1;
	}
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
	{
		setError(sswhdErrors::sswhdBufferIsEmpty);
		return -1;
	}
	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
	_receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
	return d;
}

int SoftwareSerialWithHalfDuplex::available()
{
	if (!isListening())
	{
		//DEBUG("SoftwareSerial::available: Is is not listening!");
		setError(sswhdErrors::sswhdIsNotListening);
		return -1;
	}
	//DEBUG("SoftwareSerial::available: rbt:" + (String)_receive_buffer_tail + ", rbh:" + (String)_receive_buffer_head);
	return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;

}

//TBS SmartAudio protocol requires before send command set line to LOW or write dummy byte 0x00
//this procedure imitiates this requirement
void SoftwareSerialWithHalfDuplex::writeDummyByte()
{
	uint8_t oldSREG = SREG;
#if DEBUGWRITE
	digitalWrite(DEBUGRW_PIN, HIGH);
#endif
	cli();  // turn off interrupts for a clean txmit
	// NS - Set Pin to Output
	if (!_full_duplex)															//NS Added
		pinMode(_transmitPin, OUTPUT);								//NS Added  

	//set pin to LOW (or in inverse logis - to HIGH)
	tx_pin_write(_inverse_logic ? HIGH : LOW);	
	//write one stop bit, number of databit + stop bits
	for (int i = 0; i < _numberOfDataBit + _numberOfStopBit + 1; i++)
	{
		tunedDelay(_tx_delay);
	}		
	
	// NS - Set Pin back to Input
	if (!_full_duplex)
	{
		pinMode(_transmitPin, INPUT);							//NS Added
		*_transmitPortRegister |= _transmitBitMask; 									//pull _transmitPin HIGH
	}

	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);//is this stop bit?
#if DEBUGWRITE
	digitalWrite(DEBUGRW_PIN, LOW);
#endif
}
size_t SoftwareSerialWithHalfDuplex::write(uint8_t b)
{
	if (_tx_delay == 0)
	{
		setWriteError();
		setError(sswhdErrors::sswhdTxDelayIsZero);
		return 0;
	}
#if DEBUGWRITE
	digitalWrite(DEBUGRW_PIN, HIGH);
#endif
	// By declaring these as local variables, the compiler will put them
	// in registers _before_ disabling interrupts and entering the
	// critical timing sections below, which makes it a lot easier to
	// verify the cycle timings
	volatile uint8_t* reg = _transmitPortRegister;
	uint8_t reg_mask = _transmitBitMask;
	uint8_t inv_mask = ~_transmitBitMask;
	uint8_t oldSREG = SREG;
	bool inv = _inverse_logic;
	uint16_t delay = _tx_delay;

	if (inv)
		b = ~b;

	cli();  // turn off interrupts for a clean txmit

	// NS - Set Pin to Output
	if (!_full_duplex)															//NS Added
		pinMode(_transmitPin, OUTPUT);										//NS Added    

	// Write the start bit
	/*if (inv)
		*reg |= reg_mask;
	else
		*reg &= inv_mask;*/
	tx_pin_write(_inverse_logic ? HIGH : LOW);
	tunedDelay(delay);
	//taken from CustomSoftwareSerial
	uint8_t numberOfBits1 = calculateNumberOfBits1(b);
	uint8_t maxValueOfData = round(pow(2, this->_numberOfDataBit - 1));

	// Write each of the 8 bits
	//for (uint8_t i = 8; i > 0; --i)
	//{
	//	if (b & 1) // choose bit
	//		*reg |= reg_mask; // send 1
	//	else
	//		*reg &= inv_mask; // send 0

	//	tunedDelay(delay);
	//	b >>= 1;
	//}
	//start - taken from CustomSoftwareSerial
	if (inv)
	{
		for (byte mask = 0x01; mask && mask <= this->_maxValueOfDataBit; mask <<= 1)
		{
			if (b & mask) // choose bit
				tx_pin_write(LOW); // send 1
			else
				tx_pin_write(HIGH); // send 0

			tunedDelay(_tx_delay);
		}

	}
	else
	{
		for (byte mask = 0x01; mask && mask <= this->_maxValueOfDataBit; mask <<= 1)
		{
			if (b & mask) // choose bit
				tx_pin_write(HIGH); // send 1
			else
				tx_pin_write(LOW); // send 0

			tunedDelay(_tx_delay);
		}
	}
	writeParityBits(numberOfBits1);
	if (this->_parityBit != NONE)//this added from cooment at github
	{
		tunedDelay(_tx_delay);
	}
	writeStopBits();
	//end - taken from CustomSoftwareSerial
	
	// restore pin to natural state
	if (inv)
		*reg &= inv_mask;
	else
		*reg |= reg_mask;

	// NS - Set Pin back to Input
	if (!_full_duplex) 
	{
		pinMode(_transmitPin, INPUT);							//NS Added
		*reg |= reg_mask; 									//pull _transmitPin HIGH
	}
	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);//is this stop bit?
#if DEBUGWRITE
	digitalWrite(DEBUGRW_PIN, LOW);
#endif
	return 1;
}

void SoftwareSerialWithHalfDuplex::flush()
{
	if (!isListening())
	{
		setError(sswhdErrors::sswhdIsNotListening);
		return;
	}
	uint8_t oldSREG = SREG;
	cli();//disable interrupts
	_receive_buffer_head = _receive_buffer_tail = 0;
	SREG = oldSREG;//restore interrupts
}

int SoftwareSerialWithHalfDuplex::peek()
{
	if (!isListening())
	{
		setError(sswhdErrors::sswhdIsNotListening);
		return -1;
	}
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
	{
		setError(sswhdErrors::sswhdBufferIsEmpty);
		return -1;
	}
	// Read from "head"
	return _receive_buffer[_receive_buffer_head];
}

//start - taken from CustomSoftwareSerial
void SoftwareSerialWithHalfDuplex::setPort(uint16_t configuration) 
{	
	this->_numberOfDataBit = (uint8_t)(configuration / 100);
	this->_maxValueOfDataBit = round(pow(2, this->_numberOfDataBit - 1));
	this->_parityBit = (Parity)(configuration % 100 / 10);
	this->_numberOfStopBit = (uint8_t)(configuration % 10);
	this->_configuration = configuration;
	DEBUG("SoftwareSerial::setPort: Configuration: " + (String)configuration);
	DEBUG("SoftwareSerial::setPort: Number of databit: " + (String)_numberOfDataBit);
	DEBUG("SoftwareSerial::setPort: Number of Stopbit: " + (String)_numberOfStopBit);
	DEBUG("SoftwareSerial::setPort: Max Value of Databit: " + (String)_maxValueOfDataBit);
}
uint16_t SoftwareSerialWithHalfDuplex::getConfiguration()
{
	return _configuration;
}
long SoftwareSerialWithHalfDuplex::getSpeed()
{
	//DEBUG("SoftwareSerial::getSpeed: "+(String)_speed);
	return _speed;
}
uint8_t SoftwareSerialWithHalfDuplex::getNumberOfDataBit() 
{	
	return this->_numberOfDataBit;
}

Parity SoftwareSerialWithHalfDuplex::getParity() 
{
	return this->_parityBit;	
}

uint8_t SoftwareSerialWithHalfDuplex::getNumberOfStopBit() 
{
	return this->_numberOfStopBit;
}

void SoftwareSerialWithHalfDuplex::writeParityBits(uint8_t numberOfBit1) 
{
	if (this->_parityBit == NONE) 
	{
		return;
	}

	if ((this->_parityBit == EVEN && (numberOfBit1 & 0x01) && !_inverse_logic) ||
		(this->_parityBit == EVEN && !(numberOfBit1 & 0x01) && _inverse_logic) ||
		(this->_parityBit == ODD && (numberOfBit1 & 0x01) && _inverse_logic) ||
		(this->_parityBit == ODD && !(numberOfBit1 & 0x01) && !_inverse_logic)) 
	{
		tx_pin_write(HIGH); // send 1
	}

	if ((this->_parityBit == ODD && (numberOfBit1 & 0x01) && !_inverse_logic) ||
		(this->_parityBit == ODD && !(numberOfBit1 & 0x01) && _inverse_logic) ||
		(this->_parityBit == EVEN && (numberOfBit1 & 0x01) && _inverse_logic) ||
		(this->_parityBit == EVEN && !(numberOfBit1 & 0x01) && !_inverse_logic))
	{
		tx_pin_write(LOW); // send 1
	}

	if (this->_parityBit == ODD) 
	{
		if (numberOfBit1 & 0x01 && !_inverse_logic) 
		{
			tx_pin_write(LOW); // send 0
			return;
		}

		if (numberOfBit1 & 0x01 && _inverse_logic) 
		{
			tx_pin_write(HIGH); // send 1
			return;
		}

	}
}
void SoftwareSerialWithHalfDuplex::writeStopBits() 
{
	for (int8_t stopBitIndex = 0; stopBitIndex < _numberOfStopBit; stopBitIndex++)
	{
		if (_inverse_logic)
		{
			tx_pin_write(LOW);
		}
		else 
		{
			tx_pin_write(HIGH);
		}
		tunedDelay(_tx_delay/*_rx_delay_stopbit*/);//???
	}
}

uint8_t SoftwareSerialWithHalfDuplex::calculateNumberOfBits1(uint8_t sentData) 
{
	uint8_t numberOfBit1 = 0;
	uint8_t index;
	for (index = 0x80; index; index >>= 1) 
	{
		if (sentData & index) numberOfBit1++;
	}

	return numberOfBit1;
}
	//end - taken from CustomSoftwareSerial
