/*
SoftwareSerialWithHalfDuplex.h (formerly SoftwareSerial.h) - 
Multi-instance software serial with half duplex library for Arduino/Wiring

By default the library works the same as the SoftwareSerial library, 
but by adding a couple of additional arguments it can be configured for 
half-duplex. In that case, the transmit pin is set by default to an input, 
with the pull-up set. When transmitting, the pin temporarily switches to 
an output until the byte is sent, then flips back to input. When a module 
is receiving it should not be able to transmit, and vice-versa. 
This library probably won't work as is if you need inverted-logic.

This is a first draft of the library and test programs. It appears to work, 
but has only been tested on a limited basis. The library also works with 
Robotis Bioloid AX-12 motors. Seems fairly reliable up to 57600 baud. 
As with all serial neither error checking, nor addressing are implemented, 
so it is likely that you will need to do this yourself. Also, you can make 
use of other protocols such as i2c. I am looking for any feedback, advice 
and help at this stage. Changes from SoftwareSerial have been noted with a 
comment of "//NS" for your review. Only a few were required.
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

#ifndef SoftwareSerialWithHalfDuplex_h
#define SoftwareSerialWithHalfDuplex_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/
//--------------------------
//#define SSWHD_DEBUG 1 //Uncomment this define to see the diagnostics
//--------------------------
//#define _DEBUG true
// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#if SSWHD_DEBUG
#define DEBUGRECV true
#define DEBUGRW_PIN 13
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGWRITE 0
#define DEBUGRECV 0
#endif

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif
enum sswhdErrors
{
  sswhdNoErrors = 0x00,
  sswhdIsNotListening = 0x01,
  sswhdBufferIsEmpty = 0x02,
  sswhdTxDelayIsZero = 0x04,
  sswhdRXDelayStopBitNotSet = 0x08,
};
//taken from CustomSoftwareSerial
#define CSERIAL_5N1 501
#define CSERIAL_6N1 601
#define CSERIAL_7N1 701
#define CSERIAL_8N1 801

#define CSERIAL_5N2 502
#define CSERIAL_6N2 602
#define CSERIAL_7N2 702
#define CSERIAL_8N2 802

#define CSERIAL_5O1 511
#define CSERIAL_6O1 611
#define CSERIAL_7O1 711
#define CSERIAL_8O1 811

#define CSERIAL_5O2 512
#define CSERIAL_6O2 612
#define CSERIAL_7O2 712
#define CSERIAL_8O2 812

#define CSERIAL_5E1 521
#define CSERIAL_6E1 621
#define CSERIAL_7E1 721
#define CSERIAL_8E1 821

#define CSERIAL_5E2 522
#define CSERIAL_6E2 622
#define CSERIAL_7E2 722
#define CSERIAL_8E2 822
//taken from CustomSoftwareSerial
enum Parity {
  NONE = 0,
  ODD = 1,
  EVEN = 2
};

class SoftwareSerialWithHalfDuplex : public Stream
{
private:
  // per object data
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitPin;								//NS Added
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;
  uint8_t _startBits = 1, _stopBits = 1;
  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  bool _buffer_overflow;
  bool _inverse_logic;
  bool _full_duplex;							//NS Added
  //taken from CustomSoftwareSerial
  uint8_t _numberOfDataBit;
  uint8_t _maxValueOfDataBit;
  Parity _parityBit;
  uint8_t _numberOfStopBit;
  long _speed;
  uint16_t _configuration;
  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerialWithHalfDuplex *active_object;

  // private methods
  void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state);// __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  void setRxIntMsk(bool enable) __attribute__((__always_inline__));
  //taken from CustomSoftwareSerial
  void setPort(uint16_t configuration);

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  //The pin should support change interrupts :
  //-0n the Mega and Mega 2560 only the following can be used for RX : 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8(62), A9(63), A10(64), A11(65), A12(66), A13(67), A14(68), A15(69).
  //- On the Leonardo and Micro only the following can be used for RX : 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
  //SoftwareSerialWithHalfDuplex(uint8_t pin) { SoftwareSerialWithHalfDuplex(pin, pin, false, false); }
  SoftwareSerialWithHalfDuplex(uint8_t receivePin, uint8_t transmitPin, 
    bool inverse_logic = false, bool full_duplex = true);
  ~SoftwareSerialWithHalfDuplex();
  void begin(long speed);
  //taken from CustomSoftwareSerial
  void begin(long speed, uint16_t configuration);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();
  //taken from CustomSoftwareSerial
  uint8_t getNumberOfDataBit();
  Parity getParity();
  uint8_t getNumberOfStopBit();
  uint16_t getConfiguration();
  long getSpeed();
  uint8_t calculateNumberOfBits1(uint8_t sentData);
  void writeStopBits();
  void writeParityBits(uint8_t numberOfBit1);
  void writeDummyByte();
#ifdef DEBUG
  void dumpReceiveBuffer();
#endif
  virtual size_t write(uint8_t byte);  
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }
  void clearErrors();
  sswhdErrors getErrors();

  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
private:
  sswhdErrors errors = sswhdErrors::sswhdNoErrors;
  void setError(sswhdErrors error);
};

// Arduino 0012 workaround
#ifdef int
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round
#endif
#endif
