#include <Arduino.h>
#include <SoftwareSerialWithHalfDuplex.h>
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"
#ifndef VTXControl_h
#define VTXControl_h
//--------------------------
//#define VTXCDEBUG 1 //Uncomment this define to see the diagnostics
//--------------------------
#if VTXCDEBUG
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#if VTXCDEBUG
static void dumpBuffer(const uint8_t* data, const int8_t len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print((uint8_t)data[i] < 0x10 ? " 0" : " ");
    Serial.print((uint8_t)data[i], HEX);
  }
  Serial.println("");
}
#endif

enum VTXMode
{
  SmartAudio = 1,
  Tramp = 2,
};
enum VTXErrors
{
  vtxNoErrors = 0,  
  vtxParseResponseInvalidCRCOrBuffer = 0x01,
  vtxIncomingBytesZero = 0x02,
  vtxIncomingBytesLessHeaderSize = 0x04,
  vtxIncomingByteNotEqualSyncByte = 0x08,
  vtxIncomingByteNotEqualHeaderByte = 0x10,
  vtxPacketSizeGreaterMaxPacketSize = 0x20,
  vtxBufferLengthLessWholePacket = 0x40,
  //--
  vtxportIsNotListening = 0x0100,
  vtxportBufferIsEmpty = 0x0200,
  vtxportTxDelayIsZero = 0x0400,
  vtxportRXDelayStopBitNotSet = 0x8000,
  vtxtrampNotInited = 0x10000,
  vtxLastByteNotSyncStop = 0x20000,
};


class VTXControl
{
   
public:  
  VTXControl(int vtxMode, int softPin, int responseTimeOut = 1000, bool smartBaudRate = true, int numtries = 3);
  void flush();
  void waitForInMs(unsigned int ms);
  //bool setPitMode(bool enabled);
  bool setChannel(int freqIndex);//sets frequency by channel index
  bool setFrequency(uint16_t freq);//set frequency by freq value
  bool setPower(int pwrLevel);//sets power by power index in table of powers
  bool setPowerInmW(uint16_t pwrmW);//sets power by value in mW
  bool setNextChannel();
  bool setPrevChannel();
  bool updateParameters();  
  int getPowerLevel() { return pwr_Level; }
  int getChannelIndex() { return ch_index; }
  bool getPitMode() { return pitMode; }
  bool sa_readResponse();
  void clearErrors();
  VTXErrors getErrors();
  long getSpeed();
#if VTXCDEBUG
  bool testSMAWrite();
  bool testSMAResponseFromSerial1();
#endif
private:
  SoftwareSerialWithHalfDuplex* port;
  int vtx_mode = VTXMode::SmartAudio;//default
  VTXErrors errors = VTXErrors::vtxNoErrors;
  long sa_offerNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  void setError(VTXErrors error);
  ProtocolVersion sa_protocol_version;//smart audio protocol version
  int pwr_Level = -1;//default value -1 means not updated (not requested from VTX)
  int ch_index = -1;//default value -1 means not updated (not requested from VTX)
  bool pitMode = false;
  bool initialized = false;//tramp protocol needs to be initialized, so this var reflects state of Tramp initialization
  int _responseTimeOut = 1000;//in ms
  int _numtries = 3;//num tries to send request and receive response, after that we try to change baud rate and try again
  bool _smartBaudRate = true;//tries to find apporpriated baud rate to communicate with VTX, if false- just work on fixed initial baudrate
  
  //some utility functions  
  int getChannelIndex(uint16_t freq);
  uint16_t getChannelFrequency(int chIndex);
  int getPowerIndexFromMW(uint16_t pwrInmW);
  int getPowerIndexFromDbm(uint16_t pwrInDbm);
  int getPowerIndexFromV1(uint16_t pwrValue);
  uint16_t getPowerInmW(int pwrIndex);
    
  bool sa_updateSettings(); 
  bool sa_ignoreCrc() const { return SMARTAUDIO_IGNORE_CRC; }  
  bool sa_parseResponseBuffer(const uint8_t* buffer);
  // command functions
  //bool sa_setPitMode(int enabled);
  bool sa_getSettings();
  bool sa_setChannel(uint8_t channel);
  bool sa_setPower(int pwrLevel);      
  bool trampSendPacket(uint8_t* packet, bool respRequired);//trampFrame_t* packet);
  bool trampPush(const uint8_t* packet);//const trampFrame_t* object);
  bool trampSendCmd(uint8_t cmd);
  bool trampSendCmd(uint8_t cmd, uint16_t param);

  //uint8_t trampGetState();
  bool trampSetFrequency(uint16_t freq);
  bool trampSetPower(uint16_t milliWatts);
  bool trampInit();
  //bool trampSetPitMode(bool enabled);
  bool trampUpdate();
  long trampOfferNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  bool trampGetStatus();  
  bool trampReadResponse();    
};
#endif