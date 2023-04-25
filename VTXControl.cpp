#include <Arduino.h>
#include <util/delay.h>
#include "VTXControl.h"
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"
#include "SoftwareSerialWithHalfDuplex.h"

//these parameters for EACHINE TX5258
const uint16_t powers[4] = { 25, 200, 500, 800 };//in mW
//these parameters for JHEMCU RuiBet Tran3016W
//uint16_t powers[5] = { 25, 200, 400, 800, 1600 };//in mW
const uint16_t powers_v1[4] = { 7/*25mw*/, 16/*200mw*/, 25/*500mw*/, 40/*800mw*/ };//for SmartAudio protocol v1
//Smartaudio v2.1 protocol seems to be not documented (TBS documented just v1 and v2), but defined in ArduPilot
const uint16_t powers_v21[4] = { 14/*25mw*/, 20/*200mw*/, 26/*500mw*/, 30/*800mw*/ };//for SmartAudio protocol v2.1 in dbm
const uint16_t freqs[40] = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,//A band
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,//B band
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,//E band
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,//F band
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,//r Band
};

// Instantiates the VTX object with the specified parameters
VTXControl::VTXControl(int vtxMode, int softPin, int responseTimeOut = 1000, bool smartBaudRate = true, int numtries=3)
{    
  DEBUG("VTXControl: Create");
  vtx_mode = vtxMode;
  port = new SoftwareSerialWithHalfDuplex(softPin, softPin, false, false);
  _responseTimeOut = responseTimeOut;
  _numtries = numtries;
  _smartBaudRate = smartBaudRate;
  
  //with SmartAudio protocol according to TBS documentation
  //!!! Please do remember, for SmartAudio - logic level = 3.3V !!!
  //we use 4800bps 1 Start bit and 2 Stop bit, 8 data bits
  //for SmartAudio clones(wthiout TBS license) - 1 start bit, 2 stop bit, 8 data bits
  //with Tramp - 9600bps, 1 start bit, 1 stop bit, 8 data bits
  port->begin(vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_BAUD : AP_TRAMP_UART_BAUD,
    vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_CFG : AP_TRAMP_UART_CFG);
  waitForInMs(200);
}
void VTXControl::flush()
{
  port->flush();
}
void VTXControl::clearErrors()
{
  errors = VTXErrors::vtxNoErrors;
  port->clearErrors();
}
long VTXControl::getSpeed()
{
  //DEBUG("VTXControl: getSpeed:" + (String)port->getSpeed());
  port->getSpeed();
}
VTXErrors VTXControl::getErrors()
{
  sswhdErrors err = port->getErrors();
  if (err != sswhdErrors::sswhdNoErrors)
  {
    if (err & sswhdErrors::sswhdIsNotListening != 0)
      errors |= vtxportIsNotListening;
    if (err & sswhdErrors::sswhdBufferIsEmpty != 0)
      errors |= vtxportBufferIsEmpty;
    if (err & sswhdErrors::sswhdTxDelayIsZero != 0)
      errors |= vtxportTxDelayIsZero;
    if (err & sswhdErrors::sswhdRXDelayStopBitNotSet != 0)
      errors |= vtxportRXDelayStopBitNotSet;
  }
  return errors;
}
void VTXControl::setError(VTXErrors error)
{
  errors |= error;
}
bool VTXControl::sa_updateSettings()
{
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("Smart Audio Update_settings");
  while(1)//speed cycle
  {
    for (int i = 0; i < _numtries; i++)
    {
      if (sa_getSettings())
      {
        if (sa_readResponse())
          return true;//in case of success we return true
      }
      port->flush();
    }
    if (_smartBaudRate)
    {
      //so we need to change baud rate?
      newspeed = sa_offerNewSpeed(newspeed);
      if (newspeed != startspeed)
      {
        DEBUG("update_settings, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      }
      else
      {
        DEBUG("update_settings, end of tries");
        return false;//out of change speed cycle
      }
    }
    else break;
  }
  return false;
}
long VTXControl::sa_offerNewSpeed(long currentSpeed)
{
  //if speed == AP_SMARTAUDIO_UART_BAUD we're going to AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to AP_SMARTAUDIO_SMARTBAUD_MAX
  //if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to AP_SMARTAUDIO_UART_BAUD
  //so AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed)
  {
  case AP_SMARTAUDIO_UART_BAUD: return AP_SMARTAUDIO_UART_BAUD + AP_SMARTAUDIO_SMARTBAUD_STEP; 
  case AP_SMARTAUDIO_SMARTBAUD_MIN: return AP_SMARTAUDIO_SMARTBAUD_MIN + AP_SMARTAUDIO_SMARTBAUD_STEP;
  case AP_SMARTAUDIO_SMARTBAUD_MAX: return AP_SMARTAUDIO_SMARTBAUD_MIN; 
  }
  //if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  // ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_SMARTAUDIO_UART_BAUD && currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MAX) || 
    (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MIN && currentSpeed < AP_SMARTAUDIO_UART_BAUD))
    return currentSpeed + AP_SMARTAUDIO_SMARTBAUD_STEP;  
  if (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MAX) return AP_SMARTAUDIO_SMARTBAUD_MAX;
  if (currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MIN) return AP_SMARTAUDIO_SMARTBAUD_MIN;
}
#if VTXCDEBUG
bool VTXControl::testSMAWrite()
{
  return sa_getSettings();
}
bool VTXControl::testSMAResponseFromSerial1()
{  
  SettingsResponseFrame response;
  response.header.init(SMARTAUDIO_RSP_GET_SETTINGS_V1, 5);
  response.channel = 2;
  response.power = 25;//v1
  response.operationMode = 0x04;//pitmode turned on
  response.frequency = 5800;
  DEBUG("push response (Serial1):" + (String)sizeof(SettingsResponseFrame));
  bool res = Serial1.write((uint8_t*)&response, sizeof(SettingsResponseFrame)) == sizeof(SettingsResponseFrame);
  //Packet command;
  //// according to the spec the length should include the CRC, but no implementation appears to
  //// do this
  //command.frame.header.init(SMARTAUDIO_CMD_GET_SETTINGS, 0);
  //command.frame_size = SMARTAUDIO_COMMAND_FRAME_SIZE;
  //command.frame.payload[0] = crc8_dvb_s2_update(0, &command.frame, SMARTAUDIO_COMMAND_FRAME_SIZE - 1);
  //DEBUG("push to write:" + (String)sizeof(Packet));
  //bool res = Serial1.write((uint8_t*)&command, sizeof(Packet)) == sizeof(Packet);

  return res;
}
#endif
//bool VTXControl::sa_setPitMode(int enabled)
//{
//  //activate - IN RANGE PIT FLAG
//  //deactivate - (Quit PIT MODE
//  uint8_t mode = enabled ? 0x01 : 0x04;
//  if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_MODE, mode))
//  {
//    //DEBUG("End sma_setPitMode");
//    return readResponse(); //updateParameters();
//  }
//  return false;
//}
bool VTXControl::sa_setPower(int pwrLevel)
{
  static uint8_t buf[6] = { 0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, 0x00, 0x00 };
  switch (sa_protocol_version)
  {
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
    //res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, powers_v1[pwrLevel]);
    buf[4] = powers_v1[pwrLevel];
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v2:
    //debug("Setting power to %d", power_level);
    //res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, pwrLevel);
    buf[4] = pwrLevel;
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
    //res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, powers_v21[pwrLevel] | 0x80);
    buf[4] = powers_v21[pwrLevel] | 0x80;
    break;
  }
  buf[5] = sa_CRC8(buf, 5);
  
  DEBUG("sa_setPower, push to write:" + (String)sizeof(buf));
  //according to SA documentation:
  //The SmartAudio line need to be low before a frame is sent. 
  //If the host MCU can’t handle this it can be done by
  //sending a 0x00 dummy byte in front of the actual frame.  
  port->writeDummyByte();
  bool res = port->write((uint8_t*)&buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  port->listen();

  return res;
}
bool VTXControl::sa_setChannel(uint8_t channel)
{
  static uint8_t buf[6] = { 0xAA, 0x55, SMARTAUDIO_CMD_SET_CHANNEL, 1, 0, 0};
  buf[4] = channel;
  buf[5] = sa_CRC8(buf, 5);//exclude crc byte
  DEBUG("sa_setChannel, push to write:" + (String)sizeof(buf));
  //according to SA documentation:
  //The SmartAudio line need to be low before a frame is sent. 
  //If the host MCU can’t handle this it can be done by
  //sending a 0x00 dummy byte in front of the actual frame.  
  port->writeDummyByte();
  bool res = port->write((uint8_t*)&buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  port->listen();
  
  return res;
}
bool VTXControl::sa_getSettings()
{  
  //taken from betaflight vtx_smartaudio.c
  static uint8_t buf[5] = { 0xAA, 0x55, SMARTAUDIO_CMD_GET_SETTINGS, 0x00, 0x9F };    
  DEBUG("sa_GetSettings, push to write:" + (String)sizeof(buf));  
  //according to SA documentation:
  //The SmartAudio line need to be low before a frame is sent. 
  //If the host MCU can’t handle this it can be done by
  //sending a 0x00 dummy byte in front of the actual frame.
  port->writeDummyByte();
  //port->write((uint8_t)0x00);
  bool res = port->write((uint8_t*)&buf, sizeof(buf)) == sizeof(buf);
  //port->writeDummyByte();
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  port->listen();
  return res;
}

//relatively precise function as replacement of delay
//delay stop processor/interrupts usage, so we need to use delayMcroseconds
//but delayMicroseconds uses max 16384 value
void VTXControl::waitForInMs(unsigned int ms)
{
  int in_tens_ms = ms / 10;
  int reminder_ms = ms - (in_tens_ms * 10);
  //tens of ms
	for (int i = 0; i < in_tens_ms; i++)
	{
		delayMicroseconds(10000);
	}
  //reminded ms
  for (int i = 0; i < reminder_ms; i++)
  {
    delayMicroseconds(1000);
  }
  //DEBUG("VTXControl::waitForInMs:" + (String)ms + "ms End");
}
bool VTXControl::sa_readResponse()
{
  // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the request
  // and the initial response is 40ms long so we should wait at least 140ms before giving up
  waitForInMs(_responseTimeOut);
  
  int16_t incoming_bytes_count = port->available();  
  // check if it is a response in the wire
  if (incoming_bytes_count < 0)
  { 
    DEBUG("sa readResponse, return error on Avaialble");
    return false;
  }
  else if (incoming_bytes_count == 0) 
  {
    DEBUG("sa readResponse, incoming bytes=0, return false");
    setError(VTXErrors::vtxIncomingBytesZero);// 
    return false;
  }
  //now we have no-zero length response, we can dump it
#if VTXCDEBUG
  port->dumpReceiveBuffer();
#endif
  //special feature to correctly read Eachine TX5258 response, it generates first 0x00, 0x00 byte before sync_byte in response  
  //so we skip zero bytes at start of packet
  uint8_t b = port->peek();
  int skippedbytes = 0;
  while (b == 0x00)
  {
    port->read();//skip 0x00 byte
    skippedbytes++;
    b = port->peek();
  }
  if (skippedbytes > 0)
  {
    incoming_bytes_count = port->available();//refresh incoming_bytes_count
    DEBUG("sa readResponse, skipped zero bytes =" + (String)skippedbytes);
  }
  const uint8_t response_header_size = sizeof(FrameHeader);
  // wait until we have enough bytes to read a header
  if (incoming_bytes_count < response_header_size) 
  {
    DEBUG("sa readResponse, incoming_bytes_count < response_header_size: " + (String)incoming_bytes_count + ", return false");
    setError(VTXErrors::vtxIncomingBytesLessHeaderSize); 
    return false;
  }
  DEBUG("sa readResponse, incoming_bytes_count = " + (String)incoming_bytes_count);
  // allocate response buffer
  uint8_t response_buffer[AP_SMARTAUDIO_MAX_PACKET_SIZE];
  uint8_t buffer_length = 0;
  // expected packet size
  uint8_t packet_size = 0;
  // now have at least the header, read it if necessary
  
  //read the first non-zero byte
	b = port->read();
	// didn't see a sync byte, discard and go around again
	if (b != SMARTAUDIO_SYNC_BYTE)
	{
		DEBUG("readResponse, b != SMARTAUDIO_SYNC_BYTE (b=" + (String)b + "), return false");
		setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
		return false;
	}
	response_buffer[buffer_length++] = b;
	//read header byte
	b = port->read();
	// didn't see a header byte, discard and reset
	if (b != SMARTAUDIO_HEADER_BYTE)
	{
		buffer_length = 0;
		DEBUG("readResponse, b != SMARTAUDIO_HEADER_BYTE, return false");
		setError(VTXErrors::vtxIncomingByteNotEqualHeaderByte);
		return false;
	}
	response_buffer[buffer_length++] = b;
	// read the rest of the header
	for (; buffer_length < response_header_size; buffer_length++)
	{
		b = port->read();
		response_buffer[buffer_length] = b;
	}

	FrameHeader* header = (FrameHeader*)response_buffer;
	incoming_bytes_count -= response_header_size;

	// implementations that ignore the CRC also appear to not account for it in the frame length
	//if (sa_ignoreCrc())
	//{
	//  header->length++;
	//}
	packet_size = header->length;
  
  // read the rest of the packet
  // check for overflow
  if (packet_size >= AP_SMARTAUDIO_MAX_PACKET_SIZE)
  {
    DEBUG("readResponse, rest packet size >= AP_SMARTAUDIO_MAX_PACKET_SIZE, return false");
    setError(VTXErrors::vtxPacketSizeGreaterMaxPacketSize);
    return false;
  }
  for (uint8_t i = 0; i < packet_size; i++)
  {
    uint8_t b = port->read();    
    response_buffer[buffer_length++] = b;
  }

  // didn't get the whole packet
  if (buffer_length < packet_size + response_header_size) 
  {
    DEBUG("readResponse, buffer_length < packet_size + response_header_size, return false");
    setError(VTXErrors::vtxBufferLengthLessWholePacket);
    return false;
  }
  
  bool correct_parse = sa_parseResponseBuffer(response_buffer);
  port->flush();//clear the read buffer
  // successful response, wait another 100ms to give the VTX a chance to recover
  // before sending another command.  
  waitForInMs(100);
  DEBUG("readResponse end, correct_parse = " + (String)correct_parse);
  return correct_parse;
}
bool VTXControl::sa_parseResponseBuffer(const uint8_t* buffer)
{
  const FrameHeader* header = (const FrameHeader*)buffer;
  const uint8_t fullFrameLength = sizeof(FrameHeader) + header->length;
  const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
  const uint8_t* startPtr = buffer + 2;//exclude header and sync bytes
  const uint8_t* endPtr = buffer + headerPayloadLength;
  DEBUG("sa_parse_response_buffer(), fullFrameLength="+(String)fullFrameLength);
	if (!sa_ignoreCrc())
	{
#if VTXCDEBUG
    dumpBuffer(buffer, headerPayloadLength);    
#endif
     uint8_t crc = sa_CRC8(buffer, headerPayloadLength);
     //uint8_t crc = sa_CRC8(startPtr, headerPayloadLength-2);
     uint8_t crc_resp = *(endPtr);
     if (crc != crc_resp)
     {
       DEBUG("sa_parse_response_buffer() failed - invalid CRC or header, crc in resp=" +(String)crc_resp+", crc calc="+(String)crc);
       setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
       return false;
     }
	}
	if (header->headerByte != SMARTAUDIO_HEADER_BYTE)
	{
		setError(VTXErrors::vtxIncomingByteNotEqualHeaderByte);
		return false;
	}
	if (header->syncByte != SMARTAUDIO_SYNC_BYTE)
	{
		setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
		return false;
	}  
	switch (header->command)
	{
	case SMARTAUDIO_RSP_GET_SETTINGS_V1:
	{
		DEBUG("sa_parse_response_buffer(), Protocol version 1");
		sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v1;
		const SettingsResponseFrame* resp = (const SettingsResponseFrame*)buffer;
		pwr_Level = getPowerIndexFromV1(resp->power);
		ch_index = resp->channel;
		pitMode = (resp->operationMode & 0x04) != 0;
	}
	break;
	case SMARTAUDIO_RSP_GET_SETTINGS_V2:
	{
		DEBUG("sa_parse_response_buffer(), Protocol version 2");
		sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;
		const SettingsResponseFrame* respv2 = (const SettingsResponseFrame*)buffer;
		pwr_Level = respv2->power;
		ch_index = respv2->channel;
		pitMode = (respv2->operationMode & 0x04) != 0;
	}
	break;

	case SMARTAUDIO_RSP_GET_SETTINGS_V21:
	{
		DEBUG("sa_parse_response_buffer(), Protocol version 2.1");
		sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v21;
		const SettingsExtendedResponseFrame* respv21 = (const SettingsExtendedResponseFrame*)buffer;
		pwr_Level = getPowerIndexFromDbm(respv21->power_dbm);
		ch_index = respv21->settings.channel;
		pitMode = (respv21->settings.operationMode & 0x04) != 0;
	}
	break;
	case SMARTAUDIO_RSP_SET_POWER:
	{
    
		const U16ResponseFrame* respu16 = (const U16ResponseFrame*)buffer;
		const uint8_t power = respu16->payload & 0xFF;
		switch (sa_protocol_version)
		{
		case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2.1");
			pwr_Level = getPowerIndexFromDbm(power);
			break;
		case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 1");
			pwr_Level = getPowerIndexFromV1(power);
			break;
		default:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2");
			pwr_Level = power;
			break;
		}
	}
	break;
	case SMARTAUDIO_RSP_SET_CHANNEL:
	{
    DEBUG("sa_parse_response_buffer(), SetChannel");
		const U8ResponseFrame* respu8 = (const U8ResponseFrame*)buffer;
		uint16_t channel = respu8->payload;
		ch_index = channel;
		//debug("Channel was set to %d", resp->payload);
	}
	break;
	}
  return true;
}

//bool VTXControl::setPitMode(bool enabled)
//{
//  clearErrors();
//  switch (vtx_mode)
//  {
//  case VTXMode::SmartAudio: 
//    //DEBUG("setPitMode: SMA");
//    return sa_setPitMode(enabled);    
//  case VTXMode::Tramp: 
//    //DEBUG("setPitMode: Tramp");
//    return trampSetPitMode(enabled);    
//  }
//  return false;
//}

bool VTXControl::setPowerInmW(uint16_t pwrmW)
{
	bool res = false;
	clearErrors();
	for (int i = 0; i < _numtries; i++)//trying to set _numtries times
	{
		switch (vtx_mode)
		{
		case VTXMode::SmartAudio:
		{
      int pwrLevel = getPowerIndexFromMW(pwrmW);
      if (pwrLevel != -1)//pwr level(index) not found
      {
        if (sa_setPower(pwrLevel))
        {
          if (sa_readResponse())
          {
            res = true;
          }
        }
      }
		}
		break;
		case VTXMode::Tramp://tramp protocol sets power in mW
		{
			//tramp protocol needs to be initialized      
			if (!initialized)
			{
				trampInit();
			}
			//in milliWatts      
      if (initialized)
      {        
        res = trampSetPower(pwrmW);
      }
		}
		break;
		}
		//if procedure of setting is succesfull - check acvieved result
		if (res)
		{
      int currPwrmW = getPowerInmW(pwr_Level);
      if (currPwrmW == pwrmW)
				return true;
		}
	}
	return false;
}
bool VTXControl::setPower(int pwrLevel)
{
	if (pwrLevel >= 0 && pwrLevel < sizeof(powers))//check the index of power
	{
    bool res = false;
		clearErrors();
    for (int i = 0; i < _numtries; i++)//trying to set _numtries times
    {
			switch (vtx_mode)
			{
			case VTXMode::SmartAudio:
			{
				if (sa_setPower(pwrLevel))
				{
					if (sa_readResponse())
					{
						res = true;
					}
				}
			}
      break;
			case VTXMode::Tramp://tramp protocol sets power in mW
			{
				//tramp protocol needs to be initialized      
				if (!initialized)
				{
					trampInit();
				}
				//in milliWatts      
				if (initialized)
					res = trampSetPower(powers[pwrLevel]);				
			}
      break;
			}
      //if procedure of setting is succesfull - check acvieved result
			if (res && pwr_Level == pwrLevel)
			{
				return true;
			}
		}
	}
	return false;
}
bool VTXControl::setPrevChannel()
{
  int newCh = ch_index - 1;
  if (newCh < 0) newCh = sizeof(freqs)/sizeof(freqs[0]) - 1;
  return setChannel(newCh);
}
bool VTXControl::setNextChannel()
{
  int newCh = ch_index + 1;
  if (newCh >= sizeof(freqs) / sizeof(freqs[0])) newCh = 0;
  return setChannel(newCh);
}
uint16_t VTXControl::getPowerInmW(int pwrIndex)
{
  int pwrslen = sizeof(powers) / sizeof(powers[0]);  
  return (pwrIndex >= 0 && pwrIndex < pwrslen) ? powers [pwrIndex] : 0;
}
int VTXControl::getPowerIndexFromMW(uint16_t pwrInmW)
{
  int pwrslen = sizeof(powers) / sizeof(powers[0]);
  for (int i = 0; i < pwrslen; i++)
  { 
    if (powers[i] == pwrInmW) return i;
  }
  return -1;//not found
}
int VTXControl::getPowerIndexFromDbm(uint16_t pwrInDbm)
{
  int pwrslen = sizeof(powers_v21) / sizeof(powers_v21[0]);
  for (int i = 0; i < pwrslen; i++)
  {
    if (powers_v21[i] == pwrInDbm) return i;
  }
  return -1;//not found
}
int VTXControl::getPowerIndexFromV1(uint16_t pwrValue)
{
  int pwrslen = sizeof(powers_v1) / sizeof(powers_v1[0]);
  for (int i = 0; i < pwrslen; i++)
  {
    if (powers_v1[i] == pwrValue) return i;
  }
  return -1;//not found
}
uint16_t VTXControl::getChannelFrequency(int chIndex)
{
  int freqslen = sizeof(freqs) / sizeof(freqs[0]);
  return (chIndex >= 0 && chIndex < freqslen) ? freqs[chIndex] : 0;
}
int VTXControl::getChannelIndex(uint16_t freq)
{
  int freqslen = sizeof(freqs) / sizeof(freqs[0]);
  for (int i = 0; i < freqslen; i++)
  {
    if (freqs[i] == freq) return i;
  }
  return -1;//not found
}

bool VTXControl::setFrequency(uint16_t freq)
{  
	clearErrors();
	bool res = false;
	for (int i = 0; i < _numtries; i++)//trying to set _numtries times
	{
		switch (vtx_mode)
		{
		case VTXMode::SmartAudio:
		{
			//debug("Setting channel to %d", channel);
			//if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, chIndex))
      int chIndex = getChannelIndex(freq);
      if (chIndex != -1)//if frequency found in the table of freqs
      {
        if (sa_setChannel(chIndex))
        {
          res = sa_readResponse();
        }
      }
		}
		break;
		case VTXMode::Tramp://tramp protocol sets channel in Mhz
		{
			//tramp protocol needs to be initialized
			if (!initialized)
			{
				trampInit();
			}
			if (initialized)
				res = trampSetFrequency(freq);
		}
		break;
		}
		//if channel set succesfully - check the updated info from VTX
    if (res)
    {
      uint16_t curr_freq = getChannelFrequency(ch_index);
      if (curr_freq == freq)
         return true;
    }
	}
	return false;
}
// Sets the specified frequency to the VTX
bool VTXControl::setChannel(int chIndex)
{
  if (chIndex >= 0 && chIndex < sizeof(freqs) / sizeof(freqs[0]))
  {
    clearErrors();
    bool res = false;
    for (int i = 0; i < _numtries; i++)//trying to set _numtries times
    {
      switch (vtx_mode)
      {
      case VTXMode::SmartAudio:
      {
        //debug("Setting channel to %d", channel);
        //if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, chIndex))
        if (sa_setChannel(chIndex))
        {
          res = sa_readResponse();
        }
      }
      break;
      case VTXMode::Tramp://tramp protocol sets channel in Mhz
      {
        //tramp protocol needs to be initialized
        if (!initialized)
        {
          trampInit();
        }
        if (initialized)
          res = trampSetFrequency(freqs[chIndex]);
      }
      break;
      }
      //if channel set succesfully - check the updated info from VTX
      if (res && chIndex == ch_index)
        return true;
    }
  }
  return false;
}

bool VTXControl::updateParameters()
{
	//port->stopListening();
	switch (vtx_mode)
	{
	case VTXMode::SmartAudio:
		//DEBUG("updateParameters: SMA"); waitForInMs(200);
		return sa_updateSettings();
	case VTXMode::Tramp://tramp protocol sets channel in Mhz	
		//DEBUG("updateParameters: Tramp"); waitForInMs(200);   		
		return trampUpdate();
	}
	return false;
}

bool VTXControl::trampSendPacket(uint8_t * packet, bool respRequired)//(trampFrame_t* packet)
{    
  //DEBUG("trampSendPacket, before trampPush");  
  if (trampPush(packet))
  {        
    //with setPower and setChannel we don't wait a response, 
    //in this case we send UpdateParameters(GetConfig) to get new parameters
    bool res = respRequired ? trampReadResponse() : true;
    DEBUG("trampSendPacket, After ReadResponse, res="+(String)res);
    return res;
  }
  DEBUG("trampSendPacket, return false");
  return false;
}
bool VTXControl::trampPush(const uint8_t* packet)//const trampFrame_t* object)
{  
  //port->writeDummyByte();//to get port low before sending command
  //port->write((uint8_t)0x00);  
  //bool res = port->write((uint8_t*)&object, sizeof(trampFrame_t)) == sizeof(trampFrame_t);
  bool res = port->write(packet, TRAMP_FRAME_LENGTH) == TRAMP_FRAME_LENGTH;  
  //port->write((uint8_t)0x00);
  
  port->listen();//wait for the response
  //DEBUG("trampPush, written "+(String)sizeof(trampFrame_t) +" bytes");
  DEBUG("trampPush, written " + (String)TRAMP_FRAME_LENGTH + " bytes");
  DEBUG("trampPush, Res= " + (String)res);
#if VTXCDEBUG
  //dumpBuffer((uint8_t*)&object, sizeof(trampFrame_t));  
  dumpBuffer(packet, TRAMP_FRAME_LENGTH);
#endif
  return res;
}

bool VTXControl::trampUpdate()
{
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("TrampUpdate");
  while (1)//speed cycle
  {
    //trying several times
    for (int i = 0; i < _numtries; i++)
    {
      //tramp protocol needs to be initialized
      if (!initialized)
      {
        trampInit();
        DEBUG("TrampUpdate, Initialized:" + (String)initialized);
      }
      if (initialized)
      {
        if (trampGetStatus())
        {                    
					return true;
        }
      }
    }
    if (_smartBaudRate)
    {
      //so we need to change baud rate?
      newspeed = trampOfferNewSpeed(newspeed);
      if (newspeed != startspeed)
      {
        DEBUG("TrampUpdate, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      }
      else
      {
        DEBUG("TrampUpdate, end of tries");
        return false;//out of change speed cycle
      }
    }
    else break;
  }
  return false;
}
long VTXControl::trampOfferNewSpeed(long currentSpeed)
{
  //if speed == AP_SMARTAUDIO_UART_BAUD we're going to AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to AP_SMARTAUDIO_SMARTBAUD_MAX
  //if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to AP_SMARTAUDIO_UART_BAUD
  //so AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed)
  {
  case AP_TRAMP_UART_BAUD: return AP_TRAMP_UART_BAUD + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MIN: return AP_TRAMP_UART_BAUD_MIN + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MAX: return AP_TRAMP_UART_BAUD_MIN;
  }
  //if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  // ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_TRAMP_UART_BAUD && currentSpeed < AP_TRAMP_UART_BAUD_MAX) ||
    (currentSpeed > AP_TRAMP_UART_BAUD_MIN && currentSpeed < AP_TRAMP_UART_BAUD))
    return currentSpeed + AP_TRAMP_SMARTBAUD_STEP;
  if (currentSpeed > AP_TRAMP_UART_BAUD_MAX) return AP_TRAMP_UART_BAUD_MAX;
  if (currentSpeed < AP_TRAMP_UART_BAUD_MIN) return AP_TRAMP_UART_BAUD_MIN;
}
bool VTXControl::trampSendCmd(uint8_t cmd)
{
  /*uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;    
  buf[14] = trampCrc(buf);  
  buf[15] = TRAMP_SYNC_STOP;
  bool res = trampSendPacket(buf);*/
  return trampSendCmd(cmd, 0);
}
bool VTXControl::trampSendCmd(uint8_t cmd, uint16_t param)
{
  uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;
  if (param != 0)
  {
    buf[2] = param & 0xff;
    buf[3] = (param >> 8) & 0xff;
  }
  buf[14] = trampCrc(buf);
  buf[15] = TRAMP_SYNC_STOP;
  //we need a reponse for certain commands only
  //so calling UpdateParameters required after send command
  bool respRequired = cmd == TRAMP_COMMAND_GET_CONFIG || cmd == TRAMP_COMMAND_CMD_RF || TRAMP_COMMAND_CMD_SENSOR;
  return trampSendPacket(buf, respRequired);
}
bool VTXControl::trampInit()
{
  DEBUG("TrampInit:");
 /* trampFrame_t frame;
  trampFrameInit(TRAMP_COMMAND_CMD_RF, &frame);
  trampFrameClose(&frame);
  bool res = trampSendPacket((uint8_t*)&frame);*/
  
  //different version of sending  
  if (trampSendCmd(TRAMP_COMMAND_CMD_RF))
  { 
    DEBUG("TrampInit: Inititalized Successfully");
    initialized = true;
    return true;
  }  
	DEBUG("TrampInit: Not Inititalized");
	setError(VTXErrors::vtxtrampNotInited);
	return false;
}
bool VTXControl::trampGetStatus()
{
  //trampFrame_t frame;
  //trampFrameInit(TRAMP_COMMAND_GET_CONFIG, &frame);
  //trampFrameClose(&frame);  
  //return trampSendPacket((uint8_t*)&frame);
  //different version of sending    
  bool res =  trampSendCmd(TRAMP_COMMAND_GET_CONFIG);
  DEBUG("trampGetStatus, res="+(String)res);
  return res;
}

bool VTXControl::trampSetFrequency(uint16_t freq)
{
  //trampFrame_t frame;
  //trampFrameInit(TRAMP_COMMAND_SET_FREQ, &frame);
  //frame.payload.frequency = freq;
  //trampFrameClose(&frame);  
  //return trampSendPacket((uint8_t*)&frame);
  return trampSendCmd(TRAMP_COMMAND_SET_FREQ, freq);
}
bool VTXControl::trampSetPower(uint16_t milliWatts)
{
  //trampFrame_t frame;
  //trampFrameInit(TRAMP_COMMAND_SET_POWER, &frame);
  //frame.payload.power = milliWatts;
  //trampFrameClose(&frame);  
  //return trampSendPacket((uint8_t*)&frame);
  return trampSendCmd(TRAMP_COMMAND_SET_POWER, milliWatts);
}

bool VTXControl::trampReadResponse()
{
  // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the request
  // and the initial response is 40ms long so we should wait at least 140ms before giving up
  //waitForInMs(_responseTimeOut);
  //unsigned long currentTime = millis();  
  ////wait to receive full packet  
  unsigned long currentTime = 0;
  while (port->available() == 0)
  {
    if (currentTime > _responseTimeOut)
    {
      //DEBUG("trampReadResponse, time out of waiting response");
      break;// return false;
    }
    waitForInMs(100);
    currentTime += 100;
  }
  
  //DEBUG("trampReadResponse, before fill rx buf");
  int16_t incoming_bytes_count = port->available();
  if (incoming_bytes_count == 0)
  {    
    DEBUG("trampReadResponse, incoming bytes=0, return false");
    setError(VTXErrors::vtxIncomingBytesZero);
    return false;
  }
  //now we have no-zero length response, we can dump it
#if VTXCDEBUG
  DEBUG("trampReadResponse: Dump Received:");
  port->dumpReceiveBuffer();
#endif
  if (incoming_bytes_count == TRAMP_FRAME_LENGTH)
  {
    uint8_t tramp_rx_buf[TRAMP_FRAME_LENGTH];
    //fill out the buffer
    for (int i = 0; i < TRAMP_FRAME_LENGTH; i++)
    {
      uint8_t d = port->read();
      tramp_rx_buf[i] = d;
    }
    // Buffer is full, calculate checksum
    const trampFrame_t* frame = (const trampFrame_t*)tramp_rx_buf;
    if (frame->header.syncStart != TRAMP_SYNC_START)
    {
      DEBUG("trampReadResponse, first header byte not equal SyncByte");
      setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
      return false;
    }
    if (frame->footer.syncStop != TRAMP_SYNC_STOP)
    {
      DEBUG("trampReadResponse, last byte not equal Sync Stop");
      setError(VTXErrors::vtxLastByteNotSyncStop);
      return false;
    }
    //const uint8_t crc = trampCrc(frame);
    const uint8_t crc = trampCrc(tramp_rx_buf);
    if (crc != frame->footer.crc) 
    {
      DEBUG("trampReadResponse, Incorrect checksum");
      setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
      return false;
    }    
    DEBUG("trampReadResponse: Update Data");
    //TODO:here we need to update data dependently of response code (cmd)
    switch (frame->header.command) 
    {
    case TRAMP_COMMAND_CMD_RF://'r', Init
    {
      //we ignore this response, just parse
      DEBUG("trampReadResponse: Response from Init: do nothing");
      break;
    }
    case TRAMP_COMMAND_GET_CONFIG://'v' //we update data for current settings just sending TRAMP_COMMAND_GET_CONFIG and reading request
    {
      DEBUG("trampReadResponse: Response from Get_CONFIG: Update data");
      const uint16_t freq = frame->payload.settings.frequency;
      // Check we're not reading the request (indicated by freq zero)
      if (freq != 0)
      {
        //update data
        ch_index = getChannelIndex(frame->payload.settings.frequency);
        pwr_Level = getPowerIndexFromMW(frame->payload.settings.power);
        pitMode = (bool)frame->payload.settings.pitModeEnabled;        
        DEBUG("trampReadResponse: Updated, Freq:" + (String)frame->payload.settings.frequency);
        DEBUG("trampReadResponse: Converted to Channel:" + (String)ch_index);
        DEBUG("trampReadResponse: Updated, Power in Mw:" + (String)frame->payload.settings.power);
        DEBUG("trampReadResponse: Converted to Power Level:" + (String)pwr_Level);
        DEBUG("trampReadResponse: Updated, PitMode:" + (String)frame->payload.settings.pitModeEnabled);
      }
      break;
    }
    case TRAMP_COMMAND_CMD_SENSOR://'s' - temperature sensor, currently not used
    {
      //currently we ignore this response, just parse
      DEBUG("trampReadResponse: Response from CMD_Sensor: do nothing");
      break;
    }
    }     
    //clear port
    port->flush();    
    // successful response, wait another 100ms to give the VTX a chance to recover
    // before sending another command.
    delay(100);//çäåñü ìîæíî èñïîëüçîâàòü delay
    DEBUG("trampReadResponse:Succesfull");
    return true;
  }
  else
  {
    DEBUG("trampReadResponse2, incoming_bytes_count != TRAMP_FRAME_LENGTH");
    setError(VTXErrors::vtxBufferLengthLessWholePacket);
  }
  //clear port
  port->flush();
  return false;
}

