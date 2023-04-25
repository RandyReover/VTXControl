#ifndef VTX_SMARTAUDIO_H_
#define VTX_SMARTAUDIO_H_

#define SMARTAUDIO_IGNORE_CRC true //ignores crc calculation and verification
//#define SMARTAUDIO_SKIP_STARTDUMMYBYTES_IN_RESPONSE true //special setting for Eachine TX5258 - we receive first two 0x00 bytes and additional byte 0x00 in the end
#define SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END true //special setting for Eachine TX5258 (betaflight uses it for Smartaudio non-licensed clones) - we need to write additional zero byte at the end of command
#define SMARTAUDIO_BUFFER_CAPACITY 5

// SmartAudio Serial Protocol
#define AP_SMARTAUDIO_UART_CFG             802 //8 databits, 1 start bit, 2 stop bit
#define AP_SMARTAUDIO_UART_BAUD            4800
#define AP_SMARTAUDIO_SMARTBAUD_MIN        4560     // -5%
#define AP_SMARTAUDIO_SMARTBAUD_ARDU       4900     // this baud rate used by nighthawk  why not to try?
#define AP_SMARTAUDIO_SMARTBAUD_MAX        5040     // +5%
#define AP_SMARTAUDIO_SMARTBAUD_STEP       60
#define AP_SMARTAUDIO_UART_BUFSIZE_RX      16
#define AP_SMARTAUDIO_UART_BUFSIZE_TX      16
#define AP_SMARTAUDIO_MAX_PACKET_SIZE      32

#define SMARTAUDIO_SYNC_BYTE            0xAA
#define SMARTAUDIO_HEADER_BYTE          0x55
#define SMARTAUDIO_START_CODE           SMARTAUDIO_SYNC_BYTE + SMARTAUDIO_HEADER_BYTE
#define SMARTAUDIO_GET_PITMODE_FREQ     (1 << 14)
#define SMARTAUDIO_SET_PITMODE_FREQ     (1 << 15)
#define SMARTAUDIO_FREQUENCY_MASK       0x3FFF

#define SMARTAUDIO_CMD_GET_SETTINGS     0x03 //
#define SMARTAUDIO_CMD_SET_POWER        0x05
#define SMARTAUDIO_CMD_SET_CHANNEL      0x07
#define SMARTAUDIO_CMD_SET_FREQUENCY    0x09
#define SMARTAUDIO_CMD_SET_MODE         0x0B

#define SMARTAUDIO_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 1)
#define SMARTAUDIO_U8_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 2)
#define SMARTAUDIO_U16_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 3)

#define SMARTAUDIO_RSP_GET_SETTINGS_V1  SMARTAUDIO_CMD_GET_SETTINGS >> 1
#define SMARTAUDIO_RSP_GET_SETTINGS_V2  (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x08 //0x09
#define SMARTAUDIO_RSP_GET_SETTINGS_V21 (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x10 //0x11
#define SMARTAUDIO_RSP_SET_POWER        SMARTAUDIO_CMD_SET_POWER >> 1
#define SMARTAUDIO_RSP_SET_CHANNEL      SMARTAUDIO_CMD_SET_CHANNEL >> 1
#define SMARTAUDIO_RSP_SET_FREQUENCY    SMARTAUDIO_CMD_SET_FREQUENCY >> 1
#define SMARTAUDIO_RSP_SET_MODE         SMARTAUDIO_CMD_SET_MODE >> 1

// crc8 from betaflight
//
// CRC8 computations
//
// crc8 from betaflight crc8_dvb_s2_update
#define POLYGEN 0xd5
static uint8_t sa_CRC8(const void* data, uint32_t length)//crc8_dvb_s2_update
{
  uint8_t crc = 0;
  const uint8_t* p = (const uint8_t*)data;
  const uint8_t* pend = p + length;

  for (; p != pend; p++) 
  {
    //crc = crc8_dvb(crc, *p, 0xD5);
    crc ^= *p;
    for (uint8_t i = 0; i < 8; ++i)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ POLYGEN;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

enum ProtocolVersion {
  SMARTAUDIO_SPEC_PROTOCOL_v1 = 0,
  SMARTAUDIO_SPEC_PROTOCOL_v2 = 1,
  SMARTAUDIO_SPEC_PROTOCOL_v21 = 2
};

struct Settings 
{
  uint8_t  version;
  uint8_t  mode;
  uint8_t  channel;
  uint8_t  power;
  uint16_t frequency;
  uint8_t  band;

  uint8_t num_power_levels;
  uint8_t power_levels[8];
  uint8_t  power_in_dbm;

  uint16_t pitmodeFrequency;
  bool userFrequencyMode;     // user is setting freq
  bool initialized;
};

struct FrameHeader 
{
  uint8_t syncByte;
  uint8_t headerByte;
  uint8_t command;//or version
  uint8_t length;

  void init(uint8_t cmd, uint8_t payloadLength)
  {
    syncByte = SMARTAUDIO_SYNC_BYTE;
    headerByte = SMARTAUDIO_HEADER_BYTE;
    length = payloadLength;
    command = cmd;
  }
} /*PACKED*/__attribute__((packed));

struct Frame 
{
  FrameHeader header;
  uint8_t payload[3];
} /*PACKED*/__attribute__((packed));

struct U8ResponseFrame 
{
  FrameHeader header;
  uint8_t payload;
  //uint8_t reserved;
  //uint8_t crc;
} /*PACKED*/__attribute__((packed));

struct U16ResponseFrame 
{
  FrameHeader header;
  uint16_t payload;
  //uint8_t reserved;
  //uint8_t crc;
} /*PACKED*/__attribute__((packed));

struct SettingsResponseFrame 
{
  FrameHeader header;
  uint8_t channel;
  uint8_t power;
  uint8_t operationMode;
  uint16_t frequency;
  uint8_t crc;
} /*PACKED*/__attribute__((packed));

struct SettingsExtendedResponseFrame 
{
  SettingsResponseFrame settings;
  uint8_t power_dbm;  // current power
  uint8_t num_power_levels;
  uint8_t power_levels[8];   // first in the list of dbm levels
  //uint8_t crc;
} /*PACKED*/__attribute__((packed));

// v 2.1 additions to response frame
//0x0E (current power in dBm) 0x03 (amount of power levels) 0x00(dBm level 1) 0x0E (dBm level 2) 0x14 (dBm level 3) 0x1A (dBm level 4) 0x01(CRC8)

// request packet to be processed
struct Packet 
{
  Frame frame;
  uint8_t frame_size;
} /*PACKED*/__attribute__((packed));
#endif