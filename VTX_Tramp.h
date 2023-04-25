#ifndef VTX_TRAMP_H_
#define VTX_TRAMP_H_

#include "Arduino.h"

#define AP_TRAMP_UART_CFG             801 //8 databits, 1 start bit, 1 stop bit
#define AP_TRAMP_UART_BAUD            9600
#define AP_TRAMP_UART_BAUD_MIN            9120 //-5%
#define AP_TRAMP_UART_BAUD_MAX            10800 //+5%
#define AP_TRAMP_SMARTBAUD_STEP       60
// request and response size is 16 bytes
#define AP_TRAMP_UART_BUFSIZE_RX      32
#define AP_TRAMP_UART_BUFSIZE_TX      32

// Define periods between requests
#define TRAMP_MIN_REQUEST_PERIOD_US (200 * 1000) // 200ms
#define TRAMP_STATUS_REQUEST_PERIOD_US (1000 * 1000) // 1s

#define TRAMP_PAYLOAD_LENGTH    12

typedef struct trampSettings_s {
  uint16_t frequency;
  uint16_t power;
  uint8_t raceModeEnabled;
  uint8_t pitModeEnabled;
} __attribute__((packed)) trampSettings_t;

typedef struct trampFrameHeader_s {
  uint8_t syncStart;
  uint8_t command;
} __attribute__((packed)) trampFrameHeader_t;

#define TRAMP_HEADER_LENGTH sizeof(trampFrameHeader_t)

typedef struct trampFrameFooter_s {
  uint8_t crc;
  uint8_t syncStop;
} __attribute__((packed)) trampFrameFooter_t;

typedef union trampPayload_u {
  uint8_t buf[TRAMP_PAYLOAD_LENGTH];
  trampSettings_t settings;
  uint16_t frequency;
  uint16_t power;
  uint8_t active;
} trampPayload_t;

typedef struct trampFrame_s {
  trampFrameHeader_t header;
  trampPayload_t payload;
  trampFrameFooter_t footer;
} __attribute__((packed)) trampFrame_t;

#define TRAMP_FRAME_LENGTH sizeof(trampFrame_t)

#define TRAMP_SYNC_START            0x0F
#define TRAMP_SYNC_STOP             0x00
#define TRAMP_COMMAND_SET_FREQ      'F' // 0x46, command. not require response, down't wait a response
#define TRAMP_COMMAND_SET_POWER     'P' // 0x50, command. not require response, down't wait a response
#define TRAMP_COMMAND_ACTIVE_STATE  'I' // 0x49, command, according to ardupilot code, thats set pitmode
#define TRAMP_COMMAND_GET_CONFIG    'v' // 0x76, request(require response, wait for a response)
#define TRAMP_COMMAND_CMD_RF        'r' // 0x72, request(require response, wait for a response), activate/deactivate Tramp protocol
#define TRAMP_COMMAND_CMD_SENSOR    's' // 0x73, request(require response, wait for a response), gets current temperature of VTX

static uint8_t trampCrc(const uint8_t* frame)//const trampFrame_t* frame)
{
  //---------this is BetaFlight version, this version doesn't work
  //uint8_t crc = 0;
  //const uint8_t* p = frame;// (const uint8_t*)frame;
  //const uint8_t* pEnd = p + (TRAMP_HEADER_LENGTH + TRAMP_PAYLOAD_LENGTH);
  //for (; p != pEnd; p++) 
  //{
  //  crc += *p;
  //}
  //return crc;
  //---------this is arduPilot version, this version works
  uint8_t cksum = 0;
  for (int i = 1; i < TRAMP_FRAME_LENGTH - 2; i++) 
  {
    cksum += frame[i];
  }

  return cksum;
}
static void trampFrameInit(uint8_t frameType, trampFrame_t* frame)
{
  frame->header.syncStart = TRAMP_SYNC_START;
  frame->header.command = frameType;
  const uint8_t emptyPayload[TRAMP_PAYLOAD_LENGTH] = { 0 };
  memcpy(frame->payload.buf, emptyPayload, sizeof(frame->payload.buf));
}

static void trampFrameClose(trampFrame_t* frame)
{
  frame->footer.crc = trampCrc((const uint8_t*)frame);
  frame->footer.syncStop = TRAMP_SYNC_STOP;
}


#endif