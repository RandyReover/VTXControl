#include <Arduino.h>
#include <VTXControl.h>
#include <SoftwareSerialWithHalfDuplex.h>

VTXControl* vtx;
bool vtx_updated = false;

void setup()
{
  Serial.begin(115200);//
  //create VTXCOntrol in SmartAudio/Tramp protocol mode, 
  //on 53 port, with response time(awaiting for response) 1400ms, without smart-bauding(find/choose appropriate baudrate when VTX turns to be responsible on queries)
  
  //Urgent comment for SoftwareSerialWithHalfDuplex:
  //The pin should support change interrupts :
  //-On the Mega and Mega 2560 only the following can be used for RX : 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8(62), A9(63), A10(64), A11(65), A12(66), A13(67), A14(68), A15(69).
  //-On the Leonardo and Micro only the following can be used for RX : 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

  //vtx = new VTXControl(VTXMode::SmartAudio, 53, 1400, false);
  vtx = new VTXControl(VTXMode::Tramp, 53, 500, false);
  vtx->waitForInMs(1000);
}
void loop()
{
  if (!vtx_updated)
  {
    Serial.println(F("-------------------------------------"));
    //for robo - here we refresh parameters of VTX
    if (vtx->updateParameters())
    {
      vtx_updated = true;//succesfully updated
      PrintSettings();
      Serial.println(F("--Set Channel-------------------------------"));
      if (vtx->setChannel(13))
      {
        Serial.println(F("Channel succesfully set"));
      }
      Serial.println(F("--Set Power-------------------------------"));
      if (vtx->setPower(0))
      {
        Serial.println(F("Power succesfully set"));
      }
      if (vtx->updateParameters())
      {
        Serial.println(F("----------New parameters------"));
      PrintSettings();
      }
    }
    else
    {
      PrintErrors();
    }
    vtx->clearErrors();
    vtx->flush();
  }
}
void PrintSettings()
{
  Serial.println("Current Power Level:" + (String)vtx->getPowerLevel());
  Serial.println("Current Channel:" + (String)vtx->getChannelIndex());
  Serial.println("Current PitMode:" + (String)vtx->getPitMode());
}
void PrintErrors()
{
  VTXErrors errs = vtx->getErrors();

  if (errs == VTXErrors::vtxNoErrors)
    Serial.println(F("VTX Errors: NoErrors"));
  else
  {
    if ((errs & VTXErrors::vtxParseResponseInvalidCRCOrBuffer) != 0)
      Serial.println(F("VTX Errors: ParseResponseInvalidCRCOrBuffer"));
    if ((errs & VTXErrors::vtxIncomingBytesZero) != 0)
      Serial.println(F("VTX Errors: IncomingBytesZero"));
    if ((errs & VTXErrors::vtxIncomingBytesLessHeaderSize) != 0)
      Serial.println(F("VTX Errors: IncomingBytesLessHeaderSize"));
    if ((errs & VTXErrors::vtxIncomingByteNotEqualSyncByte) != 0)
      Serial.println(F("VTX Errors: IncomingByteNotEqualSyncByte"));
    if ((errs & VTXErrors::vtxIncomingByteNotEqualHeaderByte) != 0)
      Serial.println(F("VTX Errors: IncomingByteNotEqualHeaderByte"));
    if ((errs & VTXErrors::vtxPacketSizeGreaterMaxPacketSize) != 0)
      Serial.println(F("VTX Errors: PacketSizeGreaterMaxPacketSize"));
    if ((errs & VTXErrors::vtxBufferLengthLessWholePacket) != 0)
      Serial.println(F("VTX Errors: BufferLengthLessWholePacket"));
    if ((errs & VTXErrors::vtxportIsNotListening) != 0)
      Serial.println(F("VTX Errors: Port IsNotListening"));
    if ((errs & VTXErrors::vtxportBufferIsEmpty) != 0)
      Serial.println(F("VTX Errors: Port BufferIsEmpty"));
    if ((errs & VTXErrors::vtxportTxDelayIsZero) != 0)
      Serial.println(F("VTX Errors: Port TxDelayIsZero"));
    if ((errs & VTXErrors::vtxportRXDelayStopBitNotSet) != 0)
      Serial.println(F("VTX Errors: Port RXDelayStopBitNotSet"));
    if ((errs & VTXErrors::vtxtrampNotInited) != 0)
      Serial.println(F("VTX Errors: Tramp VTX Device not initialized"));
  }
}
void dump_byte_array(byte *buffer, byte bufferSize)
{
  for (byte i = 0; i < bufferSize; i++)
  {
    Serial.print((uint8_t)buffer[i] < 0x10 ? " 0" : " ");
    Serial.print((uint8_t)buffer[i], HEX);
  }
}
//relatively precise function as replacement of delay
//delay stop processor/interrupts usage, so we need to use delayMcroseconds
//but delayMicroseconds uses max 16384 value
void waitForInMs(unsigned int ms)
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
}
