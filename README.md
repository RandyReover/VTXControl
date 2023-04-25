# VTXControl
Arduino library providing video transmitter (VTX) control by SmartAudio/Tramp protocol.
This C/C++ code uses modified SoftwareSerialWithHalfDuplex library (part code taken from CustomSoftwareSerial to support different configuration of serial port (especially 8N2)), some code taken from BetaFlight and ArduPilot (SmartAudio and Tramp protocols support) code.
This code created to use features of Tramp/SmartAudio on VTX (like switching power modes and channels/frequencies) by code/wire on Arduino driven systems or robots, 
VTXControl works in two modes/protocols - SmartAudio and Tramp, communications with VTX established by software serial port (SoftwareSerialWithHalfDuplex).
This code had been tested on Eachine TX5258 (SmartAudio v2 protocol) and JHEMCU RuiBet Tran3016W (Tramp protocol) VTXes, so that's not an universal solution for your own VTX (just because your VTX can have manufacturer's own Tramp/SmartAudio protocol implementation).
A simplest creation of VTXControl instance example:
//for SmartAudio mode
VTXControl* vtx;
vtx = new VTXControl(VTXMode::SmartAudio, 53, 500, false);
or
//for Tramp mode
VTXControl* vtx;
vtx = new VTXControl(VTXMode::Tramp, 53, 500, false);
Sorry there is no support or help with this code, use it on your own.
