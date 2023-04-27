# VTXControl
Arduino library providing video transmitter (VTX) control by SmartAudio/Tramp protocol.

This C/C++ code uses modified SoftwareSerialWithHalfDuplex library (part code taken from CustomSoftwareSerial to support different configuration of serial port (especially 8N2)), some code taken from BetaFlight and ArduPilot (SmartAudio and Tramp protocols support) code.

This code created to use features of Tramp/SmartAudio on VTX (like switching power modes and channels/frequencies) by code/wire on Arduino driven systems or robots.

VTXControl works in two modes/protocols - SmartAudio and Tramp, communications with VTX established by software serial port (SoftwareSerialWithHalfDuplex).

This code had been tested on Eachine TX5258 (SmartAudio v2 protocol) and JHEMCU RuiBet Tran3016W (Tramp protocol) VTXes, so that's not an universal solution for your own VTX (just because your VTX can have manufacturer's own Tramp/SmartAudio protocol implementation).

This code doesn't support PitMode and GetTemperature features, but you can add these features on your own.

Please review **powers** and **freqs** arrays definitions in VTXControl.cpp to apply/modify values provided by your VTX.

A simplest creation of VTXControl instance example:

//for SmartAudio mode

VTXControl* vtx;

vtx = new VTXControl(VTXMode::SmartAudio, 53, 500, false);

or

//for Tramp mode

VTXControl* vtx;

vtx = new VTXControl(VTXMode::Tramp, 53, 500, false);

Sorry there is no support or help with this code, use it on your own.

NB: The main problem of interacting VTXControl with different VTXs is difference of SmartAudio/Tramp protocols. In other words, as BetaFlight/ArduPilot authors said, "because of poor implementation" of SmartAudio/Tramp by VTX manufacturers. So please use VTX_Test project to diagnose your own VTX - send commands, analyze responses, try different baudrates, try to add zero or not bytes to the end/start of standard frames/packets, ignore or not crc to achieve correct results with your own VTX.

**NB1: To VTX owners with >800mW power, SmartAudio v1 or v2.1 protocol: you can use following lifehack to create/modify table of powers (coded powers, dBm's): just set different levels of power on your VTX by button and catch updateParameters command response from VTX by VTX_Test project to get coded power level or dBm value for the current level of power.**
