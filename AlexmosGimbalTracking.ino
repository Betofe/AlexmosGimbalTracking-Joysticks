/*
 Name:		AlexmosGimbalTracking.ino
 Created:	7/5/2023 10:09:05 AM
 Author:	Imami Joel Betofe
*/
#include "Globals.h"
#include "MavlinkSettings.h"
#include "GimbalSettings.h"

Gimbal gimbal;
MavlinkConnection mavlink;
// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(DEBUG_BAUD_RATE);
	gimbal.init();
	mavlink.init();
	
}

// the loop function runs over and over again until power down or reset
void loop() {
	gimbal.run();
	//gimbal.SendGpsCord();

	
}
