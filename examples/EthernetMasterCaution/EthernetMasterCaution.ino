/*
    Name:				EthernetMasterCaution.ino
    Created:			02/05/2023 18:09:00
    Author:				Wiggles5289
	THIS IS AN ALPHA CODE

	This example file shows an example setup of the Master Caution Button & Indicator Light with an arduino connected
	using an Ethernet Shield. Utilizes the vanilla Ethernet Library preinstalled on the Arduino IDE. Supports only
	W5100/W5200/W5500 Chipset from Wiznet.io. Define the port 

	If you have any questions or queries, send them through the DCS Flight Panels/DCS_BIOS team who can direct your 
	questions to the right people.

	THIS IS AN ALPHA CODE
*/
#define DCSBIOS_ETHERNET
#include "DcsBios.h"

/* Declare a Master Caution Reset button on pin 10 */
DcsBios::Switch2Pos ufcMasterCaution("UFC_MASTER_CAUTION", 10);

/* Make the LED connected to pin 13 into a Master Caution Light */
DcsBios::LED masterCaution(0x1012, 0x0800, 13);

void setup() {
  DcsBios::setup();
}

void loop() {
  DcsBios::loop();
}
