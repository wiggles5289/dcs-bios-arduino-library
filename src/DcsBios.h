/* PROOF OF CONCEPT AND HARDWARE TEST Written by THE_ONE & Wiggles5289 on DCSFP Discord
Adds Ethernet functionality to DCSBIOS with Ethernet Library found in default Arduino IDE.
Only W5100/W5200/W5500 Chipsets supported by library
*/ 

#ifndef __DCSBIOS_H
#define __DCSBIOS_H

#ifndef NULL
#define NULL 0
#endif

#include <stdint.h>

#ifdef DCSBIOS_FOR_STM32
#include <itoa.h>
#endif

#include "internal/ExportStreamListener.h"
#include "internal/PollingInput.h"
#include "internal/Protocol.h"

#ifndef USART0_RX_vect
#define USART0_RX_vect USART_RX_vect
#define USART0_TX_vect USART_TX_vect
#define USART0_UDRE_vect USART_UDRE_vect
#endif

#ifndef PRR0
#define PRR0 PRR
#endif

namespace DcsBios {
	const unsigned char PIN_NC = 0xFF;
}

/*
The following is an ugly hack to work with the Arduino IDE's build system.
The DCS-BIOS Arduino Library is configured with #defines such as DCSBIOS_RS485_MASTER or DCSBIOS_RS485_SLAVE <address>.
To make sure these defines are visible when compiling the code, we can't put it into a separate translation unit.

Normally, those #defines would go in a separate "config.h" or you would use compiler flags. But since Arduino libraries
do not come with their own build system, we are just putting everything into the header file.
*/
#ifdef DCSBIOS_RS485_MASTER
	#include "internal/DcsBiosNgRS485Master.h"
	#include "internal/DcsBiosNgRS485Master.cpp.inc"
#endif
#ifdef DCSBIOS_RS485_SLAVE
	#include "internal/DcsBiosNgRS485Slave.h"
	#include "internal/DcsBiosNgRS485Slave.cpp.inc"
#endif
#ifdef DCSBIOS_IRQ_SERIAL

	namespace DcsBios {
		ProtocolParser parser;

		ISR(USART0_RX_vect) {
			volatile uint8_t c = UDR0;
			parser.processCharISR(c);
		}
		
		void setup() {
			PRR0 &= ~(1<<PRUSART0);
			UBRR0H = 0;
			UBRR0L = 3; // 250000 bps
			UCSR0A = 0;
			UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
			
			UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
		}
		
		void loop() {
			PollingInput::pollInputs();
			ExportStreamListener::loopAll();
		}

		void resetAllStates() {
			PollingInput::resetAllStates();
		}

		static void usart_tx(const char* str) {
			const char* c = str;
			while (*c) {
				while(!(UCSR0A & (1<<UDRE0))); // wait until TX buffer is empty
				UDR0 = *c++; // write byte to TX buffer
			}
		}
		
		bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
			DcsBios::usart_tx(msg);
			DcsBios::usart_tx(" ");
			DcsBios::usart_tx(arg);
			DcsBios::usart_tx("\n");
			DcsBios::PollingInput::setMessageSentOrQueued();
			return true;
		}
	}
#endif
#ifdef DCSBIOS_DEFAULT_SERIAL
	namespace DcsBios {
		ProtocolParser parser;
		void setup() {
			Serial.begin(250000);
		}
		void loop() {
			while (Serial.available()) {
				parser.processChar(Serial.read());
			}
			PollingInput::pollInputs();
			ExportStreamListener::loopAll();			
		}
		bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
			Serial.write(msg); Serial.write(' '); Serial.write(arg); Serial.write('\n');
			DcsBios::PollingInput::setMessageSentOrQueued();
			return true;
		}
		void resetAllStates() {
			PollingInput::resetAllStates();
		}
	}
#endif
#ifdef DCSBIOS_ETHERNET
	#include <Ethernet.h> //Vanilla Ethernet Library from included ArduinoIDE
						  //Only WIZnet W5100/W5200/W5500-based devices are supported
	#include <EthernetUDP.h>
	#ifndef MACADDRESS
	byte mac[] = {  //NEEDS TO HAVE A FUNCTION TO CHANGE MAC FOR EVERY UPLOAD OR DEFINED IN MAIN -Wiggles5289
  		0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
		};
	#endif
	EthernetUDP udp;
	#ifndef ETHCS
	const int ETHCS = 10; //Default CS for most shields. Can be defined in sketch.
	#endif

	IPAddress ipmulti();
	IPAddress remote_IP;

	unsigned int port=5010;
	unsigned int dcs_port=7778;

	uint8_t dataUdP[128];

	namespace DcsBios {
		
		//unsigned int trans = 0;
		ProtocolParser parser;
		
		void setup() {	
		  Ethernet.init(ETHCS); //Ethernet Chip SPI CS PIN, if not defined, will default to Pin 10.
		  Ethernet.begin(mac); //Begins Chip Communication and Ethernet Protocol, gets IP from DHCP.
		  
		  /*			-----Quick HW Detection Debug-----
		    WIP: TODO
		  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    			Serial.println("Ethernet shield was not found.");
  			} else if (Ethernet.hardwareStatus() == EthernetW5100) {
    			Serial.println("W5100 Ethernet controller detected.");
  			} else if (Ethernet.hardwareStatus() == EthernetW5200) {
    			Serial.println("W5200 Ethernet controller detected.");
  			} else if (Ethernet.hardwareStatus() == EthernetW5500) {
    			Serial.println("W5500 Ethernet controller detected.");
  		   }
						-----Quick DHCP and IP Check-----
			TODO
		  */
		  udp.begin(mac); //Begins UDP Protocol. IPAddress is not needed for any of the last 3 lines.
		  udp.beginMulticast(ipmulti,port); //ipmulti was obtained from routine in Ethernet.begin or UPD.begin. I MAY BE WRONG -Wiggles5289
		  #endif


		}
		void loop() {
		  // Read Package if available
		 
		  if(udp.parsePacket()) {
			remote_IP = udp.remoteIP(); // Gets the Remote IP			
			
			while (udp.available()) {
			   parser.processChar(udp.read());
			}
		  }  
		  PollingInput::pollInputs();
		  ExportStreamListener::loopAll();     
		  
		}
		bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		  
		  uint8_t pos = 1;
		  char car = msg[0];
		  
		  udp.beginPacket(udp.remoteIP(),dcs_port);
		  
		  while (car!=0x00){
			udp.write(car);
			car = msg[pos];
			pos++;
		  }
		  udp.write(' ');
		  car = arg[0];
		  while (car!=0x00){
			udp.write(car);
			car = arg[pos];
			pos++;
		  }
		  udp.write('\n');
		  // Serial.write(msg); Serial.write(' '); Serial.write(arg); Serial.write('\n');
		  
		  udp.endPacket();
		  
		  DcsBios::PollingInput::setMessageSentOrQueued();
		  return true;
		}
		
		bool tryToSendDcsBiosMessage(const char* msg) {
		  
		  uint8_t pos = 1;
		  char car = msg[0];
		  
		  udp.beginPacket(udp.remoteIP(),dcs_port);
		  		  
		  while (car!=0x00){
			udp.write(car);
			car = msg[pos];
			pos++;
			if (car=='\n') { // There are two commands in the same line
			  udp.write('\n');
			  udp.endPacket(); // Send 1st part of the message
			  delay(200);
			  udp.beginPacket(udp.remoteIP(),dcs_port); // Second part
			}
		  }
		  
		  udp.write('\n');
		  // Serial.write(msg); Serial.write(' '); Serial.write(arg); Serial.write('\n');
		  
		  udp.endPacket();
		  
		  DcsBios::PollingInput::setMessageSentOrQueued();
		  return true;
		}
		
		
	  }
#endif

#include "internal/Buttons.h"
#include "internal/Switches.h"
#include "internal/SyncingSwitches.h"
#include "internal/Encoders.h"
#include "internal/Potentiometers.h"
#include "internal/RotarySyncingPotentiometer.h"
#include "internal/Leds.h"
#include "internal/Servos.h"
#include "internal/Dimmer.h"
#include "internal/BcdWheels.h"
#include "internal/AnalogMultiPos.h"
#include "internal/RotarySwitch.h"
#include "internal/MatrixSwitches.h"
#include "internal/DualModeButton.h"


namespace DcsBios {
	template<unsigned int first, unsigned int second>
	unsigned int piecewiseMap(unsigned int newValue) {
		return 0;
	}

	template<unsigned int from1, unsigned int to1, unsigned int from2, unsigned int to2, unsigned int... rest>
	unsigned int piecewiseMap(unsigned int newValue) {
		if (newValue < from2) {
			return map(newValue, from1, from2, to1, to2);
		} else {
			return piecewiseMap<from2, to2, rest...>(newValue);
		}
	}
}

#ifndef DCSBIOS_RS485_MASTER
namespace DcsBios {	
	inline bool sendDcsBiosMessage(const char* msg, const char* arg) {
		while(!tryToSendDcsBiosMessage(msg, arg));
		return true;
	}
}

// for backwards compatibility, can be removed when we have a proper place to document this interface:
inline bool sendDcsBiosMessage(const char* msg, const char* arg) {
	while(!DcsBios::tryToSendDcsBiosMessage(msg, arg));
	return true;
}
#endif

#endif // include guard
