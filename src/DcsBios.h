/* WIP From The-One and Wiggles5289
NOTES FROM WIGGLES: Search '-Wiggles5289' for internal writing notes. 

We do need to have a way to maybe broadcast the DCSBIOS PC IP address. Would make it easier for users
to use this funciton without messing with routers or if they are unable to assign IPs and is dynamic.
While this is less of a problem if the user is using a switch to localize all the panels to the 
computer, for WiFi it would be very useful.
VERSION CONTROL: V0.1 29JAN2023@1610
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
#include "internal/STM32Ethernet.h" //pulled from https://github.com/stm32duino/STM32Ethernet 
								    //Would be nice to include into DCSBIOS src/internal but idk how to -Wiggles5289

#ifndef USART0_RX_vect
#define USART0_RX_vect USART_RX_vect
#define USART0_TX_vect USART_TX_vect
#define USART0_UDRE_vect USART_UDRE_vect
#endif

#ifndef PRR0
#define PRR0 PRR
#endif

#define DCSBIOS_ETHERNET_ENABLED //FOR EDITOR, REMOVE FOR PUBLISHING. USED TO MAKE DCSBIOS_ETHERNET_ENABLED READABLE ON MY IDE -Wiggles5289
#define DCSBIOS_LAN //FOR EDITOR, REMOVE FOR PUBLISHING. USED TO MAKE DCSBIOS_LAN READABLE ON MY IDE -Wiggles5289

/*Big Brain idea: Derive IP address from MAC in a particular range. I did note while glancing through
some random documentation that the range is limited to subnet(IE 239.255.50.1-239.255.50.254). Could lead to 
conflict if MAC address generation formula limits results to only 255 options.

Also not sure if uCip defined will pass through in sketch will pass to here -Wiggles5289
*/
#ifndef uCip //microcontroller board/panel IP address, needs to be unique to each arduino!
	#define uCip 239,255,50,10
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

#ifdef DCSBIOS_DEFAULT_SERIAL2
	namespace DcsBios {
		ProtocolParser parser;
		void setup() {
			Serial2.begin(250000);
		}
		void loop() {
			while (Serial2.available()) {
				parser.processChar(Serial2.read());
			}
			PollingInput::pollInputs();
			ExportStreamListener::loopAll();			
		}
		bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
			Serial2.write(msg); Serial2.write(' '); Serial2.write(arg); Serial2.write('\n');
			DcsBios::PollingInput::setMessageSentOrQueued();
			return true;
		}
	}
#endif

#ifdef DCSBIOS_LAN
	//Added to have LAN accessibility
	//#include <WiFi.h>
	//
	// Defines:
	//  #define DCSBIOS_LAN to have Wifi access	and then chose
	//
	//		#define DCSBIOS_ESP8266           - for using ESP8266
	//		#define DCSBIOS_ESP32             - for using ESP32
	//		#define DCSBIOS_ETHERNET_ENABLED  - Currently only supports Official Arduino Ethernet Library
	//
	// I do not like I did this. Should have just had defines left as is for each situation -Wiggles5289

	#ifdef DCSBIOS_ESP8266
		//ESP8266
		//#include <WiFi.h> //DOES THIS NEED TO BE ENABLED???? -Wiggles5289
	    #include <ESP8266WiFi.h>
		#include "user_interface.h" // Added to avoid losing UDP Multicast Connection
		#include <WiFiUdp.h>
		WiFiUDP udp;
		//WiFiUDP udp_send;
		IPAddress ipmulti(uCip);
		IPAddress remote_IP;
	#endif
	#ifdef DCSBIOS_ESP32
		//ESP32
		#include <WiFi.h>
		#include <WiFiUdp.h>
		WiFiUDP udp;
		//WiFiUDP udp_send;
		IPAddress ipmulti(uCip);
		IPAddress remote_IP;
	#endif
	#ifdef DCSBIOS_ETHERNET_ENABLED
		#ifdef DCSBIOS_FOR_STM32 //Checks for if an STM32 is used, requires different library if so
		#include "STM32Ethernet.h" /* pulled from https://github.com/stm32duino/STM32Ethernet (NEEDS TO BE 
									  INSTALLED with LwIP Dependency) */
		else
		#include <ethernet.h> //Vanilla Ethernet Library from included ArduinoIDE
							  //Only WIZnet W5100/W5200/W5500-based devices are supported
		EthernetUDP udp;
		IPAddress ip(uCip); // Enter an IP address for your controller
		unsigned int localPort = 8888;      // local port to listen on
		#endif
		//STM32Ethernet and Vanilla Arduino Ethernet Library is almost identical in function to the ESP8266/ESP32 WiFi Library
	#endif

	//ADD LAN

	unsigned int port=5010;
	unsigned int dcs_port=7778;

	uint8_t dataUdP[128];
	/*
	 * WLAN Configuration
	*/
	const char* ssid     = LAN_SSID;
	const char* password = LAN_PASS;
	  
	/*
	CANNOT FIND SOURCE CODE FROM THIS VERSION, BELOW MAY BE ERRONEOUSLY COMMENTED OUT -Wiggles5289
	*/
	// Static IP address
	// IPAddress local_IP(192, 168, 1, 200);
	// Gateway IP address
	// IPAddress gateway(192, 168, 1, 1);
	// IPAddress subnet(255, 255, 0, 0);


	namespace DcsBios {
		
		//unsigned int trans = 0;
		ProtocolParser parser;
		
		void setup() {	
			
		  // Init LAN
		  #if SERIAL_LOG
			Serial.begin(115200);
		  #endif

		  #ifdef DCSBIOS_ESP8266 || DCSBIOS_ESP32
		  WiFi.mode(WIFI_STA);
		  //WiFi.config(local_IP, gateway, subnet);
		  
		  #ifdef DCSBIOS_ESP8266
			wifi_set_sleep_type(NONE_SLEEP_T); //LIGHT_SLEEP_T and MODE_SLEEP_T
		  #endif
		  
		  WiFi.begin(ssid, password);
		  
		  while (WiFi.status() != WL_CONNECTED){
			#if SERIAL_LOG  					
				Serial.print(".");
			#endif
			delay(500);
		  }
		  #if SERIAL_LOG
			Serial.print("\nLAN OK - IP:");
			Serial.println(WiFi.localIP());
		  #endif
		  // multicast listener
		  
		  #ifdef DCSBIOS_ESP8266
			udp.beginMulticast(WiFi.localIP(),ipmulti,port);
		  #endif
		  
		  #ifdef DCSBIOS_ESP32
			udp.beginMulticast(ipmulti,port);
		  #endif
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
