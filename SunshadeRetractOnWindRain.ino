#include <Time.h>
#include <TimeLib.h>

// Source: http://jeelabs.net/attachments/download/49/Ook_OSV2.pde
//
// Oregon V2 decoder added - Dominique Pierre
// Oregon V3 decoder revisited - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $

#include <TimeLib.h>

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

int minSec = 0;

#define PORT 2
#define RAINSENSOR  A6

const byte reverse_bits_lookup[] = {
	0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
	0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF
};

int windGust;
bool windGustStored;
int windAverage;
bool windAverageStored;

class DecodeOOK {
protected:
	byte total_bits, flip, state, pos, data[25];

	virtual char decode (word width) =0;

public:

	enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

	DecodeOOK () { resetDecoder(); }

	virtual bool checkSum() { return true; }

	bool nextPulse (word width) {
		if (state != DONE)
		
			switch (decode(width)) {
				case -1: resetDecoder(); break;
				case 1:  state = DONE; break;
			}
		return isDone();
	}

	bool isDone () const { return state == DONE; }

	const byte* getData (byte& count) const {
		count = pos;
		return data; 
	}

	void resetDecoder () {
		total_bits = pos = flip = 0;
		state = UNKNOWN;
	}

	virtual void gotBit (char value) {
		data[pos] = (data[pos] << 1) | value;
		total_bits++;
		pos = total_bits >> 3;
		if (pos >= sizeof data) {
			resetDecoder();
			return;
		}
		state = OK;
	}

	// store a bit using Manchester encoding
	void manchester (char value) {
		flip ^= value; // manchester code, long pulse flips the bit
		gotBit(flip);
	}

};

// 433 MHz decoders


class OregonDecoderV2 : public DecodeOOK {
public:
	
	byte max_bits;
	
	OregonDecoderV2() {
		max_bits = 160;
	}
	
	// add one bit to the packet data buffer
	virtual void gotBit (char value) {
		if( !( total_bits & 0x01 ) )
			data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
		total_bits++;
		pos = total_bits >> 4;
		if (pos >= sizeof data) {
			resetDecoder();
			return;
		}
		state = OK;
	}

	virtual char decode (word width) {
		if (200 <= width && width < 1400) {
			byte w = width >= 700;
			switch (state) {
				case UNKNOWN:
					if (w != 0) {
						// Long pulse
						++flip;
// 					} else if (w == 0 && 24 <= flip) {
					} else if (32 <= flip) {
						// Short pulse, start bit
						flip = 0;
						state = T0;
					} else {
						// Reset decoder
						return -1;
					}
					break;
				case OK:
					if (w == 0) {
						// Short pulse
						state = T0;
					} else {
						// Long pulse
						manchester(1);
					}
					break;
				case T0:
					if (w == 0) {
						// Second short pulse
						manchester(0);
					} else {
						// Reset decoder
						return -1;
					}
					break;
			}
// 		} else if (width >= 2500 && pos >= 8) {
// 			return 1;
		} else {
			return -1;
		}
		
		if ( pos == 2 ) {
			if ( data[0] == 0x1A ) {
				if ( data[1] == 0x89 )			// WRGR800
					max_bits = 176;
				else if ( data[1] == 0x99 )		// WRGR800
					max_bits = 176;
			} else if ( data[0] == 0x2A ) {
				if ( data[1] == 0x19 )			// RCR800 cs8
					max_bits = 184;
				else if ( data[1] == 0x1D )		// RGR918
					max_bits = 168;
			} else if ( data[0] == 0x5A ) {
				if ( data[1] == 0x5D )			// BTHR918
					max_bits = 176;
				else if ( data[1] == 0x6D )		// BTHR918N
					max_bits = 192;
			} else if ( ( data[0] == 0x8A || data[0] == 0x9A ) && data[1] == 0xEC ) {	// RTGR328N
				max_bits = 208;
			} else if ( ( data[0] == 0xDA ) && ( data[1] == 0x78 ) ) {	// UVN800
				max_bits = 144;
			} else if ( data[0] == 0xEA ) {
				if ( data[1] == 0x4C )			// TH132N cs1
					max_bits = 136;
				else if ( data[1] == 0x7C )		// UV138 cs1
					max_bits = 240;
			} else {
				max_bits = 160;
			}
		}
		return total_bits == max_bits ? 1: 0;
	}
	
	virtual bool checkSum() {
		byte s = 0;
		for ( byte i = 0; i < pos-2 ; i++ )
			s += ( data[i] & 0xF ) + ( data[i] >> 4 );
		return ( s - 10 ) == data[pos-2];
	}
};

class VentusDecoder : public DecodeOOK {
public:
	
	VentusDecoder () {}
	
	// see also http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf
	virtual char decode (word width) {

		switch (state ) {
			case UNKNOWN:	// Signal on
				if ( 425 <= width && width < 575)
					state = T0;
				else
					return -1;
				break;
			case T0:		// Signal off = bit
				if ( 1700 < width && width < 4600 ) {
					if ( width < 2300 )
						gotBit( 0 );
					else if ( width > 3400 )
						gotBit( 1 );
					else
						return -1;
					state = UNKNOWN;
				} else if ( total_bits > 35 && 7650 < width && width < 10350 ) {
					data[pos] = data[pos] << 4;
					pos++;
					return 1;
				} else
					return -1;
				break;
			default:
				return -1;
		}
		return 0;
	}

	virtual bool checkSum() {
		byte s, t;
		bool rain = ( ( data[1] & 0x7F ) == 0x6C );
		s = ( rain ? 0x7 : 0xF );
		for ( byte i = 0; i < 8; i++ ) {
			if ( i%2 )
				t = reverse_bits_lookup[ ( data[i/2] & 0xF )];
			else
				t = reverse_bits_lookup[ ( data[i/2] >> 4  )];
			s += ( rain ? t : -t );
		}
		return ( s & 0x0F ) == reverse_bits_lookup[( data[4] >> 4 )];
	}
};

class FineOffsetDecoder : public DecodeOOK {
public:
	// https://github.com/NetHome/Coders/blob/master/src/main/java/nu/nethome/coders/decoders/FineOffsetDecoder.java
	// https://github.com/lucsmall/BetterWH2/blob/master/BetterWH2.ino
	FineOffsetDecoder() { }

	virtual char decode (word width) {
		if (width < 400 || width > 1600)
			return -1;
		
		switch (state) {
			case UNKNOWN:	// Signal start (on)
				if (width < 600) {
					state = OK;
				} else
					return -1;
				break;
			case OK:		// Signal off
				if (900 < width && width < 1100) {
					++flip;
					state = T1;
				} else
					return -1;
				break;
			case T1:		// Signal on = bit
				if (width < 600 && flip > 10) {
					gotBit(1);
				} else if ( width > 1350 ) {
					flip = 11;
					gotBit(0);
				} else {
					return -1;
				}
				
				// Temperature & humidity
				if ( total_bits == 40 && ( data[0] & 0xF0 ) != 0x30 )
						return 1;
				// Rain
				else if ( total_bits == 56 )
					return 1;
				break;
			default:
				return -1;
		}
		return 0;
	}
	
	// https://gitorious.org/sticktools/protocols/source/4698e465843a0eddc4c7029759f9c1dc79d4aab8:fineoffset.c
	virtual bool checkSum() {
		unsigned char i, bit, crc = 0;

		for (i = 0; i < pos - 1; i++) {
			crc = (data[i] ^ crc);
			for (bit = 0; bit < 8; bit++) {
				crc = ((crc << 1) ^ ((crc & 0x80) ? 0x131 : 0)) & 0xff;
			}
		}
		return ( crc == data[pos-1] );
	}
};

class MandolynDecoder : public DecodeOOK {
public:
	// https://github.com/NetHome/Coders/blob/master/src/main/java/nu/nethome/coders/decoders/UPMDecoder.java
	// http://ala-paavola.fi/jaakko/doku.php?id=wt450h
	MandolynDecoder() { }

	virtual char decode (word width) {
		if (width > 2100) {
			return -1;
		} else if (width < 800) {
			if ( total_bits > 35 )
				return 1;
			return -1;
		}
		switch (state) {
			case UNKNOWN:				// First short
				if (width < 1100) {
					state = T1;
				} else {
					return -1;
				}
				break;
			case OK:
				if ( width < 1100) {	// First short
					state = T1;
				} else if ( width > 1800 ) {
					gotBit( 0 );
				} else {
					return -1;
				}
				break;
			
			case T1:					// Signal off
				if ( width < 1100) {	// Second short
					gotBit( 1 );
					state = OK;
				} else {
					return -1;
				}
				break;

			default:
				return -1;
		}
		return 0;
	}

	virtual bool checkSum() {
		// Checking fixed positions preamble b00-b04 being b1100 and position b11-b12 being b11
		return ( ( data[0] & 0xF0 ) == 0xC0 && ( data[1] & 0x30 ) == 0x30 );
		unsigned char i, bit, crc = 0;
		for (i = 0; i < pos - 2; i++) {
			for ( bit = 0; bit < 8; bit += 2 ) {
				crc ^= ( ( data[i] >> bit ) & 0x3 );
			}
		}
		return ( crc == ( data[pos-1] & 0x3 ) );
	}
};

OregonDecoderV2   orscV2;
VentusDecoder	  ventus;
FineOffsetDecoder fineOffset;
MandolynDecoder   mandolyn;

volatile word pulse;

#if defined(__AVR_ATmega1280__)

xcv--not used --

void ext_int_1(void) {
#else
ISR(ANALOG_COMP_vect) {
#endif
	static word last;
	// determine the pulse length in microseconds, for either polarity
	pulse = micros() - last;
	last += pulse;
}

void reportSerial (const char* s, class DecodeOOK& decoder) {
	if ( !decoder.checkSum() )
		return;
	byte pos;
	byte windDataType;
	
	const byte* data = decoder.getData(pos);
	//Serial.print(s);
	//Serial.print(' ');
	for (byte i = 0; i < pos; ++i) {
		//	i	0 1 2 3 4
		//VENT 1468000070
		if (i == 1)
		{
			if ((data[i] >> 4) == 6)	 //allways wind info
			{
			windDataType = data[i];
			}
			else
			{
				windDataType = 0;
			}
    }
		if (i == 3)
		{
			if ((windDataType >> 4) == 6)	  //allways wind info
			{
				if (windDataType == 0x68)
				{
					//VENT 1468000070  avg wind    0.2m/s n6(LSB) + n7	  bits reversed
					windAverage = 0;
					windAverage += ((data[i] & 0x80) >> 7);   //i3 bit7 == n6 bit 3  to s0  
					windAverage += ((data[i] & 0x40) >> 5);   //      6           2      1
					windAverage += ((data[i] & 0x20) >> 3);   //      5           1      2
					windAverage += ((data[i] & 0x10) >> 1);   //      4           0      3
					//VENT 1468000070  avg wind    0.2m/s n6 + n7(MSB)	(<<4)  bits reversed
					windAverage += ((data[i] & 0x08) << 1);   //i3 bit3 == n7 bit 3  to s4 
					windAverage += ((data[i] & 0x04) << 3);   //      2           2      5
					windAverage += ((data[i] & 0x02) << 5);   //      1           1      6
					windAverage += ((data[i] & 0x01) << 7);   //      0           0      7  
					if (not (windAverageStored))
					{
						//Serial.print(data[i] >> 4, HEX);
						//Serial.print(data[i] & 0x0F, HEX);
						//digitalClockDisplay(); 
						//Serial.print(' ');
						//Serial.print('a');
						//Serial.print(windAverage / 5 * 3.6);
						//Serial.print( "\n" );
		    		windAverageStored = true;
					}
				}
				else
				{
					//VENT 566E000040	wind gust  0.2m/s n6(LSB) + n7	bits reversed
					windGust = 0;
					windGust += ((data[i] & 0x80) >> 7);   //i3 bit7 == n6 bit 3  to g0   
					windGust += ((data[i] & 0x40) >> 5);   //      6           2      1   
					windGust += ((data[i] & 0x20) >> 3);   //      5           1      2   
					windGust += ((data[i] & 0x10) >> 1);   //      4           0      3   
					//VENT 566E000040	wind gust  0.2m/s n6 + n7(MSB)	(<<4)  bits reversed 
					windGust += ((data[i] & 0x08) << 1);   //i3 bit3 == n7 bit 3  to g4   
					windGust += ((data[i] & 0x04) << 3);   //      2           2      5   
					windGust += ((data[i] & 0x02) << 5);   //      1           1      6   
					windGust += ((data[i] & 0x01) << 7);   //      0           0      7  
					if (not (windGustStored))
					{
						//Serial.print(data[i] >> 4, HEX);
						//Serial.print(data[i] & 0x0F, HEX);
						//Serial.print(' ');
						//Serial.print('g');
						//Serial.print(windGust / 5 * 3.6);
						//Serial.print( "\n" );
						windGustStored = true;
					}
				} //68
			}		
	} //for

	//Serial.print(data[i] >> 4, HEX);
	//Serial.print(data[i] & 0x0F, HEX);
	}
	//Serial.print( decoder.checkSum() ? "\tOK" : "\tFAIL" );
	//Serial.print( "\n" );

	decoder.resetDecoder();
}  // reportSerial


void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
/*
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
*/
Serial.print(" ");
  //Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
	Serial.print('0');
  Serial.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
	 pctime = Serial.parseInt();
	 if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
	   setTime(pctime); // Sync Arduino clock to the time received on the serial port
	 }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}




void setup () {
	Serial.begin(9600);
 	Serial.print("[WR]\n");

#if !defined(__AVR_ATmega1280__)
	pinMode(13 + PORT, INPUT);	// use the AIO pin
	digitalWrite(13 + PORT, 1); // enable pull-up

	// use analog comparator to switch at 1.1V bandgap transition
	ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);

	// set ADC mux to the proper port
	ADCSRA &= ~ bit(ADEN);
	ADCSRB |= bit(ACME);
	ADMUX = PORT - 1;
#else

	--not used --
	
	attachInterrupt(1, ext_int_1, CHANGE);

	DDRE  &= ~_BV(PE5);
	PORTE &= ~_BV(PE5);
#endif


//Serial.begin(9600);
while (!Serial) ; // Needed for Leonardo only
pinMode(13, OUTPUT);

//configure pin2 as an input and enable the internal pull-up resistor
pinMode(2, INPUT_PULLUP);
pinMode(13, OUTPUT);

setSyncProvider( requestSync);	//set function to call when sync required
Serial.println("Waiting for sync message");


}


void loop () {
  
	cli();
	word p = pulse;
	pulse = 0;
	sei();

	int secondOld=0;

	if (p != 0) {
	//Serial.println("Waiting for sync message");
 

    if (Serial.available()) {
      processSyncMessage();
    }

		if (orscV2.nextPulse(p))
			reportSerial("OSV2", orscV2);  
		if (ventus.nextPulse(p))
		{
			if (minSec != (minute()*60+second()) )
			{
				minSec = (minute()*60+second()) ;
				//		 secondOld = second;
				//		 if (ventus1=0)
				//digitalClockDisplay(); 
				reportSerial("VENT", ventus);
				//			ventus1=ventus;
				//if ( windAverageStored and windGustStored )
				{
          Serial.print( "\n" );
					digitalClockDisplay(); 
          Serial.print(' ');
          Serial.print('a');
          Serial.print(windAverage / 5.0 * 3.6 * 2.5);  // 0.2 m/s to km/h, compensate 3* for alt )
          Serial.print(' ');
          Serial.print('g');
          Serial.print(windGust / 5.0 * 3.6 * 2.5 - 3.6);  // 0.2 m/s to km/h, compensate 2* for alt )
					//delay(3000);   // read each once
					windAverageStored = false;
	  				windGustStored = false;
				}
			} 
		}  
		if (fineOffset.nextPulse(p))
			reportSerial("FINE", fineOffset);
		if (mandolyn.nextPulse(p))
			reportSerial("MAND", mandolyn);
	}

  
  //read the rainserver as digital input
 
  if (!digitalRead(2))
  {
    Serial.println("Raining");
    delay(1000);
  }
}

