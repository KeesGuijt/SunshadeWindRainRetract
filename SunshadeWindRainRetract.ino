/*

    This unit monitors windspeed and precipitation. If above set levels, a sun shade will be retracted to prevent damage from mechanical loads
	and moisture.
    Add-on for Alecto WS-4500 weather station.

*/

#include <Time.h>
#include <TimeLib.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Source: http://jeelabs.net/attachments/download/49/Ook_OSV2.pde
//
// Oregon V2 decoder added - Dominique Pierre
// Oregon V3 decoder revisited - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

int minSec = 0;

//WEATHERRFPORT:
//ref voltage pin D6 AIN0 (comparator +) pin 10
//input voltage pin D7 AIN1 (comparator -) pin 11
// or A0 pin 23 via ADC mux
//#define SOMFYRFOUT 3 // DigitalPin for Somfy

#define WEATHERRFPORT 1 //1=A0
#define RAINSENSOR  2 //2=D2 A5-analog? 
#define RESETSUPPRESSPIN 12


#define SOMFYRFOUT 3 // DigitalPin for Somfy
#define SYMBOL 640
#define HAUT 0x2
#define STOP 0x1
#define BAS 0x4
#define PROG 0x8
#define EEPROM_ADDRESS 0
#define CLOCKEEPROMADDRESS 12

#define REMOTE 0x021400    //<-- Change it!


unsigned int newRollingCode = 106;       //<-- Change it!
unsigned int rollingCode = 0;
byte frame[7];
byte checksum;

void BuildFrame(byte *frame, byte button);
void SendCommand(byte *frame, byte sync);

const byte reverse_bits_lookup[] = {
  0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
  0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF
};

int windGust;
int windGustOld;
int avgPeakKmhAge;
//bool windGustStored;
int windAverage;
int windAverageOld;
int windDirection;
unsigned long nonInterruptLoopCount = 0;
unsigned long shortLoopCount = 0;
int lastZeroBitLength = 0;
int lastOneBitLength = 0;
int lastLeaderBitLength = 0;
bool batteryLow = false;

class DecodeOOK {
  protected:
    byte total_bits, flip, state, pos, data[25];

    virtual char decode (word width) = 0;

  public:

    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () {
      resetDecoder();
    }

    virtual bool checkSum() {
      return true;
    }

    bool nextPulse (word width) {
      if (state != DONE)

        switch (decode(width)) {
          case -1: resetDecoder(); break;
          case 1:  state = DONE; break;
        }
      return isDone();
    }

    bool isDone () const {
      return state == DONE;
    }

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

// 433 MHz decoders for weather stations


class OregonDecoderV2 : public DecodeOOK {
  public:

    byte max_bits;

    OregonDecoderV2() {
      max_bits = 160;
    }

    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
      if ( !( total_bits & 0x01 ) )
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
      return total_bits == max_bits ? 1 : 0;
    }

    virtual bool checkSum() {
      byte s = 0;
      for ( byte i = 0; i < pos - 2 ; i++ )
        s += ( data[i] & 0xF ) + ( data[i] >> 4 );
      return ( s - 10 ) == data[pos - 2];
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
            {
              gotBit( 0 );
              lastZeroBitLength = width;
            }
            else if ( width > 3400 )
            {
              gotBit( 1 );
              lastOneBitLength = width;
            }
            else
              return -1;
            state = UNKNOWN;
          } else if ( total_bits > 35 && 7650 < width && width < 10350 ) {
            data[pos] = data[pos] << 4;
            pos++;
            lastLeaderBitLength = width;
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
        if ( i % 2 )
          t = reverse_bits_lookup[ ( data[i / 2] & 0xF )];
        else
          t = reverse_bits_lookup[ ( data[i / 2] >> 4  )];
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
      return ( crc == data[pos - 1] );
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
      return ( crc == ( data[pos - 1] & 0x3 ) );
    }
};

OregonDecoderV2   orscV2;
VentusDecoder	  ventus;
FineOffsetDecoder fineOffset;
MandolynDecoder   mandolyn;

volatile word pulse;

#if defined(__AVR_ATmega1280__)

//--not used --

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
  //Serial.print("-");
  if ( !decoder.checkSum() )
    //return;
    ;
  byte windDataType = 0;
  byte pos;
  const byte* data = decoder.getData(pos);

  //capture data with bad chacksum
  //pos=4;
  //Serial.print(pos);
  if (pos < 4)
  {
    return;
  }

  for (byte i = 0; i < pos; ++i) {
    //	i	0 1 2 3 4
    //VENT 1468000070

    if (i == 1)
    {
      if ( ((data[i] >> 4) == 6)  || ((data[i] >> 4) == 0x0E) )	 //allways wind info 
      {
        if ( (data[i] != 0x6C ) && (data[i] != 0xAC)  )  //6E or 6F     not 6C
        {
          windDataType = data[i] & 0x7F;
          if ((windDataType & 0x6E) == 0x6E)  //6E or 6F
          {
            windDirection = ((data[i] & 0x01) );   //      0           0      8  bit 0 (0 or 1)  9 bits long
          }
        }
      }
      else
      {
        windDataType = 0;
      }
      //check battery
      //n2 == Ex (bit 8 = nibble 1xxx)
      if ((data[i] & 0xA0) == 0xA0)
      {
         batteryLow = true;
      }       
    }
    if (i == 2)
    {
      if ((windDataType & 0x6E) == 0x6E)  //6E or 6F
      {
        //windDirection =                       ;   //      0           0      8 (0 or 256)    9 bits long
        windDirection += ((data[i] & 0x80) >> 6);   //i2 bit7 == n4 bit 3  to d1
        windDirection += ((data[i] & 0x40) >> 4);   //      6           2      2
        windDirection += ((data[i] & 0x20) >> 2);   //      5           1      3
        windDirection += ((data[i] & 0x10)     );   //      4           0      4
        //VENT 1468000070  avg wind    0.2m/s n6 + n7(MSB)	(<<4)  bits reversed
        windDirection += ((data[i] & 0x08) << 2);   //i2 bit3 == n5 bit 3  to d5
        windDirection += ((data[i] & 0x04) << 4);   //      2           2      6
        windDirection += ((data[i] & 0x02) << 6);   //      1           1      7
        windDirection += ((data[i] & 0x01) << 8);   //      0           0      8
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
        } //68
      }
    } //for

    //see all packages' hex dump
    //Serial.print(data[i] >> 4, HEX);
    //Serial.print(data[i] & 0x0F, HEX);

  }
  
  //show any packet has been received
  Serial.print("_");
  
  //Serial.print( decoder.checkSum() ? "\tOK" : "\tFAIL" );
  //Serial.print(" ");
  //Serial.print(lastZeroBitLength);
  //Serial.print(" ");
  //Serial.print(lastOneBitLength);
  //Serial.print(" ");
  //Serial.print(lastLeaderBitLength);
  //Serial.print(" ");
  //Serial.print( "\n" );

  decoder.resetDecoder();
}  // reportSerial


void digitalClockDisplay() {
  // digital clock display of the time
  printDigits(year());
  printDigits(month());
  printDigits(day());
  Serial.print(" ");
  Serial.print(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.print(" ");
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void processMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  //int waitTime = 0;
  unsigned long controlCode = 0;

  const char *timeHeaderUc = "T";// Header tag for serial time sync message
  const char *timeHeaderLc = "t";// Header tag for serial time sync message
  const char *commandHeaderUc = "C";// Header tag for serial device command message
  const char *commandHeaderLc = "c";// Header tag for serial device command message

  char header = (char)Serial.read();


  if (header == 't' || header == 'T')
  {
    digitalClockDisplay();
    Serial.println(" ");
    Serial.print("Time received from PC: ");
    pctime = Serial.parseInt();
    if ( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(pctime); // Sync Arduino clock to the time received on the serial port

      //Put the float data from the EEPROM at position 'CLOCKEEPROMADDRESS'
      EEPROM.put(CLOCKEEPROMADDRESS, pctime);
      Serial.println("Timeinfo written to EEPROM.");
    }
    digitalClockDisplay();
    Serial.println(" ");
  }
  else if ( (header == 'c') || (header == 'C') )
  {
    controlCode = Serial.parseInt();
    if ( controlCode == 101 )
    {
      pinMode(RESETSUPPRESSPIN, INPUT);  //pulling the resetpin down is not a good idea....
      digitalWrite(RESETSUPPRESSPIN, 0); // set the RESETSUPPRESSPIN OFF
      Serial.print("RESETSUPPRESSPIN off\n");
      Serial.println(controlCode);
    }
  }
  else if (header == 'm' || header == 'u' || header == 'h') {
    Serial.println("Retract"); // Somfy sun shade
    BuildFrame(frame, HAUT);
    SendCommand(frame, 2);
    for (int i = 0; i < 2; i++) {
      SendCommand(frame, 7);
    }
  }
  else if (header == 's') {
    Serial.println("Stop");
    BuildFrame(frame, STOP);
    SendCommand(frame, 2);
    for (int i = 0; i < 2; i++) {
      SendCommand(frame, 7);
    }
  }
  else if (header == 'b' || header == 'd') {
    Serial.println("Descend");
    BuildFrame(frame, BAS);
    SendCommand(frame, 2);
    for (int i = 0; i < 2; i++) {
      SendCommand(frame, 7);
    }
  }
  else if (header == 'p') {
    Serial.println("Prog");
    BuildFrame(frame, PROG);
    SendCommand(frame, 2);
    for (int i = 0; i < 2; i++) {
      SendCommand(frame, 7);
    }
  }
}


/*   This part allows you to emulate a Somfy RTS or Simu HZ remote.
   If you want to learn more about the Somfy RTS protocol, check out https://pushstack.wordpress.com/somfy-rts-protocol/

   The rolling code will be stored in EEPROM, so that you can power the Arduino off.

   Easiest way to make it work for you:
    - Choose a remote number
    - Choose a starting point for the rolling code. Any unsigned int works, 1 is a good start
    - Upload the sketch
    - Long-press the program button of YOUR ACTUAL REMOTE until your blind goes up and down slightly
    - send 'p' to the serial terminal
    - me: your blind should go up and down slightly
  To make a group command, just repeat the last two steps with another blind (one by one)

  Then:
    - m, u or h will make it to go up
    - s make it stop
    - b, or d will make it to go down
    - you can also send a HEX number directly for any weird command you (0x9 for the sun and wind detector for instance)
*/



void BuildFrame(byte *frame, byte button) {
  unsigned int code;
  EEPROM.get(EEPROM_ADDRESS, code);
  frame[0] = 0xA7; // Encryption key. Doesn't matter much
  frame[1] = button << 4;  // Which button did  you press? The 4 LSB will be the checksum
  frame[2] = code >> 8;    // Rolling code (big endian)
  frame[3] = code;         // Rolling code
  frame[4] = REMOTE >> 16; // Remote address
  frame[5] = REMOTE >>  8; // Remote address
  frame[6] = REMOTE;       // Remote address

  Serial.print("Frame         : ");
  for (byte i = 0; i < 7; i++) {
    if (frame[i] >> 4 == 0) { //  Displays leading zero in case the most significant
      Serial.print("0");     // nibble is a 0.
    }
    Serial.print(frame[i], HEX); Serial.print(" ");
  }

  // Checksum calculation: a XOR of all the nibbles
  checksum = 0;
  for (byte i = 0; i < 7; i++) {
    checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
  }
  checksum &= 0b1111; // We keep the last 4 bits only


  //Checksum integration
  frame[1] |= checksum; //  If a XOR of all the nibbles is equal to 0, the blinds will
  // consider the checksum ok.

  Serial.println(""); Serial.print("With checksum : ");
  for (byte i = 0; i < 7; i++) {
    if (frame[i] >> 4 == 0) {
      Serial.print("0");
    }
    Serial.print(frame[i], HEX); Serial.print(" ");
  }


  // Obfuscation: a XOR of all the bytes
  for (byte i = 1; i < 7; i++) {
    frame[i] ^= frame[i - 1];
  }

  Serial.println(""); Serial.print("Obfuscated    : ");
  for (byte i = 0; i < 7; i++) {
    if (frame[i] >> 4 == 0) {
      Serial.print("0");
    }
    Serial.print(frame[i], HEX); Serial.print(" ");
  }
  Serial.println("");
  Serial.print("Rolling Code  : ");
  //code = 101; //temp to change remote?
  Serial.println(code);
  EEPROM.put(EEPROM_ADDRESS, code + 1); //  We store the value of the rolling code in the
  // EEPROM. It should take up to 2 adresses but the
  // Arduino function takes care of it.
}

void SendCommand(byte *frame, byte sync) {
  if (sync == 2) { // Only with the first frame.
    //Wake-up pulse & Silence
    digitalWrite(SOMFYRFOUT, 1);
    delayMicroseconds(9415);
    digitalWrite(SOMFYRFOUT, 0);
    delayMicroseconds(89565);
  }

  // Hardware sync: two sync for the first frame, seven for the following ones.
  for (int i = 0; i < sync; i++) {
    digitalWrite(SOMFYRFOUT, 1);
    delayMicroseconds(4 * SYMBOL);
    digitalWrite(SOMFYRFOUT, 0);
    delayMicroseconds(4 * SYMBOL);
  }

  // Software sync
  digitalWrite(SOMFYRFOUT, 1);
  delayMicroseconds(4550);
  digitalWrite(SOMFYRFOUT, 0);
  delayMicroseconds(SYMBOL);


  //Data: bits are sent one by one, starting with the MSB.
  for (byte i = 0; i < 56; i++) {
    if (((frame[i / 8] >> (7 - (i % 8))) & 1) == 1) {
      digitalWrite(SOMFYRFOUT, 0);
      delayMicroseconds(SYMBOL);
      digitalWrite(SOMFYRFOUT, 1);
      delayMicroseconds(SYMBOL);
    }
    else {
      digitalWrite(SOMFYRFOUT, 1);
      delayMicroseconds(SYMBOL);
      digitalWrite(SOMFYRFOUT, 0);
      delayMicroseconds(SYMBOL);
    }
  }

  digitalWrite(SOMFYRFOUT, 0);
  delayMicroseconds(30415); // Inter-frame silence
}

//end of Somfy code





void setup () {

  //wdt_enable(WDTO_8S);

  //Serial.begin(9600);
  Serial.begin(115200);
  Serial.print("[WR]\n");

  digitalWrite(RESETSUPPRESSPIN, 1);// set the RESETSUPPRESSPIN ON
  pinMode(RESETSUPPRESSPIN, OUTPUT);

  //Get the time data from the EEPROM at position 'CLOCKEEPROMADDRESS'
  unsigned long pctime;
  EEPROM.get(CLOCKEEPROMADDRESS, pctime);
  setTime(pctime); // Sync Arduino clock to the time received on the serial port
  Serial.println("Arduino clock set from Eeprom");
  digitalClockDisplay();
  //Serial.println(".");

#if !defined(__AVR_ATmega1280__)
  pinMode(13 + WEATHERRFPORT, INPUT);	// use the AIO pin
  digitalWrite(13 + WEATHERRFPORT, 1); // enable pull-up

  // use analog comparator to switch at 1.1V bandgap transition
  ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
  // use analog comparator to switch at potmeter set threshold 
  //ACSR =  _BV(ACI) | _BV(ACIE);

  // set ADC mux to the proper port
  ADCSRA &= ~ bit(ADEN);
  ADCSRB |= bit(ACME);
  ADMUX = WEATHERRFPORT - 1;
#else

  //--not used --

  attachInterrupt(1, ext_int_1, CHANGE);

  DDRE  &= ~_BV(PE5);
  PORTE &= ~_BV(PE5);
#endif


  //Serial.begin(9600);
  //while (!Serial) ; // Needed for Leonardo only

  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  //Somfy
  pinMode(SOMFYRFOUT, OUTPUT);  //Pin  an output
  digitalWrite(SOMFYRFOUT, 0); // Pin  LOW

  if (EEPROM.get(EEPROM_ADDRESS, rollingCode) < newRollingCode) {
    EEPROM.put(EEPROM_ADDRESS, newRollingCode);
  }
  Serial.print("Simulated remote number : "); Serial.println(REMOTE, HEX);
  Serial.print("Current rolling code    : "); Serial.println(rollingCode);

}


void loop () {
  static float windAverageKmh = 0;
  static float windGustKmh = 0;
  static float averagePeakKmh = 0;
  static float gustPeakKmh = 0;
  static unsigned long averagePeakKmhTimeStored = 0;
  static unsigned long gustPeakKmhTimeStored = 0;
  static unsigned long retractTimeoutTime = now(); //Somfy command can only be sent once in 15 min
  static int timeDiff = 0;
  unsigned long oldClockTime;
  bool pulseActivity;

  //no int now, handle 2 messanges if possible
  //cli();
  if (Serial.available()) {
    processMessage();
  }
  cli();
  // ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
  //cbi( ACSR, ACBG ); // disable
  //  cbi( ACSR, ACI ); // disable
  //  cbi( ACSR, ACIE ); // disable analog interrupt

  word p = pulse;
  pulse = 0;

  sei();
  //sbi( ACSR, ACBG ); // enable
  //  sbi( ACSR, ACI ); // enable
  //  sbi( ACSR, ACIE ); // enable analog interrupt

  if (p != 0) {
    //Serial.print(".");
    pulseActivity = true;

    if (orscV2.nextPulse(p))
    {
      Serial.print("OSV1?");
      reportSerial("OSV2", orscV2);
    }
    if (ventus.nextPulse(p))
    {
      //Serial.print(p);
      //Serial.print(" ");
      reportSerial("VENT", ventus);
      //Serial.print(".");
      //if (minSec != (minute() * 60 + second()) )  //only proces once a second
      {
        minSec = (minute() * 60 + second()) ;
        //		 if (ventus1=0)
        //digitalClockDisplay();
        //reportSerial("VENT", ventus);
        //			ventus1=ventus;

        //if ( (windAverage != windAverageOld) || (windGust != windGustOld) )  //only proces changed data, once a second
        {
          minSec = (minute() * 60 + second()) ;
          windAverageKmh = float(windAverage) / 5.0 * 3.6 * 2.5; //personal scaling // 0.2 m/s to km/h, compensate 3* for alt
          windGustKmh = float(windGust) / 5.0 * 3.6 * 2.0;  // 0.2 m/s to km/h, compensate 3* for alt )
          //Serial.println();
          Serial.print("@ ");
          digitalClockDisplay();
          /*
          Serial.print("Average: ");
          if ( windAverageKmh < 10)
          {
            Serial.print(' ');
          }
          Serial.print(windAverageKmh);
          Serial.print(' ');
          Serial.print("Gust: ");
          if ( windGustKmh < 10)
          {
            Serial.print(' ');
          }
          Serial.print(windGustKmh);
          */
        }
        // Average Peak
        if ( averagePeakKmh < windAverageKmh)
        {
          averagePeakKmh = windAverageKmh;
          averagePeakKmhTimeStored = now();
        }
        if ( (averagePeakKmhTimeStored + 30 * 60) < now() ) //30 min
        {
          averagePeakKmh -= 1; // start lowering peak value to find a new, lower peak
        }

        //if ( (windAverage != windAverageOld) || (windGust != windGustOld) )
        {
          Serial.print(" Average Peak: ");
          /*
          if ( averagePeakKmh < 10)
          {
            Serial.print(' ');
          }
          Serial.print(averagePeakKmh);
          */
          /*
            1. Neem de windsnelheid in km/h, en trek daar de wortel uit.
            2.Is de uitkomst kleiner of gelijk aan 7, trek er dan 1 van af.
            Is de uitkomst groter dan 10, tel er dan 1 bij op.
            3.Rond de uitkomst af, je hebt nu de windkracht in Bf.
          */
          long averagePeakBft = sqrt(averagePeakKmh);
          if (averagePeakBft <= 7)
          {
            averagePeakBft--;
          }
          else if (averagePeakBft > 10)
          {
            averagePeakBft++;
          }
          if ( averagePeakBft < 0 || averagePeakBft > 12 )
          {
            averagePeakBft = 0;
          }
          //Serial.print(" = ");
          Serial.print(averagePeakBft);
          Serial.print("_Bft.");
          //Serial.println();

          //Gust Peak

          if (gustPeakKmh < windGustKmh)
          {
            gustPeakKmh = windGustKmh;
            gustPeakKmhTimeStored = now();
          }
          Serial.print(" Gust Peak: ");
          //Serial.print(gustPeakKmh);

          long gustPeakBft = sqrt(gustPeakKmh);
          if (gustPeakBft <= 7)
          {
            gustPeakBft--;
          }
          else if (gustPeakBft > 10)
          {
            gustPeakBft++;
          }
          if ( gustPeakBft < 0 || gustPeakBft > 12 )
          {
            gustPeakBft = 0;
          }
          //Serial.print(" = ");
          Serial.print(gustPeakBft);
          Serial.print("_Bft.");
          Serial.print(" From: ");
          if ( windDirection < 10)
          {
            Serial.print(' ');
          }
          if ( windDirection < 100)
          {
            Serial.print(' ');
          }
          Serial.print(windDirection);
          if ( gustPeakKmh > 44 )       //km/h trigger
          {
            Serial.print(" Too_windy ");
          }
        }

        if ( gustPeakKmh > 50 )       //km/h trigger
        {
          if ( retractTimeoutTime < now() )
          {
            Serial.print(" Retracting (Wind)....");
            BuildFrame(frame, HAUT);   // Somfy is a French company, after all.
            SendCommand(frame, 2);
            for (int i = 0; i < 2; i++) {
              SendCommand(frame, 7);
            }
            retractTimeoutTime = now() + (15 * 60); //15 minutes to seconds, Somfy command can only be sent once in 15 min
          }
        }
        if ( (gustPeakKmhTimeStored + 30 * 60) < now() ) //30 min
        {
          gustPeakKmh -= 1; // start lowering peak value to find a new, lower peak
        }

        //if ( (windAverage != windAverageOld) || (windGust != windGustOld) )
        {
          windAverageOld = windAverage;
          windGustOld = windGust;
          if ( (!digitalRead(RAINSENSOR))  )
          {
            Serial.print(" Raining ");
          }
          else
          {
            Serial.print(" Dry ");
          }
          Serial.print( " " );
        }
      }
      if ( (!digitalRead(2))  )
      {
        //Serial.print(" Raining ");
        if ( retractTimeoutTime < now() )
        {
          Serial.println(" Retracting (Rain)....");
          BuildFrame(frame, HAUT);   // Somfy is a French company, after all.
          SendCommand(frame, 2);
          for (int i = 0; i < 2; i++) {
            SendCommand(frame, 7);
          }
          retractTimeoutTime = now() + (15 * 60); //15 minutes to seconds, Somfy command can only be sent once in 15 min
        }
      }
      //end of datastring
      if (batteryLow)
      {
         Serial.print( " Battery_Low" );
      }
      Serial.println( " $" );

    } //ventus
    if (fineOffset.nextPulse(p))
    {
      Serial.print("fineOffset?");
      reportSerial("FINE", fineOffset);
    }
    if (mandolyn.nextPulse(p))
    { Serial.print("mand?");
      reportSerial("MAND", mandolyn);
    }
  } //p!=0

  nonInterruptLoopCount++;
  shortLoopCount++;
  if ( (hour() == 18) and (minute() == 00) and (second() < 5) )
  {
    Serial.println("Retract"); // Somfy sun shade
    BuildFrame(frame, HAUT);
    SendCommand(frame, 2);
    for (int i = 0; i < 2; i++) {
      SendCommand(frame, 7);
    }
  }

  if ( minSec == 0 )  //after a reset, wait for first weather data
  {
    minSec = minute() * 60 + second();  //fake time read after a reset
  }
  timeDiff = ( (minute() * 60 + second()) - minSec );
  if ( timeDiff < -3300 )
  {
    timeDiff = timeDiff + 3600;
  }
  if ( timeDiff > 3300 )
  {
    timeDiff = timeDiff - 3600;
  }

  //try to detect problem faster, make visable in realtime
  //display something at least every 60 sec,
  //check clock using loop, check loop using clock
  //both should advance, else print
  //check every 60 sec : loopcount has increased enough ?
  //check every x loops: time has past enough ?

  if ( shortLoopCount > 800000  )  //show clock every short loop
  {
    //Serial.print(".");
    //Serial.print(now() - oldClockTime);
    oldClockTime = now();
    shortLoopCount = 0;
    if (pulseActivity == true)
    {
      //Serial.print(" ");
      //Serial.print("*");
      pulseActivity = false;
    }
    //Serial.println(" ");
  }
  if ( second() == 0 )  //show shortLoopCount every minute
  {
    //Serial.print(",");
    //Serial.print(shortLoopCount);
    delay(1000);
    if (pulseActivity == true)
    {
      //Serial.print(" ");
      //Serial.print("*");
      pulseActivity = false;
    }
    //Serial.println(" ");
  }
  if ( (timeDiff > 300) || (timeDiff < -300) || nonInterruptLoopCount > 1600000000  )  //last read time and current time should not be more than 5 min apart
  {
    unsigned long pctime;
    pctime = now();
    EEPROM.put(CLOCKEEPROMADDRESS, pctime);
    Serial.println("Timeinfo written to EEPROM. Resetting");
    Serial.print("minSec : ");
    Serial.println(minSec);
    Serial.print("minute60+secs : ");
    Serial.println(minute() * 60 + second());
    delay(100);
    //reset
    pinMode(RESETSUPPRESSPIN, OUTPUT);
    digitalWrite(RESETSUPPRESSPIN, 0);  // set the RESETSUPPRESSPIN OFF: reset the arduino
  }

  //delay(10);

  //tell the watchdog all is well
  //wdt_reset();
} //loop

