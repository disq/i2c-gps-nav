/*
 
 SendOnlySoftwareSerial - adapted from SoftwareSerial by Nick Gammon 28th June 2012
 
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
// 
// Includes
// 
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "SendOnlySoftwareSerial.h"
#include "config.h"
//
// Lookup table
//
typedef struct _DELAY_TABLE
{
  long baud;
  unsigned short rx_delay_centering;
  unsigned short rx_delay_intrabit;
  unsigned short rx_delay_stopbit;
  unsigned short tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
#ifdef TELEMETRY_FRSKY_SOFTSERIAL_PIN
  //  baud    rxcenter   rxintra    rxstop    tx
#if TELEMETRY_FRSKY_SPEED==115200
  { 115200,   1,         17,        17,       12,    },
#elif TELEMETRY_FRSKY_SPEED==57600
  { 57600,    10,        37,        37,       33,    },
#elif TELEMETRY_FRSKY_SPEED==38400
  { 38400,    25,        57,        57,       54,    },
#elif TELEMETRY_FRSKY_SPEED==31250
  { 31250,    31,        70,        70,       68,    },
#elif TELEMETRY_FRSKY_SPEED==28800
  { 28800,    34,        77,        77,       74,    },
#elif TELEMETRY_FRSKY_SPEED==19200
  { 19200,    54,        117,       117,      114,   },
#elif TELEMETRY_FRSKY_SPEED==14400
  { 14400,    74,        156,       156,      153,   },
#elif TELEMETRY_FRSKY_SPEED==9600
  { 9600,     114,       236,       236,      233,   },
#elif TELEMETRY_FRSKY_SPEED==4800
  { 4800,     233,       474,       474,      471,   },
#elif TELEMETRY_FRSKY_SPEED==2400
  { 2400,     471,       950,       950,      947,   },
#elif TELEMETRY_FRSKY_SPEED==1200
  { 1200,     947,       1902,      1902,     1899,  },
#elif TELEMETRY_FRSKY_SPEED==300
  { 300,      3804,      7617,      7617,     7614,  },
#else
  #error "Invalid baud rate for TELEMETRY_FRSKY_SPEED"
#endif
#endif
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
#ifdef TELEMETRY_FRSKY_SOFTSERIAL_PIN
  //  baud    rxcenter    rxintra    rxstop  tx
#if TELEMETRY_FRSKY_SPEED==115200
  { 115200,   1,          5,         5,      3,      },
#elif TELEMETRY_FRSKY_SPEED==57600
  { 57600,    1,          15,        15,     13,     },
#elif TELEMETRY_FRSKY_SPEED==38400
  { 38400,    2,          25,        26,     23,     },
#elif TELEMETRY_FRSKY_SPEED==31250
  { 31250,    7,          32,        33,     29,     },
#elif TELEMETRY_FRSKY_SPEED==28800
  { 28800,    11,         35,        35,     32,     },
#elif TELEMETRY_FRSKY_SPEED==19200
  { 19200,    20,         55,        55,     52,     },
#elif TELEMETRY_FRSKY_SPEED==14400
  { 14400,    30,         75,        75,     72,     },
#elif TELEMETRY_FRSKY_SPEED==9600
  { 9600,     50,         114,       114,    112,    },
#elif TELEMETRY_FRSKY_SPEED==4800
  { 4800,     110,        233,       233,    230,    },
#elif TELEMETRY_FRSKY_SPEED==2400
  { 2400,     229,        472,       472,    469,    },
#elif TELEMETRY_FRSKY_SPEED==1200
  { 1200,     467,        948,       948,    945,    },
#elif TELEMETRY_FRSKY_SPEED==300
  { 300,      1895,       3805,      3805,   3802,   },
#else
  #error "Invalid baud rate for TELEMETRY_FRSKY_SPEED"
#endif
#endif
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
#ifdef TELEMETRY_FRSKY_SOFTSERIAL_PIN
  //  baud    rxcenter    rxintra    rxstop  tx
#if TELEMETRY_FRSKY_SPEED==115200
  { 115200,   3,          21,        21,     18,     },
#elif TELEMETRY_FRSKY_SPEED==57600
  { 57600,    20,         43,        43,     41,     },
#elif TELEMETRY_FRSKY_SPEED==38400
  { 38400,    37,         73,        73,     70,     },
#elif TELEMETRY_FRSKY_SPEED==31250
  { 31250,    45,         89,        89,     88,     },
#elif TELEMETRY_FRSKY_SPEED==28800
  { 28800,    46,         98,        98,     95,     },
#elif TELEMETRY_FRSKY_SPEED==19200
  { 19200,    71,         148,       148,    145,    },
#elif TELEMETRY_FRSKY_SPEED==14400
  { 14400,    96,         197,       197,    194,    },
#elif TELEMETRY_FRSKY_SPEED==9600
  { 9600,     146,        297,       297,    294,    },
#elif TELEMETRY_FRSKY_SPEED==4800
  { 4800,     296,        595,       595,    592,    },
#elif TELEMETRY_FRSKY_SPEED==2400
  { 2400,     592,        1189,      1189,   1186,   },
#elif TELEMETRY_FRSKY_SPEED==1200
  { 1200,     1187,       2379,      2379,   2376,   },
#elif TELEMETRY_FRSKY_SPEED==300
  { 300,      4759,       9523,      9523,   9520,   },
#else
  #error "Invalid baud rate for TELEMETRY_FRSKY_SPEED"
#endif
#endif
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of SendOnlySoftwareSerial supports only 20, 16 and 8MHz processors

#endif

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

//
// Private methods
//

/* static */ 
inline void SendOnlySoftwareSerial::tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}


void SendOnlySoftwareSerial::tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}


//
// Constructor
//
SendOnlySoftwareSerial::SendOnlySoftwareSerial(uint8_t transmitPin, bool inverse_logic /* = false */, bool errors_ok /* = false */) : 
  _tx_delay(0),
  _inverse_logic(inverse_logic),
  _errors_ok(errors_ok)
{
  setTX(transmitPin);
}

//
// Destructor
//
SendOnlySoftwareSerial::~SendOnlySoftwareSerial()
{
  end();
}

void SendOnlySoftwareSerial::setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

//
// Public methods
//

void SendOnlySoftwareSerial::begin(long speed)
{
  _tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
  {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed)
    {
      _tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

}

void SendOnlySoftwareSerial::end()
{
}


// Read data from buffer
int SendOnlySoftwareSerial::read()
{
  return -1;
}

int SendOnlySoftwareSerial::available()
{
  return 0;
}

size_t SendOnlySoftwareSerial::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  uint8_t oldSREG = SREG;
  if (!_errors_ok) {
    cli();  // turn off interrupts for a clean txmit
  }

  // Write the start bit
  tx_pin_write(_inverse_logic ? HIGH : LOW);
  tunedDelay(_tx_delay + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  if (_inverse_logic)
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(LOW); // send 1
      else
        tx_pin_write(HIGH); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(LOW); // restore pin to natural state
  }
  else
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(HIGH); // restore pin to natural state
  }

  if (!_errors_ok) {
    SREG = oldSREG; // turn interrupts back on
  }
  tunedDelay(_tx_delay);
  
  return 1;
}

void SendOnlySoftwareSerial::flush()
{
 return;
}

int SendOnlySoftwareSerial::peek()
{
  return -1;
}
