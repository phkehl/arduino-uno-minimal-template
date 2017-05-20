/*!
    \file
    \brief flipflip's Arduino Uno Stuff for avr-gcc and avr-libc

    Copyright (c) 2017 Philippe Kehl (flipflip at oinkzwurgl dot org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    \defgroup FFSTUFF flipflip's Arduino Uno Stuff

    @{
*/

#ifndef __FFSTUFF_H__
#define __FFSTUFF_H__

#include <stdio.h>         // libc: standard buffered input/output
#include <stdarg.h>        // libc: variable argument lists
#include <string.h>        // libc: string operations
#include <math.h>          // libc: mathematical functions
#include <stdlib.h>        // libc: general utilities
#include <stdint.h>        // libc: integer types
#include <inttypes.h>      // libc: fixed size integer types
#include <stddef.h>        // libc: standard type definitions
#include <stdbool.h>       // libc: boolean types and values

#include <avr/io.h>        // avr: AVR device-specific IO definitions
#include <util/delay.h>    // avr: convenience functions for busy-wait delay loops
#include <avr/wdt.h>       // avr: watchdog timer handling
#include <avr/pgmspace.h>  // avr: program space utilities
#include <avr/interrupt.h> // avr: interrupt things
#include <avr/boot.h>      // avr: boot loader support
//#include <avr/signature.h> // avr: signature support


/* ************************************************************************** */
/*!
    \name Initialisation and Status
*/

void ffInit(void);
void ffStatus(char *str, const uint16_t size);

//@}


/* ************************************************************************** */
/*!
    \name Handy Macros
    @{
*/
#ifdef __DOXYGEN__
#  define NULL 0 //!< the null pointer (from stddef.h) \hideinitializer
#endif
#define BIT(bit) (1<<(bit))   //!< bit \hideinitializer
#ifndef UNUSED
#  define UNUSED(foo) (void)foo //!< mark variable as unused to silence compiler warnings\hideinitializer
#endif
#define SETBITS(port, bits)    port |= (bits)   //!< sets the bits \hideinitializer
#define CLRBITS(port, bits)    port &= ~(bits)  //!< clears the bits \hideinitializer
#define TOGBITS(port, bits)    port ^= (bits)   //!< toggles the bits \hideinitializer
#define NUMOF(x) (sizeof(x)/sizeof(*(x)))       //!< number of elements in vector \hideinitializer
#define ENDLESS true          //!< for endless while loops \hideinitializer
#define FALLTHROUGH           //!< switch fall-through marker \hideinitializer
#define __PAD(n) uint8_t __PADNAME(__LINE__)[n]  //!< struct padding macro \hideinitializer
#define __PADFILL { 0 }           //!< to fill structure padding in initialisers \hideinitializer
#define MIN(a, b)  ((b) < (a) ? (b) : (a)) //!< smaller value of a and b \hideinitializer
#define MAX(a, b)  ((b) > (a) ? (b) : (a)) //!< bigger value of a and b \hideinitializer
#define ABS(a) ((a) > 0 ? (a) : -(a)) //!< absolute value \hideinitializer
#define CLIP(x, a, b) ((x) <= (a) ? (a) : ((x) >= (b) ? (b) : (x))) //!< clip value in range [a:b] \hideinitializer
#define STRINGIFY(x) _STRINGIFY(x) //!< preprocessor stringification  \hideinitializer
#define CONCAT(a, b)   _CONCAT(a, b) //!< preprocessor concatenation  \hideinitializer

#ifndef __DOXYGEN__
#  define _STRINGIFY(x) #x
#  define _CONCAT(a, b)  a ## b
#  define ___PADNAME(x) __pad##x
#  define __PADNAME(x) ___PADNAME(x)
#endif
//@}

//! \name Compiler Hints etc.
//@{
#define __PURE()              __attribute__ ((pure))          //!< pure \hideinitializer
#define __IRQ()               __attribute__ ((interrupt))     //!< irq \hideinitializer
#define __WEAK()              __attribute__ ((weak))          //!< weak \hideinitializer
#define __PACKED              __attribute__ ((packed))        //!< packed \hideinitializer
#define __ALIGN(n)            __attribute__ ((aligned (n)))   //!< align \hideinitializer
#ifdef __INLINE
#  undef __INLINE
#endif
#define __INLINE              inline                                 //!< inline \hideinitializer
#define __NOINLINE            __attribute__((noinline))              //!< no inline \hideinitializer
#define __FORCEINLINE         __attribute__((always_inline)) inline  //!< force inline (also with -Os) \hideinitializer
#define __USED                __attribute__((used))                  //!< used \hideinitializer
#define __NORETURN            __attribute__((noreturn))              //!< no return \hideinitializer
#define __PRINTF(six, aix)    __attribute__((format(printf, six, aix))) //!< printf() style func \hideinitializer
#define __SECTION(sec)        __attribute__((section (STRINGIFY(sec)))); //!< place symbol in section \hideinitializer
#define __NAKED               __attribute__((naked))                 //!< naked function \hideinitializer
//@}

/* ************************************************************************** */
/*!
    \name Critical Sections
    @{
*/

//! enter a critical section, asserting that interrupts are off \hideinitializer
#define CS_ENTER do { ffCsEnter();

//! leave a critical section, re-enabling the interrupts if necessary \hideinitializer
#define CS_LEAVE      ffCsLeave(); } while (0)

//@}

void ffCsEnter(void);
void ffCsLeave(void);

/* ************************************************************************** */

// helpers
#define PIN_TO_PORT(pin) CONCAT(PIN_PORT_,pin)
#define PIN_TO_DDR(pin)  CONCAT(PIN_DDR_,pin)
#define PIN_TO_PIN(pin)  CONCAT(PIN_PIN_,pin)
#define PIN_TO_BIT(pin)  CONCAT(PIN_BIT_,pin)
#define PIN_TO_MASK(pin) BIT( CONCAT(PIN_BIT_,pin) )

/*!
    \name Pin Manipulation Macros

    The \c pin arguments to these macros are the Arduino pin number or the AVR
    names prefixed with an underscrore:
    - Arduino: _D1, _D2, ..., _A0, ...
    - AVR: _PD0, _PD1, ...

    Examples:
\code{.c}
// Arduino pin 13 (built-in LED)
PIN_OUTPUT(_D13);
PIN_HIGH(_D13);

// Arduino pin A1
PIN_OUTPUT(_A1);
PIN_LOW(_A1);

// AVR pin PC5
PIN_OUTPUT(_PC5);
PIN_LOW(_PC5);

// Arduino pin 13 (built-in LED) = AVR pin PB5
#define LED_PIN _PB5
PIN_OUTPUT(LED_PIN);
PIN_HIGH(LED_PIN);
\endcode

    @{
*/
//! configure pin for output \hideinitializer
#define PIN_OUTPUT(pin)     SETBITS( PIN_TO_DDR(pin), PIN_TO_MASK(pin) )
//! set output pin high \hideinitializer
#define PIN_HIGH(pin)       SETBITS( PIN_TO_PORT(pin), PIN_TO_MASK(pin) )
//! set output pin low \hideinitializer
#define PIN_LOW(pin)        CLRBITS( PIN_TO_PORT(pin), PIN_TO_MASK(pin) )
//! toggle output pin state \hideinitializer
#define PIN_TOGGLE(pin)     TOGBITS( PIN_TO_PORT(pin), PIN_TO_MASK(pin) )
//! set pin state \hideinitializer
#define PIN_SET(pin, state) if (state) { PIN_HIGH(pin); } else { PIN_LOW(pin); }
//! configure pin for input \hideinitializer
#define PIN_INPUT(pin)      CLRBITS( PIN_TO_DDR(pin), PIN_TO_MASK(pin) )
//! enable pull-up on input pin \hideinitializer
#define PIN_PULLUP_ON(pin)  SETBITS( PIN_TO_PORT(pin), PIN_TO_MASK(pin) )
//! disable pull-up on input pin \hideinitializer
#define PIN_PULLUP_OFF(pin) CLRBITS( PIN_TO_PORT(pin), PIN_TO_MASK(pin) )
//! read input pin state \hideinitializer
#define PIN_GET(pin)        ( PIN_TO_PIN(pin) & PIN_TO_MASK(pin) ? true : false )
//@}

#define PIN_BIT_NONE  9999

#define PIN_PORT__D0  PORTD
#define PIN_DDR__D0   DDRD
#define PIN_BIT__D0   PD0
#define PIN_PIN__D0   PIND
#define PIN_PORT__PD0 PORTD
#define PIN_DDR__PD0  DDRD
#define PIN_BIT__PD0  PD0
#define PIN_PIN__PD0  PIND

#define PIN_PORT__D1  PORTD
#define PIN_DDR__D1   DDRD
#define PIN_BIT__D1   PD1
#define PIN_PIN__D1   PIND
#define PIN_PORT__PD1 PORTD
#define PIN_DDR__PD1  DDRD
#define PIN_BIT__PD1  PD1
#define PIN_PIN__PD1  PIND

#define PIN_PORT__D2  PORTD
#define PIN_DDR__D2   DDRD
#define PIN_BIT__D2   PD2
#define PIN_PIN__D2   PIND
#define PIN_PORT__PD2 PORTD
#define PIN_DDR__PD2  DDRD
#define PIN_BIT__PD2  PD2
#define PIN_PIN__PD2  PIND

#define PIN_PORT__D3  PORTD
#define PIN_DDR__D3   DDRD
#define PIN_BIT__D3   PD3
#define PIN_PIN__D3   PIND
#define PIN_PORT__PD3 PORTD
#define PIN_DDR__PD3  DDRD
#define PIN_BIT__PD3  PD3
#define PIN_PIN__PD3  PIND

#define PIN_PORT__D4  PORTD
#define PIN_DDR__D4   DDRD
#define PIN_BIT__D4   PD4
#define PIN_PIN__D4   PIND
#define PIN_PORT__PD4 PORTD
#define PIN_DDR__PD4  DDRD
#define PIN_BIT__PD4  PD4
#define PIN_PIN__PD4  PIND

#define PIN_PORT__D5  PORTD
#define PIN_DDR__D5   DDRD
#define PIN_BIT__D5   PD5
#define PIN_PIN__D5   PIND
#define PIN_PORT__PD5 PORTD
#define PIN_DDR__PD5  DDRD
#define PIN_BIT__PD5  PD5
#define PIN_PIN__PD5  PIND

#define PIN_PORT__D6  PORTD
#define PIN_DDR__D6   DDRD
#define PIN_BIT__D6   PD6
#define PIN_PIN__D6   PIND
#define PIN_PORT__PD6 PORTD
#define PIN_DDR__PD6  DDRD
#define PIN_BIT__PD6  PD6
#define PIN_PIN__PD6  PIND

#define PIN_PORT__D7  PORTD
#define PIN_DDR__D7   DDRD
#define PIN_BIT__D7   PD7
#define PIN_PIN__D7   PIND
#define PIN_PORT__PD7 PORTD
#define PIN_DDR__PD7  DDRD
#define PIN_BIT__PD7  PD7
#define PIN_PIN__PD7  PIND

#define PIN_PORT__D8  PORTB
#define PIN_DDR__D8   DDRB
#define PIN_BIT__D8   PB0
#define PIN_PIN__D8   PINB
#define PIN_PORT__PB0 PORTB
#define PIN_DDR__PB0  DDRB
#define PIN_BIT__PB0  PB0
#define PIN_PIN__PB0  PINB

#define PIN_PORT__D9  PORTB
#define PIN_DDR__D9   DDRB
#define PIN_BIT__D9   PB1
#define PIN_PIN__D9   PINB
#define PIN_PORT__PB1 PORTB
#define PIN_DDR__PB1  DDRB
#define PIN_BIT__PB1  PB1
#define PIN_PIN__PB1  PINB

#define PIN_PORT__D10 PORTB
#define PIN_DDR__D10  DDRB
#define PIN_BIT__D10  PB2
#define PIN_PIN__D10  PINB
#define PIN_PORT__PB2 PORTB
#define PIN_DDR__PB2  DDRB
#define PIN_BIT__PB2  PB2
#define PIN_PIN__PB2  PINB

#define PIN_PORT__D11  PORTB
#define PIN_DDR__D11   DDRB
#define PIN_BIT__D11   PB3
#define PIN_PIN__D11   PINB
#define PIN_PORT__PB3 PORTB
#define PIN_DDR__PB3  DDRB
#define PIN_BIT__PB3  PB3
#define PIN_PIN__PB3  PINB

#define PIN_PORT__D12  PORTB
#define PIN_DDR__D12   DDRB
#define PIN_BIT__D12   PB4
#define PIN_PIN__D12   PINB
#define PIN_PORT__PB4 PORTB
#define PIN_DDR__PB4  DDRB
#define PIN_BIT__PB4  PB4
#define PIN_PIN__PB4  PINB

#define PIN_PORT__D13  PORTB
#define PIN_DDR__D13   DDRB
#define PIN_BIT__D13   PB5
#define PIN_PIN__D13   PINB
#define PIN_PORT__PB5 PORTB
#define PIN_DDR__PB5  DDRB
#define PIN_BIT__PB5  PB5
#define PIN_PIN__PB5  PINB

#define PIN_PORT__A0  PORTC
#define PIN_DDR__A0   DDRC
#define PIN_BIT__A0   PC0
#define PIN_PIN__A0   PINC
#define PIN_PORT__PC0 PORTC
#define PIN_DDR__PC0  DDRC
#define PIN_BIT__PC0  PC0
#define PIN_PIN__PC0  PINC

#define PIN_PORT__A1  PORTC
#define PIN_DDR__A1   DDRC
#define PIN_BIT__A1   PC1
#define PIN_PIN__A1   PINC
#define PIN_PORT__PC1 PORTC
#define PIN_DDR__PC1  DDRC
#define PIN_BIT__PC1  PC1
#define PIN_PIN__PC1  PINC

#define PIN_PORT__A2  PORTC
#define PIN_DDR__A2   DDRC
#define PIN_BIT__A2   PC2
#define PIN_PIN__A2   PINC
#define PIN_PORT__PC2 PORTC
#define PIN_DDR__PC2  DDRC
#define PIN_BIT__PC2  PC2
#define PIN_PIN__PC2  PINC

#define PIN_PORT__A3  PORTC
#define PIN_DDR__A3   DDRC
#define PIN_BIT__A3   PC3
#define PIN_PIN__A3   PINC
#define PIN_PORT__PC3 PORTC
#define PIN_DDR__PC3  DDRC
#define PIN_BIT__PC3  PC3
#define PIN_PIN__PC3  PINC

#define PIN_PORT__A4  PORTC
#define PIN_DDR__A4   DDRC
#define PIN_BIT__A4   PC4
#define PIN_PIN__A4   PINC
#define PIN_PORT__PC4 PORTC
#define PIN_DDR__PC4  DDRC
#define PIN_BIT__PC4  PC4
#define PIN_PIN__PC4  PINC

#define PIN_PORT__A5  PORTC
#define PIN_DDR__A5   DDRC
#define PIN_BIT__A5   PC5
#define PIN_PIN__A5   PINC
#define PIN_PORT__PC5 PORTC
#define PIN_DDR__PC5  DDRC
#define PIN_BIT__PC5  PC5
#define PIN_PIN__PC5  PINC


/* ************************************************************************** */
/*!
    \name Serial Receive and Transmit
    @{
*/

//! serial transmit buffer size (set to 0 to disable serial transmit)
#if !defined(FF_TX_BUFSIZE) || defined(__DOXYGEN__)
#  define FF_TX_BUFSIZE 125
#endif

//! serial receive buffer size (set to 0 to disable serial receive)
#if !defined(FF_RX_BUFSIZE) || defined(__DOXYGEN__)
#  define FF_RX_BUFSIZE 0
#endif

//! serial baud rate
#if !defined(FF_RXTX_BAUDRATE) || defined(__DOXYGEN__)
#  define FF_RXTX_BAUDRATE 115200
#endif

uint8_t ffGetRxBufSize(void);

char ffReadNextChar(void);

//@}

/* ************************************************************************** */
/*!
    \name Debugging

    \note This will only work if #FF_TX_BUFSIZE is > 0.

    @{
*/

//! prints a notice
/*!
    Prints the formatted notice message, automatically
    prepended with "N: ".

    \param fmt   the format
    \param args  zero or more arguments

    \hideinitializer
*/
#define NOTICE(fmt, args...)  printf_P(PSTR("N: " fmt "\n"), ## args)


//! prints a print
/*!
    Prints the formatted print message, automatically
    prepended with "N: ".

    \param fmt   the format
    \param args  zero or more arguments

    \hideinitializer
*/
#define PRINT(fmt, args...) printf_P(PSTR("P: " fmt "\n"), ## args)


//! prints a warning
/*!
    Prints the formatted warning message, automatically
    prepended with "W: ".

    \param fmt   the format
    \param args  zero or more arguments

    \hideinitializer
*/
#define WARNING(fmt, args...) printf_P(PSTR("W: " fmt "\n"), ## args)

//! prints an error
/*!
    Prints the formatted error message, automatically
    prepended with "E: ".

    \param fmt   the format
    \param args  zero or more arguments

    \hideinitializer
*/
#define ERROR(fmt, args...) printf_P(PSTR("E: " fmt "\n"), ## args)

//! enable DEBUG() output (set to 0 to compile-out all DEBUG() calls)
#if !defined(FF_DEBUG_ENABLED) || defined(__DOXYGEN__)
#define FF_DEBUG_ENABLED 1
#endif

//! prints a debug message
/*!
    Prints the formatted debug message (if #FF_DEBUG_ENABLED > 0),
    automatically prepended with "D: ".

    \param fmt   the format
    \param args  zero or more arguments

    \hideinitializer
*/
#if ( (FF_DEBUG_ENABLED > 0) || defined(__DOXYGEN__) )
#  define DEBUG(fmt, args...) printf_P(PSTR("D: " fmt "\n"), ## args)
#else
#  define DEBUG(...) /* nothing */
#endif

//@}

/* ************************************************************************** */
/*!
    \name Utility Functions
    @{
*/

//! number of ffTic() / ffToc() registers (set to 0 to disable the functionality and free Timer/Counter1 hardware)
#if !defined(FF_NUM_TICTOC) || defined(__DOXYGEN__)
#  define FF_NUM_TICTOC 1
#endif

//! start runtime measurement
/*!
    Starts a runtime measurement.

    \param[in] reg  register (< #FF_NUM_TICTOC)
*/
void ffTic(const uint8_t reg);

//! stop runtime measurement
/*!
    Stops a runtime measurement and return the measurement.

    \param[in] reg  register (< #FF_NUM_TICTOC)

    \return  the measurement [us], or 0 if \c reg > #FF_NUM_TICTOC

    \note The result wraps at 32.768ms.
*/
uint16_t ffToc(const uint8_t reg);

//@}


/* ***** analog to digital conversion (ADC) ********************************* */
/*!
    \name Analog to Digital Conversion (ADC)
    @{
*/

//! ADC input pins
typedef enum FF_ADC_e
{
    FF_ADC_A0  = BIT(0), //!< pin A0 (PC0)
    FF_ADC_PC0 = BIT(0), //!< pin PC0 (A0)
    FF_ADC_A1  = BIT(1), //!< pin A1 (PC1)
    FF_ADC_PC1 = BIT(1), //!< pin PC1 (A1)
    FF_ADC_A2  = BIT(2), //!< pin A2 (PC2)
    FF_ADC_PC2 = BIT(2), //!< pin PC2 (A2)
    FF_ADC_A3  = BIT(3), //!< pin A3 (PC3)
    FF_ADC_PC3 = BIT(3), //!< pin PC3 (A3)
    FF_ADC_A4  = BIT(4), //!< pin A4 (PC4)
    FF_ADC_PC4 = BIT(4), //!< pin PC4 (A4)
    FF_ADC_A5  = BIT(5), //!< pin A5 (PC5)
    FF_ADC_PC5 = BIT(5), //!< pin PC5 (A5)
} FF_ADC_t;

//! initialise ADC hardware and configure pins
/*!
    \param[in] pins     mask of pins to configure
    \param[in] useAref  \c true if AREF pin is supplied with a reference voltage,
                        \c false if normal V_CC is to be used (typically: false)
*/
void ffAdcInit(const FF_ADC_t pins, const bool useAref);

//! read input on ADC pin and scale output value accordingly
/*!
    \param pin  the pin to read
    \param min  minimum value to output
    \param max  maximum value to output

    \returns value scaled from min..max, or raw ADC value if min == max
*/
int32_t ffAdcGetScaled(const FF_ADC_t pin, const int32_t min, const int32_t max);

//@}

/* ************************************************************************** */

#endif // __FFSTUFF_H__

//@}
// eof
