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

    \addtogroup FFSTUFF

    @{
*/

#include "ffstuff.h"

/* ***** serial port input and output *************************************** */

#if ( (FF_RX_BUFSIZE > 0) && (FF_RX_BUFSIZE < 50) )
#  error FF_RX_BUFSIZE must be >= 50 bytes or it will not be useful.
#endif
#if (FF_RX_BUFSIZE > 255)
#  error FF_RX_BUFSIZE must be <= 255.
#endif

#if ( (FF_TX_BUFSIZE > 0) && (FF_TX_BUFSIZE < 50) )
#  error FF_TX_BUFSIZE must be >= 50 bytes or it will not be useful.
#endif
#if (FF_TX_BUFSIZE > 255)
#  error FF_TX_BUFSIZE must be <= 255.
#endif

//------------------------------------------------------------------------------

#if ( (FF_RX_BUFSIZE > 0) || (FF_TX_BUFSIZE > 0) )

// calculate baudrate parameters
#  define BAUD FF_RXTX_BAUDRATE
#  define BAUD_TOL 5 // STFU, utils/setbaud.h!
#  include <util/setbaud.h> // avr: helper macros for baud rate calculations

static void sFfRxTxInit(void)
{
    // disable all while setting baud rate
    UCSR0A = 0x00;
    UCSR0B = 0x00;

    // set baudrate
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#   if USE_2X
    SETBITS(UCSR0A, BIT(U2X0));
#   else
    CLRBITS(UCSR0A, BIT(U2X0));
#   endif

    // set mode to asynchronous USART, no parity, 1 stop bit, 8bit (i.e. 8N1)
    UCSR0C = BIT(UCSZ00) | BIT(UCSZ01);

    // enable transmitter & receiver
    SETBITS(UCSR0B, 0
#   if (FF_TX_BUFSIZE > 0)
        | BIT(TXEN0)
#   endif
#   if (FF_RX_BUFSIZE > 0)
        | BIT(RXEN0)
#   endif
        );
}

#else
static void sFfRxTxInit(void) { }
#endif // ( (FF_RX_BUFSIZE > 0) || (FF_TX_BUFSIZE > 0) )

//------------------------------------------------------------------------------

#if (FF_TX_BUFSIZE > 0)

static volatile char     svFfTxBuf[FF_TX_BUFSIZE]; // output buffer
static volatile uint8_t  svFfTxBufHead;            // write-to-buffer pointer (index)
static volatile uint8_t  svFfTxBufTail;            // read-from-buffer pointer (index)
static volatile uint8_t  svFfTxBufSize;            // size of buffered data
static volatile uint8_t  svFfTxBufPeak;            // peak output buffer size

// adds a character to the tx buffer
static int16_t sFfOutputPutChar(char c, FILE *pFile)
{
    UNUSED(pFile);

    // interrupts enabled
    if (SREG & BIT(7))
    {
        // wait for space in the buffer
        while (svFfTxBufSize == sizeof(svFfTxBuf))
        {
            // busy-wait
        }

        CS_ENTER;

        // add to buffer
        svFfTxBuf[svFfTxBufHead] = c;
        svFfTxBufHead += 1;
        svFfTxBufHead %= sizeof(svFfTxBuf);
        svFfTxBufSize++;
        if (svFfTxBufSize > svFfTxBufPeak)
        {
            svFfTxBufPeak = svFfTxBufSize;
        }
        SETBITS(UCSR0B, BIT(UDRIE0));

        CS_LEAVE;
    }
    // interrupts are not enabled
    else
    {
        // send the char right away
        CLRBITS(UCSR0B, BIT(UDRIE0));
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = c;
    }

    return 0;
}

// the output file handle (write-only)
static FILE sFfOutputDev = FDEV_SETUP_STREAM(sFfOutputPutChar, NULL, _FDEV_SETUP_WRITE);

ISR(USART_UDRE_vect)
{
    // load next char
    if (svFfTxBufSize != 0) // (svFfTxBufHead != svFfTxBufTail)
    {
        const char c = /*~*/svFfTxBuf[svFfTxBufTail];
        svFfTxBufTail += 1;
        svFfTxBufTail %= sizeof(svFfTxBuf);
        svFfTxBufSize--;
        UDR0 = c;
    }
    else
    {
        CLRBITS(UCSR0B, BIT(UDRIE0));
    }
}

static void sFfTxInit(void)
{
    // setup tx debugging
    SETBITS(DDRD, BIT(PD1)); // TXD output
    SETBITS(PORTD, BIT(PD1));

    // initialise output
    stdout = &sFfOutputDev;      // assign the debug port to stdout
    svFfTxBufHead = 0;           // initialise the output buffer
    svFfTxBufTail = 0;
    svFfTxBufSize = 0;
    svFfTxBufPeak = 0;
}

#else
static void sFfTxInit(void) { }
#endif // (FF_TX_BUFSIZE > 0)

//------------------------------------------------------------------------------

#if (FF_RX_BUFSIZE > 0)

static volatile char      svFfRxBuf[FF_RX_BUFSIZE]; // data rx buffer
static volatile uint8_t   svFfRxBufHead;            // write-to-buffer pointer (index)
static volatile uint8_t   svFfRxBufTail;            // read-from-buffer pointer (index)
static volatile uint8_t   svFfRxBufSize;            // size of buffered data
static volatile uint8_t   svFfRxBufPeak;            // peak input buffer size
static volatile uint16_t  svFfRxBufDrop;            // number of dropped bytes

// reads a character from the rx buffer
static char sFfInputGetChar(FILE *pFile)
{
    UNUSED(pFile);

    // wait until character is available
    while (svFfRxBufSize == 0)
    {
        // busy-wait
    }

    char c;

    CS_ENTER;

    // get char
    c = /*~*/svFfRxBuf[svFfRxBufTail];
    // move tail
    svFfRxBufTail += 1;
    svFfRxBufTail %= sizeof(svFfRxBuf);
    svFfRxBufSize--;

    CS_LEAVE;

    return c;
}

// the input file handle (read-only)
static FILE sFfInputDev = FDEV_SETUP_STREAM(NULL, sFfInputGetChar, _FDEV_SETUP_READ);

ISR(USART_RX_vect) // UART, rx complete
{
    const char c = UDR0; // always read this or this ISR will fire continuously

    if ( (svFfRxBufSize == 0) || (svFfRxBufHead != svFfRxBufTail) )
    {
        svFfRxBuf[svFfRxBufHead] = c;
        svFfRxBufHead += 1;
        svFfRxBufHead %= sizeof(svFfRxBuf);
        svFfRxBufSize++;
        if (svFfRxBufSize > svFfRxBufPeak)
        {
            svFfRxBufPeak = svFfRxBufSize;
        }
    }
    else
    {
        svFfRxBufDrop++;
    }
}

__INLINE uint8_t ffGetRxBufSize(void)
{
    return svFfRxBufSize;
}

__INLINE char ffReadNextChar(void)
{
    //return fgetc(stdin);
    return sFfInputGetChar(NULL);
}

static void sFfRxInit(void)
{
    // setup receive pin
    CLRBITS(DDRD, BIT(PD0));
    SETBITS(PORTD, BIT(PD0));

    // initialise input buffer
    stdin = &sFfInputDev;       // assign the input port to stdin
    svFfRxBufHead = 0;          // initialise the output buffer
    svFfRxBufTail = 0;
    svFfRxBufSize = 0;
    svFfRxBufPeak = 0;
    svFfRxBufDrop = 0;

    // enable RX complete interrupt
#if (FF_RX_BUFSIZE > 0)
    SETBITS(UCSR0B, BIT(RXCIE0));
#endif
}

#else
__INLINE uint8_t ffGetRxBufSize(void) { return 0; }
char ffReadNextChar(void) { return '\0'; }
static void sFfRxInit(void) { }
#endif // (FF_RX_BUFSIZE > 0)


/* ***** critical sections ************************************************** */

static uint8_t sFfCriticalNesting = 0;
static uint8_t sFfCriticalState = 0;

__INLINE void ffCsEnter(void)
{
    if (!sFfCriticalNesting)
    {
        sFfCriticalState = SREG;
        cli();
    }
    sFfCriticalNesting++;
}

__INLINE void ffCsLeave(void)
{
    sFfCriticalNesting--;
    if (sFfCriticalNesting == 0)
    {
        //sei();
        SREG = sFfCriticalState;
    }
}


/* ***** utility functions ************************************************** */

#if (FF_NUM_TICTOC > 0)

//#define FF_TICTOC_FREQ (F_CPU/1024)
#define FF_TICTOC_FREQ (F_CPU/8)

static uint16_t sFfTicTocRegs[FF_NUM_TICTOC];

static void sFfTicTocInit(void)
{
    // setup tic/toc performance counter,
    // using 16bit Timer/Counter1

    // normal operation, OC1A/OC1B disconnected.
    TCCR1A = 0;
    // no PWM stuff
    TCCR1C = 0;

    // 16_000_000 / 1024 = 15625 clk/step
    // "1 TC step" = 0.064ms, 65536 steps = 4194ms
    //TCCR1B = BIT(CS12) | BIT(CS10); // prescale clk/1024

    // 16_000_000 / 1 = 16_000_000 clk/step
    // "1 TC step" = 0.0000625, 65536 steps = 4.096ms
    //TCCR1B = BIT(CS10);

    // 16_000_000 / 8 = 2_000_000 clk/step
    // "1 TC step" = 0.0005ms, 65536 steps = 32.768ms
    TCCR1B = BIT(CS11);
}

void ffTic(const uint8_t reg)
{
    if (reg < FF_NUM_TICTOC)
    {
        sFfTicTocRegs[reg] = (uint16_t)TCNT1;
    }
}

uint16_t ffToc(const uint8_t reg)
{
    uint16_t res = 0;
    if (reg < FF_NUM_TICTOC)
    {
        //const uint16_t delta = (uint16_t)TCNT1 - sFfTicTocRegs[reg];
        //res = ((uint32_t)delta * 1000 * 10) / (uint32_t)FF_TICTOC_FREQ;
        const uint16_t delta = (uint16_t)TCNT1 - sFfTicTocRegs[reg];
        res = delta / (FF_TICTOC_FREQ/1000/1000);
    }
    return res;
}

#else
static void sFfTicTocInit(void) { }
void ffTic(const uint8_t reg) { UNUSED(reg); }
uint16_t ffToc(const uint8_t reg) { UNUSED(reg); return 0; }
#endif // (FF_NUM_TICTOC > 0)


/* **** analog to digital conversion (ADC) ********************************** */

void ffAdcInit(const FF_ADC_t pins, const bool useAref)
{
    // setup selected pin(s) for input
    uint8_t didr = 0;
    if (pins & FF_ADC_A0)
    {
        PIN_INPUT(_A0);
        PIN_PULLUP_OFF(_A0);
        didr |= BIT(ADC0D);
    }
    if (pins & FF_ADC_A1)
    {
        PIN_INPUT(_A1);
        PIN_PULLUP_OFF(_A1);
        didr |= BIT(ADC1D);
    }
    if (pins & FF_ADC_A2)
    {
        PIN_INPUT(_A2);
        PIN_PULLUP_OFF(_A2);
        didr |= BIT(ADC2D);
    }
    if (pins & FF_ADC_A3)
    {
        PIN_INPUT(_A3);
        PIN_PULLUP_OFF(_A3);
        didr |= BIT(ADC3D);
    }
    if (pins & FF_ADC_A4)
    {
        PIN_INPUT(_A4);
        PIN_PULLUP_OFF(_A4);
        didr |= BIT(ADC4D);
    }
    if (pins & FF_ADC_A5)
    {
        PIN_INPUT(_A5);
        PIN_PULLUP_OFF(_A5);
        didr |= BIT(ADC5D);
    }

    // disable digital input buffers on pins used for ADC input
    DIDR0 = didr; // Digital Input Disable Register 0

    // select external AREF or normal AVCC
    ADMUX = useAref ?  // ADC Multiplexer Selection Register
        0 :            // AREF, Internal V ref turned off
        BIT(REFS0);    // AV_CC with external capacitor at AREF pin

    ADCSRA =           // ADC Control and Status Register A
      //BIT(ADPS2) | BIT(ADPS1) | BIT(ADPS0); // ADC clock: 16MHz / 128 = 125kHz
      //BIT(ADPS2) | BIT(ADPS1);              // ADC clock: 16MHz / 64 = 250kHz
        BIT(ADPS2)              | BIT(ADPS0); // ADC clock: 16MHz / 32 = 500kHz
      //             BIT(ADPS1) | BIT(ADPS0); // ADC clock: 16MHz / 16 = 1000kHz

    ADCSRB =           // ADC Control and Status Register B
        0;             // free running, no trigger
}

int32_t ffAdcGetScaled(const FF_ADC_t pin, const int32_t min, const int32_t max)
{
    // select ADC channel
    uint8_t mux = 0;
    switch (pin)
    {
        //case FF_ADC_A0:
        case FF_ADC_PC0: mux = 0         | 0         | 0        ; break;
        //case FF_ADC_A1:
        case FF_ADC_PC1: mux = 0         | 0         | BIT(MUX0); break;
        //case FF_ADC_A2:
        case FF_ADC_PC2: mux = 0         | BIT(MUX1) | 0        ; break;
        //case FF_ADC_A3:
        case FF_ADC_PC3: mux = 0         | BIT(MUX1) | BIT(MUX0); break;
        //case FF_ADC_A4:
        case FF_ADC_PC4: mux = BIT(MUX2) | 0         | 0        ; break;
        //case FF_ADC_A5:
        case FF_ADC_PC5: mux = BIT(MUX2) | 0         | BIT(MUX0); break;
    }
    ADMUX = (ADMUX & 0xf0) | mux;

    // start conversion
    ADCSRA |=   // ADC Control and Status Register A
        BIT(ADEN) | // ADC enable
        BIT(ADSC) | // ADC start conversion
        BIT(ADIF);  // clear ADC interrupt flag

    // wait for conversion to finish
    loop_until_bit_is_set(ADCSRA, ADIF);

    if (min == max)
    {
        return (int32_t)ADC;
    }
    else
    {
        return (((int32_t)ADC * (max - min)) / ((int32_t)1023)) + min;
    }
}

/* **** initialisation ****************************************************** */

void ffInit(void)
{
    // wait for stuff to power up, and don't mess with the ISP programmer
    _delay_ms(250);

    sFfRxTxInit();
    sFfTxInit();
    sFfRxInit();

    DEBUG("ff: init (rx %"PRIu8", tx %"PRIu8")", FF_RX_BUFSIZE, FF_TX_BUFSIZE);

    sFfTicTocInit();

    PRINT("flipflip's Arduino Uno Stuff for avr-gcc and avr-libc");
    PRINT("Copyright (c) Philippe Kehl (flipflip at oinkzwurgl dot org)");

    // enable interrupts
    sei();
}

void ffStatus(char *str, const uint16_t size)
{
#if ( (FF_RX_BUFSIZE > 0) && (FF_TX_BUFSIZE > 0) )
    snprintf_P(str, size,
        PSTR("rxbuf=%"PRIu8"/%"PRIu8"/%"PRIu8" (%"PRIu16") txbuf=%"PRIu8"/%"PRIu8"/%"PRIu8),
        svFfRxBufSize, svFfRxBufPeak, sizeof(svFfRxBuf), svFfRxBufDrop,
        svFfTxBufSize, svFfTxBufPeak, sizeof(svFfTxBuf));
    svFfRxBufPeak = 0;
    svFfRxBufDrop = 0;
    svFfTxBufPeak = 0;
#elif (FF_RX_BUFSIZE > 0)
    snprintf_P(str, size,
        PSTR("rxbuf=%"PRIu8"/%"PRIu8"/%"PRIu8" (%"PRIu16")"),
        svFfRxBufSize, svFfRxBufPeak, sizeof(svFfRxBuf), svFfRxBufDrop);
    svFfRxBufPeak = 0;
    svFfRxBufDrop = 0;
#elif (FF_TX_BUFSIZE > 0)
    snprintf_P(str, size,
        PSTR("txbuf=%"PRIu8"/%"PRIu8"/%"PRIu8""),
        svFfTxBufSize, svFfTxBufPeak, sizeof(svFfTxBuf));
    svFfTxBufPeak = 0;
#endif
    str[size-1] = '\0';
}

/* ************************************************************************** */
//@}
// eof
