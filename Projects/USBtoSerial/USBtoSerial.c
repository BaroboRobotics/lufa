/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include <stdbool.h>
#include <util/delay.h>
#include "USBtoSerial.h"
#include <avr/pgmspace.h>

// For accessing bootloader //
#define BOOT_START_ADDR 0x7000
typedef void (*BootPtr_t) (void) __attribute__ ((noreturn));
BootPtr_t BootStartPtr = (BootPtr_t) BOOT_START_ADDR;
//////////////////////////////////////////////////////

// assign ports & pins
#define LEDPORT     PORTB
#define LEDRED      PB5
#define LEDBLUE     PB6
#define MRSTPORT    PORTB
#define MRST        PB7
#define PWRCTRLPORT	PORTD 
#define PWRCTRL     PD4 
#define PWRBTNPORT	PORTE
#define PWRBTNPIN   PINE
#define PWRBTN      PE6
#define VBCTRL		  PF1
#define VBSNS		    PF0
#define TXM			    PD2
#define RXM			    PD3

// define constants
#define TIME_PWRUP  1000
#define TIME_PWRDN	2000
#define TIME_DEBOUNCE 50
#define PWRBTNMSK   (1<<PWRBTN)
#define T1PS256     (1<<CS12) // T1 prescaler=256
#define T1PS1024    (1<<CS12)|(1<<CS10) // T1 prescaler=1024
#define T1PS        T1PS1024
#define T1PSVAL     1024
#define VBATTLOW	3.25
#define ADCVBATTLOW	(VBATTLOW/4.2)*(2.7/3.3)*1023//709
#define VBATTGOOD	3.6
#define ADCVBATTGOOD (VBATTGOOD/4.2)*(2.7/3.3)*1023//775
#define VBATTCRITICAL 3.0
#define ADCVBATTCRITICAL (VBATTCRITICAL/4.2)*(2.7/3.3)*1023
#define VBATTMINTURNON 3.05
#define ADCVBATTMINTURNON (VBATTMINTURNON/4.2)*(2.7/3.3)*1023
#define ADCMSK		0x1F

#define T1PWRUP   ( (float)(TIME_PWRUP / 1000) ) * (F_CPU / T1PSVAL) // 2sec=15625
#define T1PWRDN		( (float)(TIME_PWRDN / 1000) ) * (F_CPU / T1PSVAL)

// Pin control macros
#define PWRBTNDN		!(PWRBTNPIN & PWRBTNMSK)
#define PWRCTRL_ENABLE	PWRCTRLPORT |= (1<<PWRCTRL)
#define PWRCTRL_DISABLE	PWRCTRLPORT &= ~(1<<PWRCTRL)
#define MRST_ENABLE	MRSTPORT &= ~(1<<MRST)
#define MRST_DISABLE	MRSTPORT |= (1<<MRST)
#define LEDBLUE_ON		LEDPORT &= ~(1<<LEDBLUE)
#define LEDBLUE_OFF		LEDPORT |= (1<<LEDBLUE)
#define LEDRED_ON		LEDPORT &= ~(1<<LEDRED)
#define LEDRED_OFF		LEDPORT |= (1<<LEDRED)

#define ESCSTRLEN 10
#define ESCCHAR1 0xAA
#define ESCCHAR2 0xA5

// Prototypes
static void PwrInit(void);
static void IO_Init(void);
static void startBtnTimer(uint8_t ps);
static void stopBtnTimer(void);
static _Bool validBtnTime(uint16_t time_ms);
/*
static void MRST_enable(void);
static void MRST_disable(void);
static void pwrctrl_enable(void);
static void pwrctrl_disable(void);
static void LED_blue_on(void);
static void LED_blue_off(void);
static void LED_red_on(void);
static void LED_red_off(void);
*/
static void chkPwrBtn(void);
static _Bool pwrBtnDn(void);
static void chkVbatt(void);
static void updateLEDs(void);
static uint16_t get_adc(uint8_t channel);
static void blink(uint8_t x);

typedef enum {
  BTNUP,
  BTNDN
} BtnStates;

typedef enum {
  PWRUP,
  PWRDN
} PwrStates;

typedef enum {
  GOODBATT,
  LOWBATT
} BattStates;

typedef enum {
  ESCNORMAL,
  ESCWAIT
} EscStates;

static BtnStates lastBtnState;
static BattStates lastBattState;
static PwrStates pwrState;
static _Bool justBooted;
static EscStates lastEscState;

static const char escStr[ESCSTRLEN] = {'!', 0x12, 0x10, 0x01, 0x30, 0x12, 0x16, 0x09, 0x12, '!'};
static uint8_t escIndex = 0;

///////////////////////////////////////////////

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[128];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  IO_Init(); // initialize power-related pins
  PwrInit();
  SetupHardware();

  RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
  RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

  LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
  GlobalInterruptEnable();

  lastEscState = ESCNORMAL;

  while (pwrState == PWRUP)
  {
    /* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
    if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
    {
      int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
      
      /*if (ReceivedByte == ESCCHAR1 && lastEscState == ESCNORMAL) 
        lastEscState = ESCWAIT;
      else if (ReceivedByte == ESCCHAR2 && lastEscState == ESCWAIT) {
        blink(3);
        //USB_Detach(); // in USBController_AVR8.c
        if (USB_DeviceState == DEVICE_STATE_Configured)
          RingBuffer_Insert(&USARTtoUSB_Buffer, 0x00);
        //BootStartPtr();
      }
      */
      /* Read bytes from the USB OUT endpoint into the USART transmit buffer */
      //else 
      if (!(ReceivedByte < 0)) {
        if (ReceivedByte == escStr[escIndex]) {
          if (escIndex < ESCSTRLEN-1) {
            escIndex++;
          }
          else {
            // send ack of esc string received
            if (USB_DeviceState == DEVICE_STATE_Configured)
              RingBuffer_Insert(&USARTtoUSB_Buffer, 0x01);
            escIndex = 0;
            blink(3);
          }
          
        }
        else
          escIndex = 0;
      
        RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
        
      }
    }

		/* Check if the UART receive buffer flush timer has expired or the buffer is nearly full */
		uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		if (BufferCount)
		{
			Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

			/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
			 * until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
			if (Endpoint_IsINReady())
			{
				/* Never send more than one bank size less one byte to the host at a time, so that we don't block
				 * while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
				uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

				/* Read bytes from the USART receive buffer into the USB IN endpoint */
				while (BytesToSend--)
				{
					/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
											RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}

					/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
					RingBuffer_Remove(&USARTtoUSB_Buffer);
				}
			}
		}

		/* Load the next byte from the USART transmit buffer into the USART */
		if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer)))
		  Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
		chkPwrBtn();
		chkVbatt();
		updateLEDs();
    //if ( pwrBtnDn() ) PORTB |= (1<<PB5);
    //else PORTB &= (1<<PB5);
	}

  USB_Disable();
  MRST_DISABLE;
  PWRCTRL_DISABLE;
}

static void PwrInit(void)
{
  _Bool validBtnPress = false;
  lastBtnState = BTNUP;
  pwrState = PWRDN;
  justBooted = true;
  uint16_t vbatt;

  do {
    vbatt = get_adc(0);
    if (vbatt < ADCVBATTMINTURNON) 
    {
      LEDRED_ON;
      LEDBLUE_OFF;
      validBtnPress = false;
    }
    else
    {
      LEDRED_ON;
      LEDBLUE_ON;

      if(lastBtnState == BTNUP)
      {
        if( pwrBtnDn() ) {
          lastBtnState = BTNDN; // change lastBtnState
          startBtnTimer( (uint8_t)T1PS1024 ); // start debounce timer
        }
      }
      if(lastBtnState == BTNDN)
      {
        if( pwrBtnDn() ) {
          if( validBtnTime( (uint16_t)T1PWRUP ) ) {
            validBtnPress = true;
            stopBtnTimer();
          }
        }
        else 
        {
          lastBtnState = BTNUP; // start over if button released early
        }
      }
    }
  } while(!validBtnPress);

  PWRCTRL_ENABLE; // pull PWRCTRL high to keep power on
  MRST_ENABLE; // take master AVR out of reset  
  pwrState = PWRUP; // change state to PWRUP
  
} // PwrInit()

static void IO_Init(void)
{
  // setup system clock prescaler to run at 8MHz
  CLKPR = (1<<CLKPCE);
  CLKPR = (1<<CLKPS0);
  
  // Setup Port B
  PORTB = (1<<MRST)|(1<<LEDBLUE)|(1<<LEDRED); // put master AVR in reset
  // PB7=MRST (OUT); PB6=LEDBLUE (OUT); PB5=LEDRED (OUT)
  DDRB = (1<<LEDBLUE)|(1<<LEDRED)|(1<<MRST);
  PORTB |= (1<<MRST);

  // Setup Port E
  // enable PWRBTN pull-up
  PORTE = (1<<PWRBTN);
  // PE6=PWRBTN (IN); PE2=VBCTRL (OUT)
  //DDRE = (1<<VBCTRL);
  
  // Setup port D
  // PD4=PWRCTRL (OUT), PD3=RXM (OUT), PD2=TXM (IN)
  DDRD = (1<<RXM)|(1<<PWRCTRL);  
  // initially keep PWRCTRL LOW to keep power off until turn-on time reached
  
  // Setup Port F
  PORTF = (1<<VBCTRL); // enable VBATT sense line
  // PF1=VBCTRL (OUT), PF0=VBSNS (IN)
  DDRF = (1<<VBCTRL);

  // ADC setup
  // REFS0: use AVcc as voltage reference
  // ADLAR: left-adjusted data for using only 8 highest bits (ADCH)
  // ADC channel (MUX[3:0]) will be chosen later
  ADMUX |= (1<<REFS0); // | (1<<ADLAR); 
  //PRR &= (0<<PRADC);
  // enable ADC, max ADC prescaler
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); 
  // ADCSRA |= (1<<ADATE) // uncomment this line to enable auto-triggering of ADC (see ADCSRB)
	
  // digital input buffers not needed for ADC0, so disable to save power
  //DIDR0 = (1<<ADC0D);
	
}

static void blink(uint8_t x)
{

  for(uint8_t i=0; i<x; i++)
  {
    LEDRED_ON;
    LEDBLUE_OFF;
    _delay_ms(100);
    LEDRED_OFF;
    LEDBLUE_ON;
    _delay_ms(100);
  }
}

static uint16_t get_adc(uint8_t channel)
{
	uint16_t adcresult;
	
	// clear old channel bits, write new channel bits
	ADMUX = (ADMUX & ~ADCMSK) | (channel & ADCMSK);
	ADCSRA |= (1<<ADSC); // start conversion
	while(ADCSRA & (1<<ADSC)) {} // wait for conversion to finish
	adcresult = ADCL;
	adcresult |= ADCH << 8;
	return adcresult; // return 16-bit ADC result
}

static void chkVbatt(void)
{
  uint16_t vbatt = get_adc(0);
  if ((lastBattState == GOODBATT) && (vbatt < ADCVBATTLOW)) lastBattState = LOWBATT;
  else if ((lastBattState == LOWBATT) && (vbatt > ADCVBATTGOOD)) lastBattState = GOODBATT;
  if (vbatt < ADCVBATTCRITICAL) pwrState = PWRDN;
} // chkVbatt()

static void updateLEDs(void)
{
  if (pwrState == PWRDN) {
    LEDRED_OFF;
    LEDBLUE_OFF;
  } else if (lastBtnState == BTNDN && !justBooted) {
    PORTB &= ~((1<<LEDRED)|(1<<LEDBLUE));//LEDRED_ON;
    //LEDBLUE_ON;
  } else if (lastBattState == GOODBATT) {
    LEDRED_OFF;
    LEDBLUE_ON;
  } else if (lastBattState == LOWBATT) {
    LEDRED_ON;
    LEDBLUE_OFF;
  }
} // updateLEDs()

static void chkPwrBtn(void)
{
  
  if (!justBooted)
  {
    if ( (lastBtnState == BTNUP) && pwrBtnDn() ) {//PWRBTNDN)) {
	  lastBtnState = BTNDN;
	  startBtnTimer( (uint8_t) T1PS1024 );
	  }
	  else // lastBtnState = BTNDN
	  {
	    // cancel timer and change state if released early
	    if (!pwrBtnDn()) {
        lastBtnState = BTNUP;
        stopBtnTimer();
      // if button still pressed check for valid time
      } else if (validBtnTime( (uint16_t)T1PWRDN )) {
        //MRST_DISABLE;
        //PWRCTRL_DISABLE;
        pwrState = PWRDN;
      }
    }
  }
  else if(!pwrBtnDn()) // only execute immediately after bootup
  {
    // change state when PwrBtn is released after initial boot
    justBooted = false;
    lastBtnState = BTNUP;
  }
  
} // chkPwrBtn()

static _Bool pwrBtnDn(void)
{
  uint8_t pbpin;
  _Bool pbdn;

  pbpin = PWRBTNPIN;
  pbdn = !(pbpin & (1<<PWRBTN));

  return pbdn;
}

static void startBtnTimer(uint8_t ps)
{
  TCNT1 = 0;
  TCCR1B = ps; // turn on Timer1, prescaler=1024
  //TIMSK4 = (1<<TOIE4); // enable Timer4 overflow interrupt
} // startBtnTimer()

static void stopBtnTimer(void)
{
  TCCR1B = 0; // disable Timer1
} // stopBtnTimer()

static _Bool validBtnTime(uint16_t time_ms)
{
  uint16_t i;
  
  i = TCNT1;
  if (i > time_ms) return true;
  return false;
} // validBtnTime()


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_2);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState == DEVICE_STATE_Configured)
	  RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Set the new baud rate before configuring the USART */
  /*
#if F_CPU==16000000
	if (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS != 57600)
	  UBRR1  = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
	else
	  UBRR1 = 33;
#elif F_CPU==8000000
  UBRR1 = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
#else
  #error F_CPU ERROR
#endif
*/
  UBRR1 = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}

