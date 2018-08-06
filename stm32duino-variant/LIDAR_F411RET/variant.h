/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
extern const PinName digitalPin[];

enum {
  PA0,   //D0
  PA1,   //D1
  PA2,   //D2
  PA3,   //D3
  PA4,   //D4
  PA5,   //D5
  PA6,   //D6
  PA7,   //D7
  PA8,   //D8
  PA9,   //D9
  PA10,  //D10
  PA11,  //D11
  PA12,  //D12
  PA13,  //D13
  PA14,  //D14
  PA15,  //D15
  
  PB0,   //D16
  PB1,   //D17
  PB2,   //D18
  PB3,   //D19
  PB4,   //D20
  PB5,   //D21
  PB6,   //D22
  PB7,   //D23
  PB8,   //D24
  PB9,   //D25
  PB10,  //D26
  PB11,  //D27
  PB12,  //D28
  PB13,  //D29
  PB14,  //D30
  PB15,  //D31
  
  PC0,   //D32
  PC1,   //D33
  PC2,   //D34
  PC3,   //D35
  PC4,   //D36
  PC5,   //D37
  PC6,   //D38
  PC7,   //D39
  PC8,   //D40
  PC9,   //D41
  PC10,  //D42
  PC11,  //D43
  PC12,  //D44
  PC13,  //D45
  PC14,  //D46/A0
  PC15,  //D47/A1
  
  // Duplicated pins in order to be aligned with PinMapADC
  PC11a,  //D48/A2
  PC11b,  //D49/A3
  PC11c,  //D50/A4
  PC11d,  //D51/A5
  PC11e,  //D52/A6  = D11
  PC11f,  //D53/A7  = D12
  PC11g,  //D54/A8  = D28
  PC11h,  //D55/A9  = D29
  PC11i,  //D56/A111 = D35
  PC11l,  //D57/A11 = D41
  PC11m,  //D58/A12 = D45
  PEND
};

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS        59
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       13
#define NUM_ANALOG_FIRST        46

// On-board LED pin number
#define LED_BUILTIN             13
#define LED_GREEN               LED_BUILTIN

// On-board user button
#define USER_BTN                PC10

// Timer Definitions
// Do not use timer used by PWM pins when possible. See PinMap_PWM.
#define TIMER_TONE              TIM10

// Do not use basic timer: OC is required
#define TIMER_SERVO             TIM2  //TODO: advanced-control timers don't work

// UART Definitions
#define SERIAL_UART_INSTANCE    2 //Connected to ST-Link
// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           PA3
#define PIN_SERIAL_TX           PA2

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR     Serial
#define SERIAL_PORT_HARDWARE    Serial
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
