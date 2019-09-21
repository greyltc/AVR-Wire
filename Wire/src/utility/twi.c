/*
  twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

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

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include "Arduino.h" // for digitalWrite

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "pins_arduino.h"
#include "twi.h"

static volatile bool twi_inRepStart;			// in the middle of a repeated start

/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
  // initialize state
  twi_inRepStart = false;
  
  // activate internal pullups for twi.
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module
  TWCR = _BV(TWEN);
}

/* 
 * Function twi_disable
 * Desc     disables twi pins
 * Input    none
 * Output   none
 */
void twi_disable(void)
{
  // disable twi module, acks, and twi interrupt
  TWCR &= ~(_BV(TWEN));

  // deactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
}

/* 
 * Function twi_setClock
 * Desc     sets twi bit rate
 * Input    Clock Frequency
 * Output   none
 */
void twi_setFrequency(uint32_t frequency)
{
  TWBR = ((F_CPU / frequency) - 16) / 2;
  
  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */
}


// reads bytes from a slave
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, bool sendStop)
{
  uint8_t error = 0xef; // error not cleared
  uint8_t i;
  uint8_t sla_rw = 0x01;
  uint8_t status = 0x00;

  // build sla+w, slave device address + r/~w bit
  sla_rw |= address << 1;

  error = twi_start(sla_rw, twi_inRepStart);

  if(error == 0x00){
    for(i = 1; i < length; ++i){
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // get byte, send ack
      while (!(TWCR & (1<<TWINT))){ // wait for byte
        continue;
      }
      status = (TWSR & 0xF8);
      if (status != 0x50){ // check result
        error = 0xe1; // error sending ack
        break;
      } else {
        data[i-1] = TWDR;
      }
    }
    if (error == 0x00){
      TWCR = (1<<TWINT) | (1<<TWEN); // get last byte, send nack
      while (!(TWCR & (1<<TWINT))){ // wait for byte
        continue;
      }
      status = (TWSR & 0xF8);
      if (status != 0x58){ // check result
        error = 0xe2; // error sending nack for last byte
      } else {
        data[i-1] = TWDR;
      }
    }
    if (sendStop && (error == 0x00)){
      twi_stop();
    } else {
      twi_inRepStart = true;
    }
  }
  if (error != 0x00){
    twi_stop();
    twi_stop();
    twi_stop();
    twi_stop();
  }
  return error;
}

// sends bytes to a slave
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, bool sendStop)
{
  uint8_t error = 0xdf; // error not cleared
  uint8_t i;
  uint8_t sla_rw = 0x00;
  uint8_t status = 0x00;

  // build sla+w, slave device address + r/~w bit
  sla_rw |= address << 1;

  error = twi_start(sla_rw, twi_inRepStart);

  if(error == 0x00){
    for(i = 0; i < length; ++i){
      TWDR = data[i]; // load byte
      TWCR = (1<<TWINT) | (1<<TWEN); // send byte
      while (!(TWCR & (1<<TWINT))){ // wait for send
        continue;
      }
      status = (TWSR & 0xF8);
      if (status != 0x28){ // check result
        error = 0xd1; // slave didn't ack
        break;
      }
    }
    if (sendStop && (error == 0x00)){
      twi_stop();
    } else {
      twi_inRepStart = true;
    }
  }

  if (error != 0x00){
    twi_stop();
    twi_stop();
    twi_stop();
    twi_stop();
  }
  return error;
}


// send a stop to the bus
void twi_stop(void)
{
  // send stop condition
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

  // wait for it to complete
  while (!(TWCR & (1<<TWSTO))){
    continue;
  };
  
  twi_inRepStart = false;
}


// send start to an address
uint8_t twi_start(uint8_t address, bool this_is_repeat)
{
  uint8_t error = 0xff;   // error not cleared
  uint8_t start_code = 0x00;
  uint8_t address_ack_code = 0x00;
  uint8_t status = 0x00;

  if ((0x01 & address) == 0x01){
    address_ack_code = 0x40; // read
  } else {
    address_ack_code = 0x18; // write
  }

  if (this_is_repeat){
    start_code = 0x10;
  } else {
    start_code = 0x08;
  }

  // send start condition
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

  // wait for it to complete
  while (!(TWCR & (1<<TWINT))){
    continue;
  }

  // check result
  status = (TWSR & 0xF8);
  if (status != start_code){
    error = 0xf1; //couldn't send start
  } else {
    // load and send address
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait for it to complete
    while (!(TWCR & (1<<TWINT))){
      continue;
    };

    // check for ack of address+r/~w
    status = (TWSR & 0xF8);
    if (status != address_ack_code){
      error = 0xf2; //slave didn't ack address+r/~w byte
    } else {
      error = 0x00;
    }
  }
  return error; // success
}
