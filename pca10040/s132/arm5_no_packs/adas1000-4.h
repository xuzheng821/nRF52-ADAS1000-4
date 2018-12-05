/* Copyright (c) 2018 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

 /*
  The purpose of this document is to be purely functional, and not intended as an
  API of any sort. Read the documentation!
 */

#ifndef ADAS1000_4_H__
#define ADAS1000_4_H__
// Includes:
#include "nrf_drv_spi.h"
#include "ble_eeg.h"
#include <stdint.h>

// Number of Registers:
#define ADAS1000_4_NUM_REGS 0x41

// Define Register Addresses:
#define ADAS1000_4_REGADDR_NOP 0x00
#define ADAS1000_4_REGADDR_ECGCTL 0x01
#define ADAS1000_4_REGADDR_LOFFCTL 0x02
#define ADAS1000_4_REGADDR_RESPCTL 0x03
#define ADAS1000_4_REGADDR_PACECTL 0x04
#define ADAS1000_4_REGADDR_CMREFCTL 0x05
#define ADAS1000_4_REGADDR_GPIOCTL 0x06
#define ADAS1000_4_REGADDR_PACEAMPTH 0x07
#define ADAS1000_4_REGADDR_TESTTONE 0x08
#define ADAS1000_4_REGADDR_CALDAC 0x09
#define ADAS1000_4_REGADDR_FRMCTL 0x0A
#define ADAS1000_4_REGADDR_FILTCTL 0x0B
#define ADAS1000_4_REGADDR_LOFFUTH 0x0C
#define ADAS1000_4_REGADDR_LOFFLTH 0x0D
#define ADAS1000_4_REGADDR_PACEEDGETH 0x0E
#define ADAS1000_4_REGADDR_PACELVLTH 0x0F
#define ADAS1000_4_REGADDR_LADATA 0x11
#define ADAS1000_4_REGADDR_LLDATA 0x12
#define ADAS1000_4_REGADDR_RADATA 0x13
#define ADAS1000_4_REGADDR_PACEDATA 0x1A
#define ADAS1000_4_REGADDR_RESPMAG 0x1B
#define ADAS1000_4_REGADDR_RESPPH 0x1C
#define ADAS1000_4_REGADDR_LOFF 0x1D
#define ADAS1000_4_REGADDR_DCLEADOFF 0x1E
#define ADAS1000_4_REGADDR_OPSTAT 0x1F
#define ADAS1000_4_REGADDR_CALLA 0x21
#define ADAS1000_4_REGADDR_CALLL 0x22
#define ADAS1000_4_REGADDR_CALRA 0x23
#define ADAS1000_4_REGADDR_LOAMLA 0x31
#define ADAS1000_4_REGADDR_LOAMLL 0x32
#define ADAS1000_4_REGADDR_LOAMRA 0x32
#define ADAS1000_4_REGADDR_PACE1DATA 0x33
#define ADAS1000_4_REGADDR_PACE2DATA 0x3A
#define ADAS1000_4_REGADDR_PACE3DATA 0x3B
#define ADAS1000_4_REGADDR_FRAMES 0x3C
#define ADAS1000_4_REGADDR_CRC 0x40
#define ADAS1000_4_REGADDR_RSVD 0x41 // RESERVED, DON'T USE.

// Function Prototypes:
void adas_spi_init(void);

void adas_spi_uninit(void);

void adas_powerdown(void);

void adas_powerup(void);

void adas_reset_regs(void);

void adas_write_register(uint8_t addr, uint32_t value);

void adas_read_register(uint8_t addr, uint32_t *value);

void adas_write_default_registers(void);

void adas_read_default_registers(void);

void adas_begin_frame_transactions(void); 

void adas_read_frames(uint8_t number_frames, ble_eeg_t *p_eeg);

#endif



