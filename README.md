# DPA on a BLAKE-256-based PRF within SPHINCS-256

We propose and implement a DPA on a BLAKE-256-based PRF. We evaluated our attack using physically measured power consumption traces of a SAM3X8E ARM Cortex-M3 executing the code in [blake256.c](blake256.c)

This work was accepted to COSADE 2018: Matthias J. Kannwischer, Aymeric Genêt, Denis Butin, Juliane Krämer, and Johannes Buchmann, "Differential Power Analysis of XMSS and SPHINCS", COSADE 2018, Singapore (to appear)

## Program description

The program runs the BLAKE-256 reference implementation on an embedded device (such as an [Arduino Due board](https://store.arduino.cc/usa/arduino-due)) with an input received from UART and toggles a PIO pin between targeted operations to isolate the capture of their power consumption.

## Dependencies
 - https://www.arduino.cc/en/Main/Software
 - http://www.atmel.com/microsite/atmel-studio/
 - Atmel Software Framework modules:
    - Compiler abstraction layer and code utilities - SAM3/SAM4 implementation
    - System Clock Control - SAM3X/A implementation
    - Delay routines - Common API for SAM, UC3 and XMEGA
    - GPIO - General purpose I/O pins - SAM3X compiler driver
    - IOPORT - General purpose I/O service - SAM3X/A implementation
    - Standard serial I/O (stdio) - SAM implementation
    - PIO - Parallel Input/Output Controller - SAM3X compiler driver
    - TC - Timer Counter (driver)

## Quick start
 - Run Atmel Studio 7 and create a new solution for the device ATSAM3X8E
 - Add the previous list of required modules
 - Add the file [blake256.c](blake256.c) to your solution as a main entry point
 - Build the solution
 - Download the code on your board


 - Open and configure a terminal application with these settings:
    - 115200 bauds
    - 8 bits of data
    - No parity
    - 1 stop bit
    - No flow control
 - Start or reset the application: the `PIO_TRIGGER` pin should toggle for 1 μs
 - Send a full message (see next section for details)
 - With a delay of 1 μs between each targeted operation, the `PIO_TRIGGER` should toggle during the two following operations that occur in the first iteration of `G(0,5,10,15,8)` :
    - `v[0] = (m[8] ^ cst[9]) + (v[5] + v[0])`
    - `v[15] = ROT(v[15] ^ v[0], 16)`

## Communicatation with the device
The program is configured to run the BLAKE-256 hash function with an input received from UART communication. For each sent message, the device will either respond with a specific byte code, or echo the received data.

The general structure of the only command that can be sent is the following: `0xAA [COUNT] [DELAY] [SK1] [ADDR]`.

| Message (hex) | Size (byte) | Description | Response (hex) |
|---------------|-------------|-------------|----------------|
| `0xAA`  | 1  | Transfer initiation code | `0xBB` |
| `COUNT` | 2  | Number of executions | `COUNT` |
| `DELAY` | 2  | Delay (ms) between executions | `COUNT` |
| `SK1`   | 32 | Full SPHINCS-256 first secret key (should be fixed) | - |
| `ADDR`  | 8  | Address of an instance (should change for each execution) | `SK1` `ADDR` |

**Example**: `AA 0001 0000 1F1E1D1C1B1A19181716151413121110000102030405060708090A0B0C0D0E0F DEADBEEF`

### License of the BLAKE-256 implementation

```
BLAKE-256 reference eBASH implementation
author: Jean-Philippe Aumasson (jeanphilippe.aumasson@gmail.com)

Level of copyright protection: 0
Level of patent protection: 0
```

### License of the Atmel Software Framework configuration code

The code for console configuration and the initialisation come from the *Getting-Started Application on SAM - Arduino Due/X* Example Project implemented by Atmel Corporation. Their use is legitimate, according to the license provided with the source code:

```
 Copyright (c) 2011-2016 Atmel Corporation. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. The name of Atmel may not be used to endorse or promote products derived
    from this software without specific prior written permission.

 4. This software may only be redistributed and used in connection with an
    Atmel microcontroller product.

 THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
```
