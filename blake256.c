/**
 * \file blake256.c
 *
 * \brief Power analysis simulation of BLAKE-256 in the case of SPHINCS-256.
 *
 * Copyright (c) 2011-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "delay.h"
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* BLAKE-256 parameters */
typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned char u8;

typedef struct  {
	u32 h[8], s[4], t[2];
	int buflen, nullt;
	u8  buf[64];
} blake256_state;

/* SPHINCS-256 parameters */
#define SEED_BYTES 32
#define ADDR_BYTES 8
#define SUBTREE_HEIGHT 5
#define SUBTREE_MASK ~((1 << SUBTREE_HEIGHT) - 1)

/* UART communication */
#define WAITING_CHAR 0xaa
#define SENDING_CHAR 0xbb

static unsigned char readchar[2] = {'\0', '\0'};

#define READ_SHORT(target) do { \
	readchar[0] = fgetc(stdin); \
	readchar[1] = fgetc(stdin); \
	target = (((uint16_t) readchar[0]) << 8) | (readchar[1]); \
} while (0)

#define SEND_SHORT(target) do { \
	readchar[0] = (unsigned char) (target >> 8); \
	printf("%c", readchar[0]); \
	readchar[1] = (unsigned char) (target & 0x00ff); \
	printf("%c", readchar[1]); \
} while (0)

#define READ_BYTE(target) do { \
	readchar[0] = fgetc(stdin); \
	target = readchar[0]; \
} while (0)

#define SEND_BYTE(target) do { \
	readchar[0] = (unsigned char) (target); \
	printf("%c", readchar[0]); \
} while (0)

#define WAIT_FOR_CHAR do { \
	while ((readchar[0] = fgetc(stdin)) != WAITING_CHAR); \
	printf("%c", SENDING_CHAR); \
} while (0)

/* GPIO toggle */
#define PIO_TRIGGER PIO_PA14_IDX

static int triggered = 0;

#define TRIGGER do { \
	if (triggered) { \
		ioport_set_pin_level(PIO_TRIGGER, 1); \
		ioport_set_pin_level(PIO_TRIGGER, 0); \
		triggered = 0; \
	} else { \
		ioport_set_pin_level(PIO_TRIGGER, 0); \
		ioport_set_pin_level(PIO_TRIGGER, 1); \
		triggered = 1; \
	} \
} while (0)

/* Delay in between targeted operation (time domain isolation) */
#define OP_DELAY_US 1

/**
 *  Configures UART console.
 */

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
	#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
	#endif
		.paritytype = CONF_UART_PARITY,
	#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
	#endif
	};

	/* Configures console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

#define U8TO32(p)					\
(((u32)((p)[0]) << 24) | ((u32)((p)[1]) << 16) |	\
((u32)((p)[2]) <<  8) | ((u32)((p)[3])      ))
#define U32TO8(p, v)					\
(p)[0] = (u8)((v) >> 24); (p)[1] = (u8)((v) >> 16);	\
(p)[2] = (u8)((v) >>  8); (p)[3] = (u8)((v)      );

static const u8 sigma[][16] = {
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 },
	{14,10, 4, 8, 9,15,13, 6, 1,12, 0, 2,11, 7, 5, 3 },
	{11, 8,12, 0, 5, 2,15,13,10,14, 3, 6, 7, 1, 9, 4 },
	{ 7, 9, 3, 1,13,12,11,14, 2, 6, 5,10, 4, 0,15, 8 },
	{ 9, 0, 5, 7, 2, 4,10,15,14, 1,11,12, 6, 8, 3,13 },
	{ 2,12, 6,10, 0,11, 8, 3, 4,13, 7, 5,15,14, 1, 9 },
	{12, 5, 1,15,14,13, 4,10, 0, 7, 6, 3, 9, 2, 8,11 },
	{13,11, 7,14,12, 1, 3, 9, 5, 0,15, 4, 8, 6, 2,10 },
	{ 6,15,14, 9,11, 3, 0, 8,12, 2,13, 7, 1, 4,10, 5 },
	{10, 2, 8, 4, 7, 6, 1, 5,15,11, 9,14, 3,12,13 ,0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 },
	{14,10, 4, 8, 9,15,13, 6, 1,12, 0, 2,11, 7, 5, 3 },
	{11, 8,12, 0, 5, 2,15,13,10,14, 3, 6, 7, 1, 9, 4 },
	{ 7, 9, 3, 1,13,12,11,14, 2, 6, 5,10, 4, 0,15, 8 }};

static const u32 cst[16] = {
	0x243F6A88,0x85A308D3,0x13198A2E,0x03707344,
	0xA4093822,0x299F31D0,0x082EFA98,0xEC4E6C89,
	0x452821E6,0x38D01377,0xBE5466CF,0x34E90C6C,
	0xC0AC29B7,0xC97C50DD,0x3F84D5B5,0xB5470917};

static const u8 padding[] =
	{0x80,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


static void blake256_compress( blake256_state *S, const u8 *block ) {

	u32 v[16], m[16], i, left, right;
	#define ROT(x,n) (((x)<<(32-n))|( (x)>>(n)))
	#define G(a,b,c,d,e)                                  \
	v[a] += (m[sigma[i][e]] ^ cst[sigma[i][e+1]]) + v[b]; \
	v[d] = ROT( v[d] ^ v[a],16);                          \
	v[c] += v[d];                                         \
	v[b] = ROT( v[b] ^ v[c],12);                          \
	v[a] += (m[sigma[i][e+1]] ^ cst[sigma[i][e]])+v[b];   \
	v[d] = ROT( v[d] ^ v[a], 8);                          \
	v[c] += v[d];                                         \
	v[b] = ROT( v[b] ^ v[c], 7);

	#define G_UP(a,b,c,d,e)                               \
	left = v[a] + v[b];                                   \
	right = m[sigma[i][e]] ^ cst[sigma[i][e+1]];          \
	TRIGGER;                                              \
	delay_us(OP_DELAY_US);                                \
	v[a] = left + right;                                  \
	delay_us(OP_DELAY_US);                                \
	TRIGGER;                                              \
	delay_us(OP_DELAY_US);                                \
	TRIGGER;                                              \
	delay_us(OP_DELAY_US);                                \
	left = v[d] ^ v[a];                                   \
	delay_us(OP_DELAY_US);                                \
	TRIGGER;                                              \
	v[d] = ROT( left,16);                                 \
	v[c] += v[d];                                         \
	v[b] = ROT( v[b] ^ v[c],12);                          \
	v[a] += (m[sigma[i][e+1]] ^ cst[sigma[i][e]])+v[b];   \
	v[d] = ROT( v[d] ^ v[a], 8);                          \
	v[c] += v[d];                                         \
	v[b] = ROT( v[b] ^ v[c], 7);

	for(i=0; i<16;++i)  m[i] = U8TO32(block + i*4);
	for(i=0; i< 8;++i)  v[i] = S->h[i];
	v[ 8] = S->s[0] ^ 0x243F6A88;
	v[ 9] = S->s[1] ^ 0x85A308D3;
	v[10] = S->s[2] ^ 0x13198A2E;
	v[11] = S->s[3] ^ 0x03707344;
	v[12] =  0xA4093822;
	v[13] =  0x299F31D0;
	v[14] =  0x082EFA98;
	v[15] =  0xEC4E6C89;
	if (S->nullt == 0) {
		v[12] ^= S->t[0];
		v[13] ^= S->t[0];
		v[14] ^= S->t[1];
		v[15] ^= S->t[1];
	}

	/* Unrolls first loop iteration */
	i = 0;
	G( 0, 4, 8,12, 0 );
	G( 1, 5, 9,13, 2 );
	G( 2, 6,10,14, 4 );
	G( 3, 7,11,15, 6 );
	G( 3, 4, 9,14,14 );
	G( 2, 7, 8,13,12 );
	G_UP( 0, 5,10,15, 8 ); /* TRIGGER */
	G( 1, 6,11,12,10 );

	for (i = 1; i < 14; ++i)
	{
		G( 0, 4, 8,12, 0 );
		G( 1, 5, 9,13, 2 );
		G( 2, 6,10,14, 4 );
		G( 3, 7,11,15, 6 );
		G( 3, 4, 9,14,14 );
		G( 2, 7, 8,13,12 );
		G( 0, 5,10,15, 8 );
		G( 1, 6,11,12,10 );
	}

	for(i=0; i<16;++i)  S->h[i%8] ^= v[i];
	for(i=0; i<8 ;++i)  S->h[i] ^= S->s[i%4];
}


static void blake256_init( blake256_state *S ) {

	S->h[0]=0x6A09E667;
	S->h[1]=0xBB67AE85;
	S->h[2]=0x3C6EF372;
	S->h[3]=0xA54FF53A;
	S->h[4]=0x510E527F;
	S->h[5]=0x9B05688C;
	S->h[6]=0x1F83D9AB;
	S->h[7]=0x5BE0CD19;
	S->t[0]=S->t[1]=S->buflen=S->nullt=0;
	S->s[0]=S->s[1]=S->s[2]=S->s[3] =0;
}


static void blake256_update( blake256_state *S, const u8 *data, u64 datalen ) {

	int left=S->buflen >> 3;
	int fill=64 - left;
		
	if( left && ( ((datalen >> 3) & 0x3F) >= fill ) ) {
		memcpy( (void*) (S->buf + left), (void*) data, fill );
		S->t[0] += 512;
		if (S->t[0] == 0) S->t[1]++;
		blake256_compress( S, S->buf );
		data += fill;
		datalen  -= (fill << 3);
		left = 0;
	}

	while( datalen >= 512 ) {
		S->t[0] += 512;
		if (S->t[0] == 0) S->t[1]++;
		blake256_compress( S, data );
		data += 64;
		datalen  -= 512;
	}
		
	if( datalen > 0 ) {
		memcpy( (void*) (S->buf + left), (void*) data, datalen>>3 );
		S->buflen = (left<<3) + datalen;
	}
	else S->buflen=0;
}


static void blake256_final( blake256_state *S, u8 *digest ) {
		
	u8 msglen[8], zo=0x01, oo=0x81;
	u32 lo=S->t[0] + S->buflen, hi=S->t[1];
	if ( lo < S->buflen ) hi++;
	U32TO8(  msglen + 0, hi );
	U32TO8(  msglen + 4, lo );

	if ( S->buflen == 440 ) { /* one padding byte */
		S->t[0] -= 8;
		blake256_update( S, &oo, 8 );
	} else {
		if ( S->buflen < 440 ) { /* enough space to fill the block  */
			if ( !S->buflen ) S->nullt=1;
			S->t[0] -= 440 - S->buflen;
			blake256_update( S, padding, 440 - S->buflen );
		}
		else { /* need 2 compressions */
			S->t[0] -= 512 - S->buflen;
			blake256_update( S, padding, 512 - S->buflen );
			S->t[0] -= 440;
			blake256_update( S, padding+1, 440 );
			S->nullt = 1;
		}
		blake256_update( S, &zo, 8 );
		S->t[0] -= 8;
	}
	S->t[0] -= 64;
	blake256_update( S, msglen, 64 );
		
	U32TO8( digest + 0, S->h[0]);
	U32TO8( digest + 4, S->h[1]);
	U32TO8( digest + 8, S->h[2]);
	U32TO8( digest +12, S->h[3]);
	U32TO8( digest +16, S->h[4]);
	U32TO8( digest +20, S->h[5]);
	U32TO8( digest +24, S->h[6]);
	U32TO8( digest +28, S->h[7]);
}


static int crypto_hash_blake256( unsigned char *out, const unsigned char *in, unsigned long long inlen ) {

	blake256_state S;
	blake256_init( &S );
	blake256_update( &S, in, inlen*8 );
	blake256_final( &S, out );
	return 0;
}

/**
 *  Simulates power analysis attack on BLAKE-256 in the case of SPHINCS-256.
 *
 *  Reads the whole BLAKE-256 input from UART, which consists of SEED_BYTES
 *  + ADDR_BYTES.
 *
 *  The simulation computes the leaves address as in the SPHINCS-256 signing
 *  procedure. Therefore, there are 2^SUBTREE_HEIGHT calls with every iteration
 *  of the address least significant bits.
 *
 */

static void hash_attack(uint16_t count, uint16_t del_ms)
{
	unsigned char seed[SEED_BYTES];
	unsigned char buffer[SEED_BYTES + ADDR_BYTES];
	unsigned char * addr = buffer + SEED_BYTES;
	unsigned char j;
	int i;

	for(i = 0; i < SEED_BYTES + ADDR_BYTES; ++i)
	{
		READ_BYTE(buffer[i]);
	}

	for (i = 0; i < SEED_BYTES + ADDR_BYTES; ++i)
	{
		SEND_BYTE(buffer[i]);
	}

	for (i = 0; i < count; ++i)
	{
		for (j = 0; j < (1 << SUBTREE_HEIGHT); ++j)
		{
			/* Clears the SUBTREE_HEIGHT least significant bits */
			addr[7] = (addr[7] & SUBTREE_MASK) | j;

			crypto_hash_blake256(seed, buffer, SEED_BYTES + ADDR_BYTES);

			delay_ms(del_ms);
		}
	}
}

/**
 *  Power analysis attack entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */

int main(void)
{
	/* Function to run */
	void (*run_func)(uint16_t, uint16_t) = hash_attack;

	/* Initializes the SAM system */
	sysclk_init();
	board_init();
	configure_console();
	ioport_init();

	/* Directs pin and initializes to zero */
	ioport_set_pin_dir(PIO_TRIGGER, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIO_TRIGGER, 0);

	/* Tests trigger */
	TRIGGER;
	delay_us(OP_DELAY_US);
	TRIGGER;

	while (1)
	{
		uint16_t count = 0;
		uint16_t del_ms = 0;

		/* Waits for 0xaa and responds with 0xbb */
		WAIT_FOR_CHAR;

		READ_SHORT(count);
		SEND_SHORT(count);
		READ_SHORT(del_ms);
		SEND_SHORT(del_ms);

		(run_func)(count, del_ms);
	}

	return 0;
}
