/*
 * Copyright (C) 2000, 2001, 2002, 2003 Håkan Hjort <d95hjort@dtek.chalmers.se>
 *
 * This file is part of libdvdread.
 *
 * libdvdread is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * libdvdread is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with libdvdread; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "dvdread/bitreader.h"

int dvdread_getbits_init(getbits_state_t *state, uint8_t *start, size_t size) {
  if ((state == NULL) || (start == NULL)) return 0;
  state->start = start;
  state->bit_position = 0;
  state->byte_position = 0;
  state->byte = start[0];
  state->size = size;
  return 1;
}

static void dvdread_getbits_refill(getbits_state_t *state) {
    ++state->byte_position;
    state->byte = state->start[state->byte_position];
    state->bit_position = 0;
}

/* Non-optimized getbits. */
/* This can easily be optimized for particular platforms. */
uint32_t dvdread_getbits(getbits_state_t *state, uint32_t number_of_bits) {
  uint8_t byte;
  if (number_of_bits > 32) {
    printf("Number of bits > 32 in getbits\n");
    abort();
  }

  if (state->bit_position == 8) {
    if (state->byte_position >= state->size - 1)
        return 0;
    dvdread_getbits_refill(state);
  }
  uint8_t current_bits_available = 8 - state->bit_position;
  uint8_t bits_to_read = number_of_bits > current_bits_available ? current_bits_available : number_of_bits;
  byte = state->byte;
  byte <<= state->bit_position;
  /* Read at most a byte on the currently loaded byte. If we need more, we'll
   * read more later on */
  byte >>= 8 - bits_to_read;
  state->bit_position += number_of_bits;
  /* If we had enough bytes to read, just return those */
  if (number_of_bits <= current_bits_available) {
    return byte;
  }
  /* If we don't have enough bits, fetch the next byte */
  if (state->byte_position >= state->size - 1)
      return 0;
  dvdread_getbits_refill(state);
  uint32_t remaining_bits = number_of_bits - current_bits_available;
  return ((uint32_t)byte << (remaining_bits)) |
          dvdread_getbits(state, remaining_bits);
}

#if 0  /* TODO: optimized versions not yet used */

/* WARNING: This function can only be used on a byte boundary.
            No checks are made that we are in fact on a byte boundary.
 */
uint16_t dvdread_get16bits(getbits_state_t *state) {
  uint16_t result;
  state->byte_position++;
  result = (state->byte << 8) + state->start[state->byte_position++];
  state->byte = state->start[state->byte_position];
  return result;
}

/* WARNING: This function can only be used on a byte boundary.
            No checks are made that we are in fact on a byte boundary.
 */
uint32_t dvdread_get32bits(getbits_state_t *state) {
  uint32_t result;
  state->byte_position++;
  result = (state->byte << 8) + state->start[state->byte_position++];
  result = (result << 8) + state->start[state->byte_position++];
  result = (result << 8) + state->start[state->byte_position++];
  state->byte = state->start[state->byte_position];
  return result;
}

#endif

#ifdef BITREADER_TESTS

int main()
{
    uint8_t buff[2] = {
        0x6E, 0xC2
        // 0b 01101110 11000010
    };
    getbits_state_t state;
    dvdread_getbits_init(&state, buff, sizeof(buff));

    uint32_t bits = dvdread_getbits(&state, 3);
    assert(bits == 3);

    bits = dvdread_getbits(&state, 3);
    assert(bits == 3);

    bits = dvdread_getbits(&state, 4);
    assert(bits == 11);

    bits = dvdread_getbits(&state, 6);
    assert(bits == 2);

    dvdread_getbits_init(&state, buff, sizeof(buff));
    bits = dvdread_getbits(&state, 10);
    assert(bits == 443);

    bits = dvdread_getbits(&state, 6);
    assert(bits == 2);

    dvdread_getbits_init(&state, buff, sizeof(buff));
    bits = dvdread_getbits(&state, 16);
    assert(bits == 28354);

    buff[0] = buff[1] = 0xFF;
    dvdread_getbits_init(&state, buff, sizeof(buff));
    bits = dvdread_getbits(&state, 16);
    assert(bits == 0xFFFF);

    uint8_t large[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    dvdread_getbits_init(&state, large, sizeof(large));
    bits = dvdread_getbits(&state, 8);
    assert(bits == 0xFF);
    bits = dvdread_getbits(&state, 32);
    assert(bits == 0xFFFFFFFF);

    return 0;
}

#endif
