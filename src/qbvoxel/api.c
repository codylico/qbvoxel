/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/api.c
 * @brief Base file for the Qubicle parser library
 */
#define QBVoxel_API_Impl
#include "qbvoxel/api.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

char const* qbvoxel_api_version(void) {
  return "0.4";
}

unsigned long int qbvoxel_api_from_u32(unsigned char const* b) {
  /* assert little-endian, twos complement */
  unsigned long int const out =
      (b[0])
    | (((unsigned long int)b[1])<<8)
    | (((unsigned long int)b[2])<<16)
    | (((unsigned long int)b[3])<<24);
  return out;
}

void qbvoxel_api_to_u32(unsigned char* b, unsigned long int v) {
  b[0] = (unsigned char)(v&255);
  b[1] = (unsigned char)((v>>8)&255);
  b[2] = (unsigned char)((v>>16)&255);
  b[3] = (unsigned char)((v>>24)&255);
  return;
}

long int qbvoxel_api_from_i32(unsigned char const* b) {
  /* assert little-endian, twos complement */
  unsigned long int const xout = qbvoxel_api_from_u32(b);
  if (xout >= 0x80000000ul) {
    unsigned long int const out = (xout^0xFFffFFfflu)+1ul;
    static unsigned long int const trap = (~(unsigned long int)(LONG_MIN))+1ul;
    if (out == trap)
      return LONG_MIN;
    else if (out > trap) {
      errno = ERANGE;
      return LONG_MIN;
    } else return -( (long int)(out) );
  } else return (long int)xout;
}

void qbvoxel_api_to_i32(unsigned char* b, long int v) {
  unsigned long int const xv = (unsigned long int)v;
  qbvoxel_api_to_u32(b, xv);
  return;
}

int qbvoxel_api_get_error(struct qbvoxel_state const* s) {
  return s->last_error;
}

void* qbvoxel_api_malloc(unsigned int sz) {
  if (sz == 0u || sz >= ((size_t)-1))
    return NULL;
  else return malloc(sz);
}

void qbvoxel_api_free(void* p) {
  free(p);
  return;
}

struct qbvoxel_state* qbvoxel_api_alloc_state(void) {
  struct qbvoxel_state* q =
    (struct qbvoxel_state*)qbvoxel_api_malloc(sizeof(struct qbvoxel_state));
  if (q != NULL) {
    memset(q, 0, sizeof(struct qbvoxel_state));
    return q;
  } else return NULL;
}

unsigned int qbvoxel_api_get_flags(struct qbvoxel_state const* s) {
  return s->flags;
}

void qbvoxel_api_set_flags(struct qbvoxel_state* s, unsigned int f) {
  s->flags = (f&15);
  return;
}
