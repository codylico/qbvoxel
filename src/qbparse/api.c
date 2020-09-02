/**
 * @file qbparse/api.c
 * @brief Base file for the Qubicle parser library
 */
#define QBParse_API_Impl
#include "qbparse/api.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

char const* qbparse_api_version(void) {
  return "0.1";
}

unsigned long int qbparse_api_from_u32(unsigned char const* b) {
  /* assert little-endian, twos complement */
  unsigned long int const out =
      (b[0])
    | (((unsigned long int)b[1])<<8)
    | (((unsigned long int)b[2])<<16)
    | (((unsigned long int)b[3])<<24);
  return out;
}

void qbparse_api_to_u32(unsigned char* b, unsigned long int v) {
  b[0] = (unsigned char)(v&255);
  b[1] = (unsigned char)((v>>8)&255);
  b[2] = (unsigned char)((v>>16)&255);
  b[3] = (unsigned char)((v>>24)&255);
  return;
}

long int qbparse_api_from_i32(unsigned char const* b) {
  /* assert little-endian, twos complement */
  unsigned long int const xout = qbparse_api_from_u32(b);
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

void qbparse_api_to_i32(unsigned char* b, long int v) {
  unsigned long int const xv = (unsigned long int)v;
  qbparse_api_to_u32(b, xv);
  return;
}

int qbparse_api_get_error(struct qbparse_state const* s) {
  return s->last_error;
}

void* qbparse_api_malloc(unsigned int sz) {
  if (sz == 0u || sz >= ((size_t)-1))
    return NULL;
  else return malloc(sz);
}

void qbparse_api_free(void* p) {
  free(p);
  return;
}

struct qbparse_state* qbparse_api_alloc_state(void) {
  struct qbparse_state* q =
    (struct qbparse_state*)qbparse_api_malloc(sizeof(struct qbparse_state));
  if (q != NULL) {
    memset(q, 0, sizeof(struct qbparse_state));
    return q;
  } else return NULL;
}
