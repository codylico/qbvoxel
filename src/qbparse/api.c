/**
 * @file qbparse/api.c
 * @brief Base file for the Qubicle parser library
 */
#define QBParse_API_Impl
#include "qbparse/api.h"

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