#include "qbparse/api.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef int (*test_cb)(void*);

static int test_u32(void*);
static int test_i32(void*);
static int test_lasterr(void*);

static struct {
  char const* nm;
  test_cb cb;
} const test_list[] = {
  { "u32", test_u32 },
  { "i32", test_i32 },
  { "lasterr", test_lasterr }
};

int test_u32(void* p) {
  static unsigned long int const values[] = {
      7, 0x12345678, 0xFEDCBA98
    };
  static unsigned char const value_enc[] = {
      7, 0, 0, 0, 0x78, 0x56, 0x34, 0x12,
      0x98, 0xBA, 0xDC, 0xFE
    };
  static size_t const count = sizeof(values)/sizeof(values[0]);
  size_t i, j = 0;
  (void)p;
  for (i = 0, j = 0; i < count; ++i, j += 4) {
    unsigned long int const v = values[i];
    unsigned char const* const enc = value_enc+j;
    unsigned char buf[4];
    qbparse_api_to_u32(buf, v);
    if (memcmp(buf, enc, 4) != 0)
      return EXIT_FAILURE;
    if (qbparse_api_from_u32(enc) != v)
      return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int test_i32(void* p) {
  static long int const values[] = {
      7, 0x12345678, -0x1234568l
    };
  static unsigned char const value_enc[] = {
      7, 0, 0, 0, 0x78, 0x56, 0x34, 0x12,
      0x98, 0xBA, 0xDC, 0xFE
    };
  static size_t const count = sizeof(values)/sizeof(values[0]);
  size_t i, j = 0;
  (void)p;
  for (i = 0, j = 0; i < count; ++i, j += 4) {
    unsigned long int const v = values[i];
    unsigned char const* const enc = value_enc+j;
    unsigned char buf[4];
    qbparse_api_to_i32(buf, v);
    if (memcmp(buf, enc, 4) != 0)
      return EXIT_FAILURE;
    if (qbparse_api_from_i32(enc) != v)
      return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int test_lasterr(void* p) {
  qbparse_state s;
  (void)p;
  s.last_error = 0;
  if (qbparse_api_get_error(&s) != 0)
    return EXIT_FAILURE;
  s.last_error = 5;
  if (qbparse_api_get_error(&s) == 0)
    return EXIT_FAILURE;
  return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
  size_t const test_count = sizeof(test_list)/sizeof(test_list[0]);
  size_t test_i;
  int total_res = EXIT_SUCCESS;
  void* p = NULL;
  fprintf(stderr,"1..%u\n", (unsigned int)test_count);
  for (test_i = 0; test_i < test_count; ++test_i) {
    int const res = test_list[test_i].cb(p);
    char const* result_text;
    if (res != EXIT_SUCCESS && res != 0) {
      total_res = EXIT_FAILURE;
      result_text = "not ok";
    } else result_text = "ok";
    fprintf(stderr, "%s %u %s\n",
      result_text, ((unsigned int)(test_i+1)), test_list[test_i].nm);
  }
  return total_res;
}
