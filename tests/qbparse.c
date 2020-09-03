#include "qbparse/api.h"
#include "qbparse/parse.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef int (*test_cb)(void*);

static int test_u32(void*);
static int test_i32(void*);
static int test_lasterr(void*);
static int test_alloc(void*);
static int test_allocstate(void*);
static int test_parseinit(void*);
static int test_header(void*);
static int test_flags(void*);


/* sample Qubicle file */
static unsigned char const three_qb[] = {
  0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
  0x07,0x4c,0x61,0x79,0x65,0x72,0x2e,0x31,0x03,0x00,0x00,0x00,
  0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0xfb,0xff,0xff,0xff,
  0x00,0x00,0x00,0x00,0xfd,0xff,0xff,0xff,0x32,0x3c,0x39,0xff,
  0x4b,0x69,0x2f,0xff,0x32,0x3c,0x39,0xff,0x4b,0x69,0x2f,0xff,
  0x6a,0xbe,0x30,0xff,0x4b,0x69,0x2f,0xff,0x6a,0xbe,0x30,0xff,
  0x99,0xe5,0x50,0xff,0x6a,0xbe,0x30,0xff,0x4b,0x69,0x2f,0xff,
  0x6a,0xbe,0x30,0xff,0x4b,0x69,0x2f,0xff,0x6a,0xbe,0x30,0xff,
  0x99,0xe5,0x50,0xff,0x6a,0xbe,0x30,0xff,0x99,0xe5,0x50,0xff,
  0xff,0xff,0xff,0xff,0x99,0xe5,0x50,0xff,0x32,0x3c,0x39,0xff,
  0x4b,0x69,0x2f,0xff,0x32,0x3c,0x39,0xff,0x4b,0x69,0x2f,0xff,
  0x6a,0xbe,0x30,0xff,0x4b,0x69,0x2f,0xff,0x6a,0xbe,0x30,0xff,
  0x99,0xe5,0x50,0xff,0x6a,0xbe,0x30,0xff
};

static struct {
  char const* nm;
  test_cb cb;
} const test_list[] = {
  { "u32", test_u32 },
  { "i32", test_i32 },
  { "lasterr", test_lasterr },
  { "alloc", test_alloc },
  { "allocstate", test_allocstate },
  { "parseinit", test_parseinit },
  { "header", test_header },
  { "flags", test_flags }
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

int test_alloc(void* q) {
  (void)q;
  /* */{
    void* const p = qbparse_api_malloc(64);
    if (p == NULL)
      return EXIT_FAILURE;
    qbparse_api_free(p);
  }
  /* */{
    void* const p = qbparse_api_malloc(0);
    if (p != NULL) {
      qbparse_api_free(p);
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

int test_allocstate(void* q) {
  /* */{
    qbparse_state* const p = qbparse_api_alloc_state();
    if (p == NULL)
      return EXIT_FAILURE;
    p->cb = NULL;
    qbparse_api_free(p);
  }
  return EXIT_SUCCESS;
}

int test_parseinit(void* q) {
  qbparse_state st;
  if (qbparse_parse_init(&st, NULL) != 0)
    return EXIT_FAILURE;
  if (qbparse_api_get_error(&st) != 0) {
    qbparse_parse_clear(&st);
    return EXIT_FAILURE;
  }
  qbparse_parse_clear(&st);
  return EXIT_SUCCESS;
}

int test_header(void* q) {
  unsigned int const len = (unsigned int)sizeof(three_qb);
  qbparse_state st;
  if (len < 24)
    return EXIT_FAILURE;
  qbparse_parse_init(&st, NULL);
  /* parse the header */{
    if (qbparse_parse_do(&st, 24, three_qb) != 24) {
      qbparse_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (qbparse_api_get_error(&st) != 0) {
      qbparse_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (st.flags != 02) {
      qbparse_parse_clear(&st);
      return EXIT_FAILURE;
    }
  }
  qbparse_parse_clear(&st);
  return EXIT_SUCCESS;
}


int test_flags(void* p) {
  qbparse_state s;
  (void)p;
  qbparse_api_set_flags(&s, 15);
  if (s.flags != 15)
    return EXIT_FAILURE;
  s.flags = 4;
  if (qbparse_api_get_flags(&s) != 4)
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
