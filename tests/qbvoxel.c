/* SPDX-License-Identifier: Unlicense */
#include "qbvoxel/api.h"
#include "qbvoxel/parse.h"
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
static int test_arrayresize(void*);
static int test_flags(void*);
static int test_addmatrix(void*);
static int test_fillmatrix(void*);

struct cb_matrix {
  unsigned int width, height, depth;
  int x, y, z;
  char name[16];
  struct qbvoxel_voxel *data;
};
struct cb_matrix_array {
  size_t count;
  struct cb_matrix *matrices;
};

static int cb_resize(void* p, unsigned long int n);
static unsigned long int cb_size(void const* p);
static int cb_get_matrix
  (void const* p, unsigned long int i, struct qbvoxel_matrix_info *mi);
static int cb_set_matrix
  (void* p, unsigned long int i, struct qbvoxel_matrix_info const* mi);
static int cb_read_voxel
  ( void const* p, unsigned long int i,
    unsigned long int x,unsigned long int y,unsigned long int z,
    struct qbvoxel_voxel* v);
static int cb_write_voxel
  ( void* p, unsigned long int i,
    unsigned long int x,unsigned long int y,unsigned long int z,
    struct qbvoxel_voxel const* v);

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
  { "arrayresize", test_arrayresize },
  { "flags", test_flags },
  { "addmatrix", test_addmatrix},
  { "fillmatrix", test_fillmatrix}
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
    qbvoxel_api_to_u32(buf, v);
    if (memcmp(buf, enc, 4) != 0)
      return EXIT_FAILURE;
    if (qbvoxel_api_from_u32(enc) != v)
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
    qbvoxel_api_to_i32(buf, v);
    if (memcmp(buf, enc, 4) != 0)
      return EXIT_FAILURE;
    if (qbvoxel_api_from_i32(enc) != v)
      return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int test_lasterr(void* p) {
  qbvoxel_state s;
  (void)p;
  s.last_error = 0;
  if (qbvoxel_api_get_error(&s) != 0)
    return EXIT_FAILURE;
  s.last_error = 5;
  if (qbvoxel_api_get_error(&s) == 0)
    return EXIT_FAILURE;
  return EXIT_SUCCESS;
}

int test_alloc(void* q) {
  (void)q;
  /* */{
    void* const p = qbvoxel_api_malloc(64);
    if (p == NULL)
      return EXIT_FAILURE;
    qbvoxel_api_free(p);
  }
  /* */{
    void* const p = qbvoxel_api_malloc(0);
    if (p != NULL) {
      qbvoxel_api_free(p);
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

int test_allocstate(void* q) {
  /* */{
    qbvoxel_state* const p = qbvoxel_api_alloc_state();
    if (p == NULL)
      return EXIT_FAILURE;
    p->cb = NULL;
    qbvoxel_api_free(p);
  }
  return EXIT_SUCCESS;
}

int test_parseinit(void* q) {
  qbvoxel_state st;
  if (qbvoxel_parse_init(&st, NULL) != 0)
    return EXIT_FAILURE;
  if (qbvoxel_api_get_error(&st) != 0) {
    qbvoxel_parse_clear(&st);
    return EXIT_FAILURE;
  }
  qbvoxel_parse_clear(&st);
  return EXIT_SUCCESS;
}

int test_header(void* q) {
  unsigned int const len = (unsigned int)sizeof(three_qb);
  qbvoxel_state st;
  if (len < 24)
    return EXIT_FAILURE;
  qbvoxel_parse_init(&st, NULL);
  /* parse the header */{
    if (qbvoxel_parse_do(&st, 24, three_qb) != 24) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (qbvoxel_api_get_error(&st) != 0) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (st.flags != 02) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
  }
  qbvoxel_parse_clear(&st);
  return EXIT_SUCCESS;
}

int test_arrayresize(void* q) {
  struct qbvoxel_i* const qi = (struct qbvoxel_i *)q;
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)(qi->p);
  unsigned int const len = (unsigned int)sizeof(three_qb);
  qbvoxel_state st;
  if (len < 24)
    return EXIT_FAILURE;
  qbvoxel_parse_init(&st, qi);
  /* parse the header */{
    if (qbvoxel_parse_do(&st, 24, three_qb) != 24) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (qbvoxel_api_get_error(&st) != 0) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
    if (qma->count != 1) {
      qbvoxel_parse_clear(&st);
      return EXIT_FAILURE;
    }
  }
  qbvoxel_parse_clear(&st);
  return EXIT_SUCCESS;
}

int test_flags(void* p) {
  qbvoxel_state s;
  (void)p;
  qbvoxel_api_set_flags(&s, 15);
  if (s.flags != 15)
    return EXIT_FAILURE;
  s.flags = 4;
  if (qbvoxel_api_get_flags(&s) != 4)
    return EXIT_FAILURE;
  return EXIT_SUCCESS;
}

int test_addmatrix(void* q) {
  struct qbvoxel_i* const qi = (struct qbvoxel_i *)q;
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)(qi->p);
  unsigned int const len = (unsigned int)sizeof(three_qb);
  qbvoxel_state st;
  int res = EXIT_FAILURE;
  qbvoxel_parse_init(&st, qi);
  /* parse the header */do {
    if (len < 56)
      break;
    if (qbvoxel_parse_do(&st, 56, three_qb) != 56)
      break;
    if (qbvoxel_api_get_error(&st) != 0)
      break;
    if (qma->count != 1)
      break;
    if (qma->matrices[0].width != 3)
      break;
    if (qma->matrices[0].height != 3)
      break;
    if (qma->matrices[0].depth != 3)
      break;
    if (qma->matrices[0].x != -5)
      break;
    if (qma->matrices[0].y != 0)
      break;
    if (qma->matrices[0].z != -3)
      break;
    if (strncmp(qma->matrices[0].name, "Layer.1", 15) != 0)
      break;
    res = EXIT_SUCCESS;
  } while (0);
  qbvoxel_parse_clear(&st);
  return res;
}

int test_fillmatrix(void* q) {
  struct qbvoxel_i* const qi = (struct qbvoxel_i *)q;
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)(qi->p);
  unsigned int const len = (unsigned int)sizeof(three_qb);
  qbvoxel_state st;
  int res = EXIT_FAILURE;
  qbvoxel_parse_init(&st, qi);
  /* parse the header */do {
    if (len < 164)
      break;
    if (qbvoxel_parse_do(&st, 164, three_qb) != 164)
      break;
    if (qbvoxel_api_get_error(&st) != 0)
      break;
    if (qma->count != 1)
      break;
    if (qma->matrices[0].data == NULL)
      break;
    /* */{
      struct qbvoxel_voxel const vxl1 = { 50, 60, 57, 255 };
      if (memcmp(qma->matrices[0].data+0, &vxl1, sizeof(qbvoxel_voxel)) != 0)
        break;
    }
    /* */{
      struct qbvoxel_voxel const vxl2 = { 106, 190, 48, 255 };
      if (memcmp(qma->matrices[0].data+26, &vxl2, sizeof(qbvoxel_voxel)) != 0)
        break;
      if (memcmp(qma->matrices[0].data+24, &vxl2, sizeof(qbvoxel_voxel)) != 0)
        break;
      if (memcmp(qma->matrices[0].data+6, &vxl2, sizeof(qbvoxel_voxel)) != 0)
        break;
    }
    res = EXIT_SUCCESS;
  } while (0);
  qbvoxel_parse_clear(&st);
  return res;
}





int cb_resize(void* p, unsigned long int n) {
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  /* free the old ones */{
    unsigned int i;
    for (i = 0; i < qma->count; ++i) {
      free(qma->matrices[i].data);
    }
    free(qma->matrices);
    qma->matrices = NULL;
    qma->count = 0;
  }
  /* make the new ones */if (n > 0) {
    struct cb_matrix *mx = (struct cb_matrix*)calloc
        (n, sizeof(struct cb_matrix));
    if (mx == NULL)
      return -1;
    else {
      qma->matrices = mx;
      qma->count = n;
    }
  }
  return 0;
}
unsigned long int cb_size(void const* p) {
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  return (unsigned long int)qma->count;
}
int cb_get_matrix
  (void const* p, unsigned long int i, struct qbvoxel_matrix_info *mi)
{
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  if (i >= qma->count)
    return QBVoxel_ErrOutOfRange;
  else {
    struct cb_matrix const* const m = qma->matrices+i;
    memset(mi->name, 0, sizeof(mi->name));
    memcpy(mi->name, m->name, 16);
    mi->pos_x = m->x;
    mi->pos_y = m->y;
    mi->pos_z = m->z;
    mi->size_x = m->width;
    mi->size_y = m->height;
    mi->size_z = m->depth;
    return 0;
  }
}
int cb_set_matrix
  (void* p, unsigned long int i, struct qbvoxel_matrix_info const* mi)
{
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  if (i >= qma->count)
    return QBVoxel_ErrOutOfRange;
  else if (mi->size_x > 10 || mi->size_y > 10 || mi->size_z > 10) {
    return QBVoxel_ErrMemory;
  } else {
    struct cb_matrix* const m = qma->matrices+i;
    size_t const sz = mi->size_x * mi->size_y * mi->size_z;
    struct qbvoxel_voxel* data = (struct qbvoxel_voxel*)calloc
      (sz, sizeof(struct qbvoxel_voxel));
    if (data == NULL)
      return QBVoxel_ErrMemory;
    free(m->data);
    m->data = data;
    memcpy(m->name, mi->name, 15);
    m->name[15] = '\0';
    m->x = mi->pos_x;
    m->y = mi->pos_y;
    m->z = mi->pos_z;
    m->width = mi->size_x;
    m->height = mi->size_y;
    m->depth = mi->size_z;
    return 0;
  }
}
int cb_read_voxel
  ( void const* p, unsigned long int i,
    unsigned long int x,unsigned long int y,unsigned long int z,
    struct qbvoxel_voxel* v)
{
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  if (i >= qma->count)
    return QBVoxel_ErrOutOfRange;
  else {
    struct cb_matrix* const m = qma->matrices+i;
    if (x >= m->width || y >= m->height || z >= m->depth)
      return QBVoxel_ErrOutOfRange;
    *v = m->data[x + (y + z*m->height)*m->width];
    return 0;
  }
}
int cb_write_voxel
  ( void* p, unsigned long int i,
    unsigned long int x,unsigned long int y,unsigned long int z,
    struct qbvoxel_voxel const* v)
{
  struct cb_matrix_array *const qma = (struct cb_matrix_array *)p;
  if (i >= qma->count)
    return QBVoxel_ErrOutOfRange;
  else {
    struct cb_matrix* const m = qma->matrices+i;
    if (x >= m->width || y >= m->height || z >= m->depth)
      return QBVoxel_ErrOutOfRange;
    m->data[x + (y + z*m->height)*m->width] = *v;
    return 0;
  }
}





int main(int argc, char **argv) {
  size_t const test_count = sizeof(test_list)/sizeof(test_list[0]);
  size_t test_i;
  int total_res = EXIT_SUCCESS;
  struct cb_matrix_array qma = {0};
  struct qbvoxel_i p = {
      &qma, cb_resize, cb_size, cb_get_matrix, cb_set_matrix,
      cb_read_voxel, cb_write_voxel
    };
  fprintf(stderr,"1..%u\n", (unsigned int)test_count);
  for (test_i = 0; test_i < test_count; ++test_i) {
    int const res = test_list[test_i].cb(&p);
    char const* result_text;
    if (res != EXIT_SUCCESS && res != 0) {
      total_res = EXIT_FAILURE;
      result_text = "not ok";
    } else result_text = "ok";
    fprintf(stderr, "%s %u %s\n",
      result_text, ((unsigned int)(test_i+1)), test_list[test_i].nm);
  }
  cb_resize(&qma,0);
  return total_res;
}
