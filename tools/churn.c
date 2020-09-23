/* SPDX-License-Identifier: Unlicense */
#include "qbvoxel/api.h"
#include "qbvoxel/parse.h"
#include "qbvoxel/gen.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>


struct cb_matrix {
  unsigned int width, height, depth;
  int x, y, z;
  char name[256];
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
    memcpy(mi->name, m->name, 255);
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
  else if (mi->size_x >= UINT_MAX) {
    return QBVoxel_ErrMemory;
  } else if (mi->size_x > 0u && mi->size_y >= UINT_MAX/mi->size_x) {
    return QBVoxel_ErrMemory;
  } else if (mi->size_x > 0u && mi->size_y > 0u
      &&  mi->size_z >= (UINT_MAX/mi->size_x)/mi->size_y)
  {
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
    memcpy(m->name, mi->name, 255);
    m->name[255] = '\0';
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
  struct cb_matrix_array qma = {0};
  struct qbvoxel_i p = {
      &qma, cb_resize, cb_size, cb_get_matrix, cb_set_matrix,
      cb_read_voxel, cb_write_voxel
    };
  char const* in_filename = NULL;
  char const* out_filename = NULL;
  int res = EXIT_SUCCESS;
  int flags = -1;
  /* arguments */{
    int help_tf = 0;
    int argi;
    for (argi = 1; argi < argc; ++argi) {
      if (argv[argi][0] == '-') {
        if (strcmp(argv[argi], "-f") == 0) {
          if (argi+1 < argc) {
            ++argi;
            flags = (int)strtol(argv[argi], NULL, 0);
          }
        } else if (strcmp(argv[argi], "-?") == 0
            ||  strcmp(argv[argi], "-h") == 0)
        {
          help_tf = 1;
        } else {
          fprintf(stderr, "unsupported option \"%s\"\n", argv[argi]);
          res = EXIT_FAILURE;
        }
      } else if (in_filename == NULL) {
        in_filename = argv[argi];
      } else if (out_filename == NULL) {
        out_filename = argv[argi];
      } else {
        fprintf(stderr, "extra file name given; aborting.\n");
        res = EXIT_FAILURE;
      }
    }
    if (help_tf) {
      fputs("usage: ./qbvoxel_churn [options] (infile) (outfile)\n\n"
          "arguments:\n"
          "  (infile)     input file\n"
          "  (outfile)    output file\n\n"
          "options:\n"
          "  -?           print help message\n"
          "  -f (flags)   set flags\n"
          , stderr);
      return EXIT_FAILURE;
    }
  }
  /* parse */if (res == EXIT_SUCCESS) {
    unsigned int sz = 256;
    FILE* inf = (in_filename != NULL) ? fopen(in_filename, "rb") : NULL;
    qbvoxel_state st;
    qbvoxel_parse_init(&st, &p);
    if (in_filename == NULL) {
      fputs("missing input file name\n", stderr);
    } else if (inf == NULL) {
      perror("can not open file");
      res = EXIT_FAILURE;
    } else while (sz > 0 && res == EXIT_SUCCESS) {
      unsigned char inbuf[256];
      size_t const readlen = fread(inbuf, 1u, 256u, inf);
      sz = (unsigned int)readlen;
      /* */{
        unsigned int const parselen = qbvoxel_parse_do(&st, sz, inbuf);
        if (parselen < readlen) {
          int const err = qbvoxel_api_get_error(&st);
          fprintf(stderr, "parse error %i\n", err);
          res = EXIT_FAILURE;
          break;
        }
      }
    }
    if (inf != NULL)
      fclose(inf);
    if (flags == -1)
      flags = qbvoxel_api_get_flags(&st);
    qbvoxel_parse_clear(&st);
  }
  /* generate */if (res == EXIT_SUCCESS) {
    qbvoxel_state st;
    qbvoxel_gen_init(&st, &p);
    qbvoxel_api_set_flags(&st, flags);
    unsigned int sz = 256;
    FILE* outf = out_filename !=NULL ? fopen(out_filename, "wb") : NULL;
    if (out_filename == NULL) {
      fputs("missing output file name\n", stderr);
    } else if (outf == NULL) {
      perror("can not open file");
      res = EXIT_FAILURE;
    } else /* generate */while (sz > 0) {
      unsigned char outbuf[256];
      sz = qbvoxel_gen_do(&st, 256, outbuf);
      if (sz < 256) {
        int const err = qbvoxel_api_get_error(&st);
        if (err != 0) {
          fprintf(stderr, "parse error %i\n", err);
          res = EXIT_FAILURE;
          break;
        }
      }
      /* */{
        size_t const writelen = fwrite(outbuf, 1, sz, outf);
        if (writelen < sz) {
          perror("can not write to file");
          res = EXIT_FAILURE;
          break;
        }
      }
    }
    qbvoxel_gen_clear(&st);
    if (outf != NULL)
      fclose(outf);
  }
  cb_resize(&qma,0);
  return res;
}
