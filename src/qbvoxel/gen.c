/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/gen.c
 * @brief Base file for the Qubicle parser library
 */
#define QBVoxel_API_Impl
#include "qbvoxel/gen.h"
#include <string.h>
#include <stddef.h>

/**
 * @brief Fetch a voxel from the callback.
 * @param s generator state
 * @param pos write position for buffer
 */
static
void qbvoxel_gen_cb_fetch(struct qbvoxel_state *s, size_t pos);
/**
 * @brief Advance the voxel position.
 * @param s generator state
 * @return nonzero at end of slice, zero otherwise
 */
static
int qbvoxel_gen_nextvoxel(struct qbvoxel_state *s);
/**
 * @brief Advance the matrix position.
 * @param s generator state
 */
static
void qbvoxel_gen_newmatrix(struct qbvoxel_state *s);


/* BEGIN qbvoxel/gen static */
void qbvoxel_gen_cb_fetch(struct qbvoxel_state *s, size_t pos) {
  if (s->cb != NULL) {
    struct qbvoxel_voxel rgba = { 0 };
    s->last_error = (*s->cb->read_voxel)
      (s->cb->p, s->i, s->x, s->y, s->z, &rgba);
    if (s->flags & QBVoxel_FlagBGRA) {
      s->buffer[pos+0u] = rgba.b;
      s->buffer[pos+1u] = rgba.g;
      s->buffer[pos+2u] = rgba.r;
      s->buffer[pos+3u] = rgba.a;
    } else /*RGBA */{
      s->buffer[pos+0u] = rgba.r;
      s->buffer[pos+1u] = rgba.g;
      s->buffer[pos+2u] = rgba.b;
      s->buffer[pos+3u] = rgba.a;
    }
  } else {
    memset(s->buffer+pos,0,4u);
  }
  return;
}

int qbvoxel_gen_nextvoxel(struct qbvoxel_state *s) {
  s->x += 1u;
  if (s->x >= s->width) {
    s->x = 0u;
    s->y += 1u;
  }
  if (s->y >= s->height) {
    s->y = 0u;
    s->z += 1u;
    return 1;
  } else return 0;
}

void qbvoxel_gen_newmatrix(struct qbvoxel_state *s) {
  unsigned long int const matrix_count =
    qbvoxel_api_from_u32(s->buffer+24);
  s->i += 1u;
  s->state = (s->i >= matrix_count) ? 255u : 1u;
  return;
}
/* END   qbvoxel/gen static */

int qbvoxel_gen_init(struct qbvoxel_state *s, struct qbvoxel_i* cb) {
  s->last_error = 0;
  s->state = 0u;
  s->flags = 0u;
  s->pos = 0u;
  s->i = 0lu;
  s->x = 0u;
  s->y = 0u;
  s->z = 0u;
  s->width = 0u;
  s->height = 0u;
  s->depth = 0u;
  memset(s->buffer, 0, sizeof(unsigned char)*32);
  s->cb = cb;
  memset(s->name_buffer, 0, sizeof(unsigned char)*256);
  return 0;
}

void qbvoxel_gen_clear(struct qbvoxel_state *s) {
  s->last_error = 0;
  s->state = 0u;
  s->flags = 0u;
  s->pos = 0u;
  s->i = 0lu;
  s->x = 0u;
  s->y = 0u;
  s->z = 0u;
  s->width = 0u;
  s->height = 0u;
  s->depth = 0u;
  memset(s->buffer, 0, sizeof(unsigned char)*32);
  s->cb = NULL;
  memset(s->name_buffer, 0, sizeof(unsigned char)*256);
  return;
}

unsigned int qbvoxel_gen_do
  (struct qbvoxel_state *s, unsigned int sz, unsigned char* buf)
{
  /* states:
   * 0    - header
   * 1    - matrix name length
   * 2    - matrix name
   * 3    - matrix bounds
   * 4    - uncompressed matrix data
   * 255  - end of stream
   */
  unsigned int i;
  for (i = 0u; i < sz && s->last_error == 0; ++i) {
    if (s->state == 255u) {
      /* end of stream, so */break;
    } else switch (s->state) {
    case 0:
      if (s->pos == 0u) {
        static unsigned char const version_text[4] = { 1u, 1u, 0u, 0u };
        static unsigned char const one_text[4] = { 1u, 0u, 0u, 0u };
        static unsigned char const zero_text[4] = { 0u, 0u, 0u, 0u };
        memcpy(s->buffer+0, version_text, 4);
        memcpy(s->buffer+4,
            (s->flags&QBVoxel_FlagBGRA ? one_text : zero_text), 4);
        memcpy(s->buffer+8,
            (s->flags&QBVoxel_FlagRightHand ? one_text : zero_text), 4);
        memcpy(s->buffer+12,
            (s->flags&QBVoxel_FlagRLE ? one_text : zero_text), 4);
        memcpy(s->buffer+16,
            (s->flags&QBVoxel_FlagSideMasks ? one_text : zero_text), 4);
        if (s->cb != NULL) {
          unsigned long int const matrix_count = (*s->cb->size)(s->cb->p);
          if (matrix_count > 0xFFffFFfflu) {
            s->last_error = QBVoxel_ErrMatrixCount;
          }
          qbvoxel_api_to_u32(s->buffer+20, matrix_count);
        } else {
          memcpy(s->buffer+20, zero_text, 4);
        }
      }
      if (s->pos < 24u) {
        buf[i] = s->buffer[s->pos];
        s->pos += 1u;
      }
      if (s->pos >= 24u) {
        s->i = 0u;
        /* save the matrix size */{
          memmove(s->buffer+24, s->buffer+20, 4);
        }
        /* process the matrix size */{
          unsigned long int const matrix_count =
            qbvoxel_api_from_u32(s->buffer+24);
          s->state = (s->i >= matrix_count) ? 255u : 1u;
        }
      } break;
    case 1:
      /* matrix info acquisition */{
        struct qbvoxel_matrix_info mi = { {0}};
        if (s->cb != NULL) {
          s->last_error = (*s->cb->get_matrix)(s->cb->p, s->i, &mi);
        }
        mi.name[255] = '\0';
        /* matrix bounds */{
          qbvoxel_api_to_i32(s->buffer+12, mi.pos_x);
          qbvoxel_api_to_i32(s->buffer+16, mi.pos_y);
          qbvoxel_api_to_i32(s->buffer+20, mi.pos_z);
          qbvoxel_api_to_u32(s->buffer+0, mi.size_x);
          qbvoxel_api_to_u32(s->buffer+4, mi.size_y);
          qbvoxel_api_to_u32(s->buffer+8, mi.size_z);
          s->width = mi.size_x;
          s->height = mi.size_y;
          s->depth = mi.size_z;
        }
        /* matrix name length */{
          size_t const sz = strlen(mi.name);
          s->z = sz;
        }
        memcpy(s->name_buffer, mi.name, s->z);
        s->pos = 0u;
      }
      /* matrix name length */{
        buf[i] = s->z;
        s->state = (s->z > 0) ? 2 : 3;
      } break;
    case 2:
      /* matrix name */if (s->pos < s->z) {
        buf[i] = s->name_buffer[s->pos];
        s->pos += 1;
      }
      if (s->pos >= s->z) {
        s->pos = 0u;
        s->state = 3;
      } break;
    case 3:
      /* matrix bounds */if (s->pos < 24u) {
        buf[i] = s->buffer[s->pos];
        s->pos += 1u;
      }
      if (s->pos >= 24u) {
        s->state = (s->flags & QBVoxel_FlagRLE) ? 5u : 4u;
        s->pos = 0u;
        s->x = 0u;
        s->y = 0u;
        s->z = 0u;

        /* empty block, */if (s->z == s->depth
        ||  s->x == s->width
        ||  s->y == s->height)/* so, */
        {
          /* go to next matrix, or done if this was the last matrix */
          qbvoxel_gen_newmatrix(s);
        }
      } break;
    case 4: /* uncompressed matrix data */
      if (s->pos == 0u) {
        qbvoxel_gen_cb_fetch(s, 0);
      }
      if (s->pos < 4u) {
        buf[i] = s->buffer[s->pos];
        s->pos += 1u;
      }
      if (s->pos >= 4u) {
        s->pos = 0u;
        (void)qbvoxel_gen_nextvoxel(s);
      }
      if (s->z >= s->depth) {
        /* go to next matrix, or done if this was the last matrix */
        qbvoxel_gen_newmatrix(s);
      } break;
    }
  }
  return i;
}
