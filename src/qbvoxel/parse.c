/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/parse.c
 * @brief Base file for the Qubicle parser library
 */
#define QBVoxel_API_Impl
#include "qbvoxel/parse.h"
#include <string.h>
#include <stddef.h>

/**
 * @brief Deliver a voxel to the callback.
 * @param s parser state
 * @param pos read position for buffer
 */
static
void qbvoxel_parse_cb_put(struct qbvoxel_state *s, size_t pos);
/**
 * @brief Advance the voxel position.
 * @param s parser state
 * @return nonzero at end of slice, zero otherwise
 */
static
int qbvoxel_parse_nextvoxel(struct qbvoxel_state *s);
/**
 * @brief Advance the matrix position.
 * @param s parser state
 */
static
void qbvoxel_parse_newmatrix(struct qbvoxel_state *s);


/* BEGIN qbvoxel/parse static */
void qbvoxel_parse_cb_put(struct qbvoxel_state *s, size_t pos) {
  if (s->cb != NULL) {
    if (s->flags & QBVoxel_FlagBGRA) {
      struct qbvoxel_voxel const bgra = {
          s->buffer[2u+pos], s->buffer[1u+pos],
          s->buffer[0u+pos], s->buffer[3u+pos]
        };
      s->last_error = (*s->cb->write_voxel)(
        s->cb->p, s->i, s->x, s->y, s->z, &bgra);
    } else /*RGBA */{
      struct qbvoxel_voxel const rgba = {
          s->buffer[0u+pos], s->buffer[1u+pos],
          s->buffer[2u+pos], s->buffer[3u+pos]
        };
      s->last_error = (*s->cb->write_voxel)(
        s->cb->p, s->i, s->x, s->y, s->z, &rgba);
    }
  }
  return;
}

int qbvoxel_parse_nextvoxel(struct qbvoxel_state *s) {
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

void qbvoxel_parse_newmatrix(struct qbvoxel_state *s) {
  unsigned long int const matrix_count =
    qbvoxel_api_from_u32(s->buffer+24);
  s->i += 1u;
  s->state = (s->i >= matrix_count) ? 255u : 1u;
  return;
}
/* END   qbvoxel/parse static */


int qbvoxel_parse_init(struct qbvoxel_state *s, struct qbvoxel_i* cb) {
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

void qbvoxel_parse_clear(struct qbvoxel_state *s) {
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

unsigned int qbvoxel_parse_do
  (struct qbvoxel_state *s, unsigned int sz, unsigned char const* buf)
{
  /* states:
   * 0    - header
   * 1    - matrix name length
   * 2    - matrix name
   * 3    - matrix bounds
   * 4    - uncompressed matrix data
   * 5    - compressed matrix data / base
   * 7    - compressed matrix data / run-length encoding
   * 8    - compressed matrix data / expect next slice
   * 9    - compressed matrix data / degenerate nonzero depth
   * 255  - end of stream
   */
  unsigned int i;
  static unsigned char const next_slice[4] = { 6u, 0u, 0u, 0u };
  static unsigned char const next_rle[4] = { 2u, 0u, 0u, 0u };
  for (i = 0u; i < sz && s->last_error == 0; ++i) {
    if (s->state == 255u) {
      /* end of stream, so */break;
    } else switch (s->state) {
    case 0:
      if (s->pos < 24u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 24u) {
        static unsigned char const version_text[4] = { 1u, 1u, 0u, 0u };
        unsigned long int const color_type = qbvoxel_api_from_u32(s->buffer+4);
        unsigned long int const orient = qbvoxel_api_from_u32(s->buffer+8);
        unsigned long int const compress = qbvoxel_api_from_u32(s->buffer+12);
        unsigned long int const mask_format =
          qbvoxel_api_from_u32(s->buffer+16);
        unsigned long int const matrix_count =
          qbvoxel_api_from_u32(s->buffer+20);
        s->flags = 0u;
        if (memcmp(version_text, s->buffer, 4) != 0) {
          s->last_error = QBVoxel_ErrVersion;
          break;
        }
        if (color_type >= 2u) {
          s->last_error = QBVoxel_ErrColorType;
          break;
        } else if (color_type)
          s->flags |= QBVoxel_FlagBGRA;
        if (orient >= 2u) {
          s->last_error = QBVoxel_ErrOrientation;
          break;
        } else if (orient)
          s->flags |= QBVoxel_FlagRightHand;
        if (compress >= 2u) {
          s->last_error = QBVoxel_ErrCompress;
          break;
        } else if (compress)
          s->flags |= QBVoxel_FlagRLE;
        if (mask_format >= 2u) {
          s->last_error = QBVoxel_ErrMaskFormat;
          break;
        } else if (mask_format)
          s->flags |= QBVoxel_FlagSideMasks;
        if (s->cb != NULL) {
          s->last_error = (*s->cb->resize)(s->cb->p, matrix_count);
        }
        s->i = 0u;
        /* save the matrix size */{
          memmove(s->buffer+24, s->buffer+20, 4);
        }
        s->state = (s->i >= matrix_count) ? 255u : 1u;
      } break;
    case 1:
      /* matrix name length */{
        s->z = buf[i]&255u;
        s->pos = 0u;
        s->name_buffer[0] = '\0';
        s->state = (s->z > 0) ? 2 : 3;
      } break;
    case 2:
      /* matrix name */if (s->pos < s->z) {
        s->name_buffer[s->pos] = buf[i];
        s->pos += 1;
      }
      if (s->pos >= s->z) {
        s->name_buffer[s->z] = '\0';
        s->pos = 0u;
        s->state = 3;
      } break;
    case 3:
      /* matrix bounds */if (s->pos < 24u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 24u) {
        struct qbvoxel_matrix_info mi = {
            {0},
            /* pos_x */qbvoxel_api_from_i32(s->buffer+12),
            /* pos_y */qbvoxel_api_from_i32(s->buffer+16),
            /* pos_z */qbvoxel_api_from_i32(s->buffer+20),
            /* size_x */qbvoxel_api_from_u32(s->buffer+0),
            /* size_y */qbvoxel_api_from_u32(s->buffer+4),
            /* size_z */qbvoxel_api_from_u32(s->buffer+8)
          };
        s->width = mi.size_x;
        s->height = mi.size_y;
        s->depth = mi.size_z;
        s->x = 0;
        s->y = 0;
        s->z = 0;
        memcpy(mi.name, s->name_buffer, 256*sizeof(char));
        if (s->cb != NULL) {
          s->last_error = (*s->cb->set_matrix)(s->cb->p, s->i, &mi);
        }
        s->pos = 0u;
        if (s->flags & QBVoxel_FlagRLE) {
          if (s->depth == 0u) {
            /* [[fallthrough]] */;
          } else if (s->width == 0u || s->height == 0u) {
            s->state = 9u;
            break;
          } else {
            s->state = 5u;
            break;
          }
        } else {
          if (s->depth == 0u || s->height == 0u || s->width == 0u) {
            /* [[fallthrough]] */;
          } else {
            s->state = 4u;
            break;
          }
        }
        /* empty block, so */{
          /* go to next matrix, or done if this was the last matrix */
          qbvoxel_parse_newmatrix(s);
        }
      } break;
    case 4: /* uncompressed matrix data */
      if (s->pos < 4u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 4u) {
        s->pos = 0u;
        qbvoxel_parse_cb_put(s, 0u);
        qbvoxel_parse_nextvoxel(s);
      }
      if (s->z >= s->depth) {
        /* go to next matrix, or done if this was the last matrix */
        qbvoxel_parse_newmatrix(s);
      } break;
    case 5: /* compressed matrix data / base */
      if (s->pos < 4u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 4u) {
        int advance_tf = 0;
        if (memcmp(s->buffer, next_slice, 4) == 0) {
          /* early quit, run through final voxels */
          /* TODO is this an error condition? */
          memset(s->buffer+4, 0, 4u);
          do {
            qbvoxel_parse_cb_put(s, 4u);
          } while (qbvoxel_parse_nextvoxel(s) == 0);
          s->pos = 0u;
          advance_tf = 1;
        } else if (memcmp(s->buffer, next_rle, 4) == 0) {
          s->state = 7u;
        } else {
          qbvoxel_parse_cb_put(s, 0u);
          if (qbvoxel_parse_nextvoxel(s) ) {
            s->state = 8u;
          }
          s->pos = 0u;
        }
        if (advance_tf && s->z >= s->depth) {
          /* go to next matrix, or done if this was the last matrix */
          qbvoxel_parse_newmatrix(s);
        }
      } break;
    case 7: /* compressed matrix data / run-length encoding */
      if (s->pos < 12u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 12u) {
        unsigned long int const count = qbvoxel_api_from_u32(s->buffer+4u);
        unsigned long int j;
        int advance_tf = 0;
        for (j = 0u; j < count && (!advance_tf); ++j) {
          qbvoxel_parse_cb_put(s, 8u);
          advance_tf = qbvoxel_parse_nextvoxel(s);
        }
        if (j < count) {
          /* overran slice boundary; signal an error */
          s->last_error = QBVoxel_ErrOverrun;
        } else if (advance_tf) {
          s->state = 8u;
        } else {
          s->state = 5u;
        }
        s->pos = 0u;
      } break;
    case 8: /* compressed matrix data / expect next slice */
    case 9: /* compressed matrix data / degenerate nonzero depth */
      if (s->pos < 4u) {
        s->buffer[s->pos] = buf[i];
        s->pos += 1u;
      }
      if (s->pos >= 4u) {
        if (memcmp(s->buffer, next_slice, 4) == 0) {
          /* okay */;
          if (s->z >= s->depth) {
            /* go to next matrix, or done if this was the last matrix */
            qbvoxel_parse_newmatrix(s);
          } else if (s->state != 9)
            s->state = 5u;
          else { /* stay put */; }
        } else s->last_error = QBVoxel_ErrOverrun;
        s->pos = 0u;
      } break;
    }
  }
  return i;
}
