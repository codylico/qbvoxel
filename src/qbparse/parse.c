/**
 * @file qbparse/api.c
 * @brief Base file for the Qubicle parser library
 */
#define QBParse_API_Impl
#include "qbparse/parse.h"
#include <string.h>
#include <stddef.h>

int qbparse_parse_init(struct qbparse_state *s, struct qbparse_i* cb) {
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

void qbparse_parse_clear(struct qbparse_state *s) {
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

unsigned int qbparse_parse_do
  (struct qbparse_state *s, unsigned int sz, unsigned char const* buf)
{
  /* states:
   * 0    - header
   * 1    - matrix name length
   * 2    - matrix name
   * 3    - matrix bounds
   * 255  - end of stream
   */
  unsigned int i;
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
        unsigned long int const color_type = qbparse_api_from_u32(s->buffer+4);
        unsigned long int const orient = qbparse_api_from_u32(s->buffer+8);
        unsigned long int const compress = qbparse_api_from_u32(s->buffer+12);
        unsigned long int const mask_format =
          qbparse_api_from_u32(s->buffer+16);
        unsigned long int const matrix_count =
          qbparse_api_from_u32(s->buffer+20);
        s->flags = 0u;
        if (memcmp(version_text, s->buffer, 4) != 0) {
          s->last_error = QBParse_ErrVersion;
          break;
        }
        if (color_type >= 2u) {
          s->last_error = QBParse_ErrColorType;
          break;
        } else if (color_type)
          s->flags |= QBParse_FlagBGRA;
        if (orient >= 2u) {
          s->last_error = QBParse_ErrOrientation;
          break;
        } else if (orient)
          s->flags |= QBParse_FlagRightHand;
        if (compress >= 2u) {
          s->last_error = QBParse_ErrCompress;
          break;
        } else if (compress)
          s->flags |= QBParse_FlagRLE;
        if (mask_format >= 2u) {
          s->last_error = QBParse_ErrMaskFormat;
          break;
        } else if (mask_format)
          s->flags |= QBParse_FlagSideMasks;
        if (s->cb != NULL) {
          s->last_error = (*s->cb->resize)(s->cb->p, matrix_count);
        }
        s->i = 0u;
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
        struct qbparse_matrix_info mi = {
            {0},
            /* pos_x */qbparse_api_from_i32(s->buffer+12),
            /* pos_y */qbparse_api_from_i32(s->buffer+16),
            /* pos_z */qbparse_api_from_i32(s->buffer+20),
            /* size_x */qbparse_api_from_u32(s->buffer+0),
            /* size_y */qbparse_api_from_u32(s->buffer+4),
            /* size_z */qbparse_api_from_u32(s->buffer+8)
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
        s->state = 255u;
      } break;
    }
  }
}
