/* SPDX-License-Identifier: Unlicense */
/*
 * Notes about this tool:
 * - Matrices are column major.
 * - Vectors are multiplied on the right of a matrix.
 */
#include "qbvoxel/api.h"
#include "qbvoxel/parse.h"
#include "qbvoxel/gen.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <float.h>
#include <ctype.h>


struct cb_matrix {
  unsigned int width, height, depth;
  int x, y, z;
  int active_tf;
  struct qbvoxel_voxel *data;
};
struct cb_matrix_array {
  size_t count;
  struct cb_matrix *matrices;
};

enum resolve_voxel_coord {
  ResolveVoxel_X = 0,
  ResolveVoxel_Y = 1,
  ResolveVoxel_Z = 2,
  ResolveVoxel_max = 3
};
enum box_side {
  BoxSide_none = 0,
  BoxSide_XMinus = -1,
  BoxSide_XPlus = +1,
  BoxSide_YMinus = -2,
  BoxSide_YPlus = +2,
  BoxSide_ZMinus = -3,
  BoxSide_ZPlus = +3,
};

enum output_format {
  Output_guess = 0,
  Output_PPM,
  Output_PNG
};
struct png_chunker {
  unsigned short low_adler;
  unsigned short high_adler;
  unsigned short data_pos;
  unsigned char past_header;
  unsigned char data_buf[1024];
  size_t expect_total;
  size_t expect_index;
  unsigned long const* crc_table;
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

/**
 * @brief Multiply two 4x4 matrices.
 * @param[out] dst output matrix
 * @param a left matrix
 * @param b right matrix
 */
static
void matrix_mult(float* dst, float const* a, float const* b);
/**
 * @brief Multiply a 4x4 matrix by a vector.
 * @param[out] dst output vector
 * @param a left matrix
 * @param b right vector
 */
static
void matrix_vec_mult(float* dst, float const* m, float const* v);
/**
 * @brief Multiply a 4x4 matrix by a vector, with perspective divide.
 * @param[out] dst output vector
 * @param a left matrix
 * @param b right vector
 */
static
void matrix_vec_multw(float* dst, float const* m, float const* v);
/**
 * @brief Invert a matrix.
 * @param[out] inverted matrix
 * @param src matrix
 * @return nonzero on success, zero if the matrix might be "singular"
 */
static
int matrix_invert(float* dst, float const* src);
/**
 * @brief Trace a ray through a Qubicle matrix.
 * @param[out] color output color
 * @param[out] normal output normal
 * @param voxels the Qubicle matrix
 * @param ray_start start of ray
 * @param ray_end "end" of ray
 * @return a coordinate along the ray, with `0` at the start and `1` at
 *   the "end"
 */
static
float trace_ray
  ( float* color, float* normal, struct cb_matrix const* voxels,
    float const* ray_start, float const* ray_end);
/**
 * @brief Determine when a ray crosses two axis-aligned planes.
 * @param[out] left_right array of `2` floats, to hold the
 *     `t` for crossing the left and right planes, respectively.
 * @param ray_start coordinate of starting point of ray `(t=0)`
 * @param ray_end coordinate of end point of ray `(t=1)`
 * @param box_left coordinate of the left plane
 * @param box_right coordinate of the right plane
 * @return whether the ray crosses or not
 * @note This function handles the parallel cases.
 */
static
int ray_range_unordered
  ( float* left_right, float ray_start, float ray_end,
    float box_left, float box_right);
/**
 * @brief Compute a position along a ray.
 * @param[out] dst array of 3 floats, to hold the position
 * @param t ray-based coordinate
 * @param ray_start coordinate of starting point of ray `(t=0)`
 * @param ray_end coordinate of end point of ray `(t=1)`
 */
static
void ray_interpolate
  (float* dst, float t, float const* ray_start, float const* ray_end);
/**
 * @brief Subtract one vector from another.
 * @param[out] dst array of 3 floats, to hold the position
 * @param a another vector
 * @param b the vector to subtract
 */
static
void vec3_subtract(float *dst, float const* a, float const* b);
/**
 * @brief Resolve which voxel of a matrix gets hit.
 * @param coord axis hint for voxel checking
 * @param[out] color output color
 * @param[out] normal output normal
 * @param voxels the matrix to trace
 * @param box_start start of a ray `(t=0)`
 * @param box_end end of a ray `(t=1)`
 * @return a coordinate along a ray for intersection, or `+HUGE_VAL`
 *   on miss
 */
static
float resolve_voxel(int coord, float *color, float *normal,
  struct cb_matrix const* voxels, float const* box_start,
  float const* box_end);
/**
 * @brief Convert a voxel coordinate to a flat array index.
 * @param voxels the matrix to use
 * @param x x-axis coordinate
 * @param y y-axis coordinate
 * @param z z-axis coordinate
 * @return a flat array index
 */
static
size_t resolve_voxel_pos
  ( struct cb_matrix const* voxels, unsigned int x, unsigned int y,
    unsigned int z);
/**
 * @brief Compute a ray-box intersection.
 * @param[out] side which side of the box was hit, or `BoxSide_none` on miss
 * @param[out] back_intercept the ray coordinate at exit
 * @param corner_1 most negative corner of the box
 * @param corner_2 most positive corner of the box
 * @param ray_start coordinate of starting point of ray `(t=0)`
 * @param ray_end coordinate of end point of ray `(t=1)`
 * @return the ray coordinate at entrance of box
 */
static
float ray_intersect_box
  ( int* side, float* back_intercept,
    float const* corner_1, float const* corner_2,
    float const* ray_start, float const* ray_end);
/**
 * @brief The pixel shader.
 * @param[in,out] color input color for diffuse, output color for shaded
 * @param raster_pos position of pixel in world space
 * @param light_pos position of point light in world space
 * @param ambient ambient color
 * @param normal surface normal
 * @param qma matrix list for shadow computation
 */
static
void pixel_shade
  ( float* color, float const* raster_pos, float const* light_pos,
    float const* ambient, float const* normal, struct cb_matrix_array *qma);
/**
 * @brief Output a framebuffer to a Portable Pixmap Format file.
 * @param[out] outf output file stream
 * @param use_width width of the framebuffer in pixels
 * @param use_height height of the framebuffer in pixels
 * @param frame pixel data for the framebuffer
 */
static
void write_to_ppm(FILE* outf, unsigned use_width, unsigned use_height,
  struct qbvoxel_voxel const* frame);
/**
 * @brief Output a framebuffer to a Portable Network Graphics file.
 * @param[out] outf output file stream
 * @param use_width width of the framebuffer in pixels
 * @param use_height height of the framebuffer in pixels
 * @param frame pixel data for the framebuffer
 */
static
void write_to_png(FILE *outf, unsigned use_width, unsigned use_height,
  struct qbvoxel_voxel const *frame);
/**
 * @brief Encode an unsigned 32-bit integer to a byte array for PNG.
 * @param b byte array
 * @param v the unsigned integer
 */
static
void to_u32_png(unsigned char* b, unsigned long int v);
/**
 * @brief Update a PNG checksum.
 * @param table lookup table
 * @param crc checksum to update
 * @param data bytes to feed into the checksum
 * @param n number of bytes to feed
 * @return updated checksum
 * @note Based on pseudo-code from Appendix 15 of RFC 2083.
 */
static
unsigned long run_crc_png(unsigned long const* table, unsigned long crc,
  unsigned char const* data, size_t n);
/**
 * @brief Generate a lookup table for PNG checksums.
 * @param[out] crc 256-entry table to generate
 * @note Based on pseudo-code from Appendix 15 of RFC 2083.
 */
static
void generate_crc_png(unsigned long* crc);
/**
 * @brief Chunk PNG data together for output.
 * @param png tracking structure
 * @param[out] f output stream
 * @param v next byte to write
 */
static
void idat_chunk_png(struct png_chunker* png, FILE* f, unsigned char v);
/**
 * @brief Post the Adler32 checksum.
 * @param png tracking structure
 * @param[out] f output stream
 */
static
void idat_adler_png(struct png_chunker* png, FILE* f);

static
int pull_float_args(float* dst, int n, int*argi, int argc, char **argv);
static
unsigned int clamp255(float x);
static
unsigned int clamp254(float x);


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
    mi->name[0] = 0;
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
    m->active_tf = 1;
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


void matrix_mult(float* dst, float const* a, float const* b) {
  size_t i;
  float output[16];
  for (i = 0u; i < 16u; ++i) {
    double sum = 0.f;
    size_t k;
    for (k = 0u; k < 4u; ++k) {
      sum += a[(i&3)+k*4u] * b[k+(i&12u)];
    }
    output[i] = sum;
  }
  memcpy(dst, output, 16u*sizeof(float));
  return;
}

void matrix_vec_mult(float* dst, float const* m, float const* v) {
  size_t i;
  float output[4];
  for (i = 0u; i < 4u; ++i) {
    float sum = 0.f;
    size_t k;
    for (k = 0u; k < 4u; ++k) {
      sum += m[(i&3)+k*4u] * v[k];
    }
    output[i] = sum;
  }
  memcpy(dst, output, 4u*sizeof(float));
  return;
}
void matrix_vec_multw(float* dst, float const* m, float const* v) {
  float output[4];
  matrix_vec_mult(output, m, v);
  if (output[3] > FLT_EPSILON || output[3] < -FLT_EPSILON) {
    float const w = output[3];
    output[0] /= w;
    output[1] /= w;
    output[2] /= w;
  }
  memcpy(dst, output, 4u*sizeof(float));
  return;
}

int matrix_invert(float* dst, float const* src) {
  static float const iden[16] = {1.0,0.f,0.f,0.f,  0.f,1.f,0.f,0.f,
    0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f};
  float left[16];
  float right[16];
  memcpy(left, src, 16*sizeof(float));
  memcpy(right, iden, 16*sizeof(float));
  /* elimination */{
    size_t steps;
    for (steps = 0u; steps < 3u; ++steps) {
      size_t const end = (steps+1u)*4u;
      size_t const diag = steps + steps*4u;
      /* row swaps */{
        size_t i;
        size_t maxrow = ~(size_t)0u;
        float maxpivot = 0.f;
        for (i = steps*4u; i < end; ++i) {
          float const absval = (float)fabs(left[i]);
          if (absval > maxpivot) {
            maxrow = i;
            maxpivot = absval;
          }
        }
        if (maxpivot < FLT_EPSILON) {
          /* divide by zero warning */
          memcpy(dst, iden, 16*sizeof(float));
          return 0;
        } else if (diag != maxrow) {
          size_t const pos = maxrow%4u;
          size_t j;
          for (j = 0u; j < 4u; ++j) {
            /* */{
              float const tmp = left[j*4u+pos];
              left[j*4u+pos] = left[j*4u+steps];
              left[j*4u+steps] = tmp;
            }
            /* */{
              float const tmp = right[j*4u+pos];
              right[j*4u+pos] = right[j*4u+steps];
              right[j*4u+steps] = tmp;
            }
          }
        } else { /* all set */; }
      }
      /* eliminate */{
        size_t i;
        for (i = diag+1u; i < end; ++i) {
          float const x = left[i]/left[diag];
          size_t j;
          for (j = (i%4u); j < 16u; j += 4u) {
            size_t const diagj = j-(i-diag);
            left[j] -= x*left[diagj];
            right[j] -= x*right[diagj];
          }
        }
      }
    }
  }
  if ((float)fabs(left[15u]) < FLT_EPSILON) {
    /* divide by zero warning */
    memcpy(dst, iden, 16*sizeof(float));
    return 0;
  }
  /* back substitute */{
    size_t j;
    /* per column */for (j = 0u; j < 16u; j += 4u) {
      right[j+3u] /= left[15u];
      right[j+2u] -= right[j+3u]*left[14u];
      right[j+2u] /= left[10u];
      right[j+1u] -= right[j+3u]*left[13u];
      right[j+1u] -= right[j+2u]*left[9u];
      right[j+1u] /= left[5u];
      right[j   ] -= right[j+3u]*left[12u];
      right[j   ] -= right[j+2u]*left[8u];
      right[j   ] -= right[j+1u]*left[4u];
      right[j   ] /= left[0u];
    }
  }
  memcpy(dst, right, 16*sizeof(float));
  return 1;
}



void vec3_subtract(float *dst, float const* a, float const* b) {
  float output[3] = {a[0]-b[0], a[1]-b[1], a[2]-b[2] };
  memcpy(dst, output, sizeof(float)*3u);
  return;
}

int ray_range_unordered
  ( float* left_right, float ray_start, float ray_end,
    float box_left, float box_right)
{
  float const diff = ray_end-ray_start;
  if ((float)fabs(diff) < FLT_EPSILON) {
    int yes;
    if (box_left < box_right) {
      yes = (box_left < ray_start && ray_start < box_right);
    } else yes = (box_left > ray_start && ray_start > box_right);
    left_right[0] = -HUGE_VAL;
    left_right[1] = +HUGE_VAL;
    return yes;
  } else {
    float ray_left = (box_left-ray_start)/diff;
    float ray_right = (box_right-ray_start)/diff;
    left_right[0] = ray_left;
    left_right[1] = ray_right;
    return 1;
  }
}


float ray_intersect_box
  ( int* side, float* back_intercept,
    float const* corner_1, float const* corner_2,
    float const* ray_start, float const* ray_end)
{
  int output_side = BoxSide_none;
  float box_min = -HUGE_VAL, box_max = +HUGE_VAL;
  /* x-axis */{
    float x_leftright[2];
    int const x_intersect_tf = ray_range_unordered
      (x_leftright, ray_start[0], ray_end[0], corner_1[0], corner_2[0]);
    if (!x_intersect_tf) {
      *side = BoxSide_none;
      return +HUGE_VAL;
    } else if (x_leftright[0] < x_leftright[1]) {
      box_min = x_leftright[0];
      box_max = x_leftright[1];
      output_side = BoxSide_XMinus;
    } else {
      box_min = x_leftright[1];
      box_max = x_leftright[0];
      output_side = BoxSide_XPlus;
    }
  }
  /* y-axis */{
    float y_leftright[2];
    int const y_intersect_tf = ray_range_unordered
      (y_leftright, ray_start[1], ray_end[1], corner_1[1], corner_2[1]);
    if (!y_intersect_tf) {
      *side = BoxSide_none;
      return +HUGE_VAL;
    } else if (y_leftright[0] < y_leftright[1]) {
      if (y_leftright[0] > box_min) {
        output_side = BoxSide_YMinus;
        box_min = y_leftright[0];
      }
      if (y_leftright[1] < box_max) {
        box_max = y_leftright[1];
      }
    } else {
      if (y_leftright[1] > box_min) {
        output_side = BoxSide_YPlus;
        box_min = y_leftright[1];
      }
      if (y_leftright[0] < box_max) {
        box_max = y_leftright[0];
      }
    }
  }
  /* z-axis */ {
    float z_leftright[2];
    int const z_intersect_tf = ray_range_unordered
      (z_leftright, ray_start[2], ray_end[2], corner_1[2], corner_2[2]);
    if (!z_intersect_tf) {
      *side = BoxSide_none;
      return +HUGE_VAL;
    } else if (z_leftright[0] < z_leftright[1]) {
      if (z_leftright[0] > box_min) {
        output_side = BoxSide_ZMinus;
        box_min = z_leftright[0];
      }
      if (z_leftright[1] < box_max) {
        box_max = z_leftright[1];
      }
    } else {
      if (z_leftright[1] > box_min) {
        output_side = BoxSide_ZPlus;
        box_min = z_leftright[1];
      }
      if (z_leftright[0] < box_max) {
        box_max = z_leftright[0];
      }
    }
  }
  if (box_min >= box_max-FLT_EPSILON) {
    *side = BoxSide_none;
    /* no meaningful intersection, so */return +HUGE_VAL;
  } else {
    *side = output_side;
    if (back_intercept != NULL)
      *back_intercept = box_max;
    return box_min;
  }
}


float trace_ray
  ( float* color, float* normal, struct cb_matrix const* voxels,
    float const* ray_start, float const* ray_end)
{
  float z_intercept;
  /* perform a box intersection */{
    float const pos[3] =
      {(float)voxels->x, (float)voxels->y, (float)voxels->z};
    float const opposite[3] =
      { pos[0] + (float)voxels->width,
        pos[1] + (float)voxels->height,
        pos[2] + (float)voxels->depth};
    int side;
    float box_max;
    z_intercept = ray_intersect_box
      (&side, &box_max, pos, opposite, ray_start, ray_end);
    if (side == 0) {
      /* no meaningful intersection, so */return +HUGE_VAL;
    } else {
      float const first_z_intercept = z_intercept < 0.f ? 0.f : z_intercept;
      float box_start[3];
      float box_end[3];
      //box_side_pseudocolor(color, side);
      ray_interpolate(box_start, first_z_intercept, ray_start, ray_end);
      ray_interpolate(box_end, box_max, ray_start, ray_end);
      /* into array space */{
        vec3_subtract(box_start, box_start, pos);
        vec3_subtract(box_end, box_end, pos);
      }
      /* iterate */{
        float const x_diff = (float)fabs(box_end[0] - box_start[0]);
        float const y_diff = (float)fabs(box_end[1] - box_start[1]);
        float const z_diff = (float)fabs(box_end[2] - box_start[2]);
        int coord;
        if (x_diff > y_diff && x_diff > z_diff) {
          coord = ResolveVoxel_X;
        } else if (y_diff > z_diff) {
          coord = ResolveVoxel_Y;
        } else {
          coord = ResolveVoxel_Z;
        }
        z_intercept = first_z_intercept +
            resolve_voxel(coord, color, normal, voxels, box_start, box_end)
          * (box_max-first_z_intercept);
      }
    }
  }
  return z_intercept;
}

float resolve_voxel(int coord, float *color, float *normal,
  struct cb_matrix const* voxels, float const* box_start,
  float const* box_end)
{
  unsigned int f_min = (unsigned int)box_start[coord];
  unsigned int f_max = (unsigned int)box_end[coord];
  unsigned int const dims[3] = {voxels->width, voxels->height, voxels->depth};
  unsigned int const dir[3] = {
      (box_start[0] < box_end[0]) ? +1 : ((unsigned int)-1),
      (box_start[1] < box_end[1]) ? +1 : ((unsigned int)-1),
      (box_start[2] < box_end[2]) ? +1 : ((unsigned int)-1)
    };
  unsigned int const slice_size = dims[0]*dims[1];
  float const diff[3] = {
      box_end[0] - box_start[0], box_end[1] - box_start[1],
      box_end[2] - box_start[2]
    };
  if (f_min >= dims[coord]) {
    f_min = dims[coord]-1u;
  }
  if (f_max >= dims[coord]) {
    f_max = dims[coord]-1u;
  }
  /* the work */{
    unsigned int f_i;
    unsigned int const pre_steps = 
      (f_min < f_max) ? (f_max-f_min+1u) : (f_min-f_max+1u);
    unsigned int const steps =
      (pre_steps < UINT_MAX) ? pre_steps+1u : UINT_MAX;
    unsigned int old_loc[3] = {
        (unsigned int)box_start[0], (unsigned int)box_start[1],
        (unsigned int)box_start[2]
      };
    for (f_i = 0u; f_i < steps; ++f_i) {
      unsigned int next_loc[3];
      unsigned int point = f_i*dir[coord] + f_min;
      float const rediff = (point - box_start[coord])/diff[coord];
      next_loc[0] = (unsigned int)(rediff*diff[0] + box_start[0]);
      next_loc[1] = (unsigned int)(rediff*diff[1] + box_start[1]);
      next_loc[2] = (unsigned int)(rediff*diff[2] + box_start[2]);
      /* */{
        float box_min = (float)(+HUGE_VAL);
        int side = BoxSide_none;
        size_t use_voxel = ((size_t)-1);
        size_t voxel_pos[8] = {
            resolve_voxel_pos(voxels, old_loc[0], old_loc[1], old_loc[2]),
            resolve_voxel_pos
              (voxels, old_loc[0]+dir[0], old_loc[1], old_loc[2]),
            resolve_voxel_pos
              (voxels, old_loc[0], old_loc[1]+dir[1], old_loc[2]),
            resolve_voxel_pos
              (voxels, old_loc[0], old_loc[1], old_loc[2]+dir[2]),
            resolve_voxel_pos
              (voxels, old_loc[0]+dir[0], old_loc[1]+dir[1], old_loc[2]),
            resolve_voxel_pos
              (voxels, old_loc[0], old_loc[1]+dir[1], old_loc[2]+dir[2]),
            resolve_voxel_pos
              (voxels, old_loc[0]+dir[0], old_loc[1], old_loc[2]+dir[2]),
            resolve_voxel_pos
              (voxels, old_loc[0]+dir[0], old_loc[1]+dir[1], old_loc[2]+dir[2])
          };
        size_t i;
        /* inspect the neighboring voxels */
        for (i = 0u; i < 8u; ++i) {
          size_t const this_voxel = voxel_pos[i];
          if (this_voxel == ((size_t)-1))
            continue;
          else if (voxels->data[this_voxel].a != 0) {
            unsigned int const local_x = this_voxel%dims[0];
            unsigned int const local_y = (this_voxel/dims[0])%dims[1];
            unsigned int const local_z = this_voxel/slice_size;
            float const global_coords[3] = {
                local_x-0.001f, local_y-0.001f, local_z-0.001f,
              };
            float const global_coords_plus1[3] = {
                (local_x+1u)+0.001f, (local_y+1u)+0.001f,
                (local_z+1u)+0.001f
              };
            /* intersect */
            int next_side;
            float const next_intercept = ray_intersect_box
              ( &next_side, NULL, global_coords, global_coords_plus1,
                box_start, box_end);
            if (next_side && next_intercept < box_min) {
              box_min = next_intercept;
              side = next_side;
              use_voxel = this_voxel;
            }
          } else continue;
        }
        /* assign the color */if (side != BoxSide_none) {
          color[0] = voxels->data[use_voxel].r/255.f;
          color[1] = voxels->data[use_voxel].g/255.f;
          color[2] = voxels->data[use_voxel].b/255.f;
          color[3] = 1.f;
          normal[0] = 0.f;normal[1] = 0.f;normal[2] = 0.f;
          switch (side) {
          case BoxSide_XMinus: normal[0] = -1.f; break;
          case BoxSide_XPlus: normal[0] = +1.f; break;
          case BoxSide_YMinus: normal[1] = -1.f; break;
          case BoxSide_YPlus: normal[1] = +1.f; break;
          case BoxSide_ZMinus: normal[2] = -1.f; break;
          case BoxSide_ZPlus: normal[2] = +1.f; break;
          }
          //box_side_pseudocolor(color, side);
          return box_min;
        }
      }
      memcpy(old_loc, next_loc, sizeof(old_loc));
    }
  }
  return 0.f;
}

void ray_interpolate
  (float* dst, float t, float const* ray_start, float const* ray_end)
{
  unsigned int i;
  float output[3];
  for (i = 0; i < 3u; ++i) {
    output[i] = ray_start[i] + t*(ray_end[i] - ray_start[i]);
  }
  memcpy(dst, output, 3u*sizeof(float));
  return;
}

size_t resolve_voxel_pos
  ( struct cb_matrix const* voxels, unsigned int x, unsigned int y,
    unsigned int z)
{
  if (x >= voxels->width || y >= voxels->height || z >= voxels->depth)
    return ((size_t)-1);
  else return x + (y + z*(size_t)voxels->height)*voxels->width;
}

int pull_float_args(float* dst, int n, int*argi, int argc, char **argv) {
  int i;
  char const* option_name = argv[*argi];
  for (i = 0; i < n; ++i) {
    if ((*argi)+1 < argc) {
      ++(*argi);
      dst[i] = (float)strtod(argv[*argi], NULL);
    } else {
      fprintf(stderr, "not enough arguments given for '%s'\n", option_name);
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

unsigned int clamp255(float x) {
  return (x < 0.f) ? 0u 
    : ((x > 1.f) ? 255u : (unsigned int)(x*255.f));
};
unsigned int clamp254(float x) {
  return (x < 0.f) ? 1u 
    : ((x > 1.f) ? 255u : (unsigned int)(x*254.f) + 1u);
};

void pixel_shade
  ( float* color, float const* raster_pos, float const* light_pos,
    float const* ambient, float const* normal, struct cb_matrix_array *qma)
{
  float light_dir[3];
  float light_power;
  float const orig_color[3] = { color[0], color[1], color[2] };
  vec3_subtract(light_dir, light_pos, raster_pos);
  /* normalize */{
    double const len = sqrt(light_dir[0]*(double)light_dir[0]
      + light_dir[1]*(double)light_dir[1]
      + light_dir[2]*(double)light_dir[2]);
    if (len > FLT_EPSILON) {
      light_dir[0] /= len;
      light_dir[1] /= len;
      light_dir[2] /= len;
    } else {/* light it up */
      memcpy(light_dir, normal, 3*sizeof(float));
    }
  }
  /* compute the "diffuse" coefficient */{
    float const pre_light_power = light_dir[0]*normal[0]
        + light_dir[1] * normal[1]
        + light_dir[2] * normal[2];
    if (pre_light_power < 0.f) {
      light_power = 0.25f;
    } else if (pre_light_power > 1.f) {
      light_power = 1.f;
    } else light_power = pre_light_power*0.75f+0.25f;
    /* shadow cast */if (pre_light_power > 0.f) {
      size_t k;
      float closest = +HUGE_VAL;
      float const one_minus = 1.f-FLT_EPSILON;
      float const light_start[3] = {
          raster_pos[0] + 0.015625*light_dir[0],
          raster_pos[1] + 0.015625*light_dir[1],
          raster_pos[2] + 0.015625*light_dir[2],
        };
      for (k = 0u; k < qma->count; ++k) {
        float tmp_shade_color[4];
        float tmp_shade_normal[3];
        float t = trace_ray
          ( tmp_shade_color, tmp_shade_normal, qma->matrices+k,
            light_start, light_pos);
        if (t < FLT_EPSILON || t > one_minus)
          continue;
        else if (t < closest) {
          closest = t;
        }
      }
      if (closest < 1.f) { /* light is blocked, compute a shadow */
        light_power = (light_power > 0.375f) ? 0.375f : light_power;
      }
    }
  }
  /* compute the final color */{
    float const ambient_coeff[3] = {
        1.f - ambient[0], 1.f - ambient[1], 1.f - ambient[2]
      };
    float const diffuse[3] = { orig_color[0], orig_color[1], orig_color[2] };
    unsigned int i;
    for (i = 0u; i < 3u; ++i)
      color[i] = diffuse[i]*light_power*ambient_coeff[i] + ambient[i];
  }
  return;
}


void to_u32_png(unsigned char* dst, unsigned long int val) {
  for (int i = 3; i >= 0; --i) {
    dst[i] = (unsigned char)(val&255u);
    val >>= 8;
  }
  return;
}
unsigned long run_crc_png(unsigned long const* table, unsigned long crc,
  unsigned char const* data, size_t n)
{
  size_t i;
  for (i = 0; i < n; ++i) {
    crc = (crc>>8) ^ table[(crc ^ data[i])&255u];
  }
  return crc;
}
void idat_chunk_png(struct png_chunker* png, FILE* f, unsigned char v) {
  unsigned short const adler_wrap = 65521u;
  png->low_adler = (png->low_adler+v)%adler_wrap;
  png->high_adler = (png->high_adler+png->low_adler)%adler_wrap;
  png->data_buf[png->data_pos++] = v;
  png->expect_index += 1;
  if (png->data_pos >= sizeof(png->data_buf)
  ||  png->expect_index >= png->expect_total)
  {
    unsigned long crc = 0xca50f9e1;
    int const at_end = (png->expect_index >= png->expect_total);
    unsigned char idat[8] = {0,0,0,0, 0x49,0x44,0x41,0x54};
    unsigned char zlib_block[7] = {0};
    unsigned char num_buf[4] = {0};
    static unsigned char const zlib_start[2] = {8,29};
    /* avoid counting Adler32 as part of "compressed" data */
    unsigned short real_data_count = png->data_pos;
    unsigned long idat_size = png->data_pos;
    size_t const remaining = png->expect_total - png->expect_index;
    if (remaining < 4) {
      real_data_count = (real_data_count > 4)
        ? real_data_count-(4 - remaining) : 0;
    }
    /* generate zlib block header */
    if (!png->past_header) {
      crc = run_crc_png(png->crc_table, crc, zlib_start, sizeof(zlib_start));
      idat_size += sizeof(zlib_start);
    }
    if (real_data_count > 0) {
      zlib_block[0] = (unsigned char)at_end;
      qbvoxel_api_to_u32(zlib_block+1, real_data_count);
      qbvoxel_api_to_u32(zlib_block+3, ~real_data_count);
      crc = run_crc_png(png->crc_table, crc, zlib_block, 5);
      idat_size += 5;
    }
    /* write to output */
    to_u32_png(idat, idat_size);
    fwrite(idat, sizeof(char), sizeof(idat), f);
    if (!png->past_header) {
      fwrite(zlib_start, sizeof(char), 2, f);
      png->past_header = 1;
    }
    if (real_data_count > 0)
      fwrite(zlib_block, sizeof(char), 5, f);
    crc = run_crc_png(png->crc_table, crc, png->data_buf, png->data_pos);
    fwrite(png->data_buf, sizeof(char), png->data_pos, f);
    to_u32_png(num_buf, crc^0xFFffFFff);
    fwrite(num_buf, sizeof(char), sizeof(num_buf), f);
    png->data_pos = 0;
  }
}
void idat_adler_png(struct png_chunker* png, FILE* f) {
  unsigned char num_buf[4] = {0};
  unsigned long const adler32 =
    ((unsigned long)png->high_adler<<16) | png->low_adler;
  int i;
  to_u32_png(num_buf, adler32);
  for (i = 0; i < 4; ++i)
    idat_chunk_png(png, f, num_buf[i]);
  return;
}

void write_to_ppm(FILE* outf, unsigned use_width, unsigned use_height,
  struct qbvoxel_voxel const* frame)
{
  unsigned int j;
  fprintf(outf, "P3 %u %u 255\n", use_width, use_height);
  for (j = 0u; j < use_height; ++j) {
    unsigned int i;
    for (i = 0u; i < use_width; ++i) {
      struct qbvoxel_voxel const vx = frame[i + j*use_width];
      fprintf(outf, "%u %u %u\n", vx.r, vx.g, vx.b);
    }
  }
  return;
}
void write_to_png(FILE* outf, unsigned use_width, unsigned use_height,
  struct qbvoxel_voxel const* frame)
{
  static unsigned char signature[8] = {137,80,78,71,13,10,26,10};
  unsigned char header[17] = {0x49,0x48,0x44,0x52,
    0,0,0,0,  0,0,0,0,  8, 2, 0, 0, 0};
  unsigned char num_buf[4] = {0};
  static unsigned char const end[12] = {0,0,0,0, 0x49,0x45,0x4E,0x44, 0xAE,0x42,0x60,0x82};
  struct png_chunker png = {1};
  unsigned j;
  /* generate CRC table */
  unsigned long crc_table[256] = {0};
  generate_crc_png(crc_table);
  png.crc_table = crc_table;
  /* write the signature */
  fwrite(signature, sizeof(char), sizeof(signature), outf);
  /* write the header */
  to_u32_png(header+4, use_width);
  to_u32_png(header+8, use_height);
  {
    unsigned long header_crc = run_crc_png(crc_table, 0xFFffFFff, header, sizeof(header));
    to_u32_png(num_buf, sizeof(header)-4);
    fwrite(num_buf, sizeof(char), sizeof(num_buf), outf);
    fwrite(header, sizeof(char), sizeof(header), outf);
    to_u32_png(num_buf, header_crc^0xFFffFFff);
    fwrite(num_buf, sizeof(char), sizeof(num_buf), outf);
  }
  /* write the data */
  png.expect_total = use_height*(use_width*(size_t)3u+1u)+4u;
  for (j = 0; j < use_height; ++j) {
    unsigned int i;
    /* output null filter */
    idat_chunk_png(&png, outf, 0);
    for (i = 0; i < use_width; ++i) {
      struct qbvoxel_voxel const vx = frame[i + j*use_width];
      idat_chunk_png(&png, outf, vx.r);
      idat_chunk_png(&png, outf, vx.g);
      idat_chunk_png(&png, outf, vx.b);
    }
  }
  idat_adler_png(&png, outf);
  /* write the end */
  fwrite(end, sizeof(char), sizeof(end), outf);
  return;
}

void generate_crc_png(unsigned long* crc) {
  int i;
  for (i = 0; i < 256; ++i) {
    unsigned long base = i;
    int bit;
    for (bit = 0; bit < 8; ++bit) {
      unsigned long const xor = (base & 1u) ? 0xedb88320ul : 0;
      base = xor^(base >> 1);
    }
    crc[i] = base;
  }
  return;
}

int main(int argc, char **argv) {
  struct cb_matrix_array qma = {0};
  struct qbvoxel_i p = {
      &qma, cb_resize, cb_size, cb_get_matrix, cb_set_matrix,
      cb_read_voxel, cb_write_voxel
    };
  struct cb_matrix_array framebuffers = {0u};
  char const* in_filename = NULL;
  char const* out_filename = NULL;
  int res = EXIT_SUCCESS;
  int width = -1, height =-1;
  size_t center_index = ((size_t)-1);
  float modelview[16] = {1.f,0.f,0.f,0.f,  0.f,1.f,0.f,0.f,
    0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f};
  float projection[16] = {1.f,0.f,0.f,0.f,  0.f,1.f,0.f,0.f,
    0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f};
  float light_pos[3] = {0.f,1000.f,0.f};
  float ambient[3] = {0.f,0.f,0.f};
  float background[3] = {0.f,0.f,0.f};
  unsigned int righthand_tf = 0;
  int diagnose_tf = 0;
  enum output_format out_form = Output_guess;
  /* arguments */{
    int help_tf = 0;
    int argi;
    for (argi = 1; argi < argc; ++argi) {
      if (argv[argi][0] == '-' && argv[argi][1] != '\0') {
        if (strcmp(argv[argi], "-d") == 0) {
          if (argi+1 < argc) {
            ++argi;
            width = (int)strtol(argv[argi], NULL, 0);
          }
          if (argi+1 < argc) {
            ++argi;
            height = (int)strtol(argv[argi], NULL, 0);
          }
        } else if (strcmp(argv[argi], "-c") == 0) {
          if (argi+1 < argc) {
            ++argi;
            center_index = (size_t)strtoul(argv[argi], NULL, 0);
          }
        } else if (strcmp(argv[argi], "-s") == 0) {
          if (argi+1 < argc) {
            float const scale_x = (float)strtod(argv[argi+1], NULL);
            float const scale[16] = {scale_x,0.f,0.f,0.f,
              0.f,scale_x,0.f,0.f,
              0.f,0.f,scale_x,0.f, 0.f,0.f,0.f,1.f};
            ++argi;
            matrix_mult(modelview, scale, modelview);
          }
        } else if (strcmp(argv[argi], "-x") == 0) {
          if (argi+1 < argc) {
            float const move_x = (float)strtod(argv[argi+1], NULL);
            float const move[16] = {1.f,0.f,0.f,0.f,
              0.f,1.f,0.f,0.f,
              0.f,0.f,1.f,0.f, move_x,0.f,0.f,1.f};
            ++argi;
            matrix_mult(modelview, move, modelview);
          }
        } else if (strcmp(argv[argi], "-y") == 0) {
          if (argi+1 < argc) {
            float const move_y = (float)strtod(argv[argi+1], NULL);
            float const move[16] = {1.f,0.f,0.f,0.f,
              0.f,1.f,0.f,0.f,
              0.f,0.f,1.f,0.f, 0.f,move_y,0.f,1.f};
            ++argi;
            matrix_mult(modelview, move, modelview);
          }
        } else if (strcmp(argv[argi], "-z") == 0) {
          if (argi+1 < argc) {
            float const move_z = (float)strtod(argv[argi+1], NULL);
            float const move[16] = {1.f,0.f,0.f,0.f,
              0.f,1.f,0.f,0.f,
              0.f,0.f,1.f,0.f, 0.f,0.f,move_z,1.f};
            ++argi;
            matrix_mult(modelview, move, modelview);
          }
        } else if (strcmp(argv[argi], "-l") == 0) {
          res = pull_float_args(light_pos, 3, &argi, argc, argv);
        } else if (strcmp(argv[argi], "--ppm") == 0) {
          out_form = Output_PPM;
        } else if (strcmp(argv[argi], "--png") == 0) {
          out_form = Output_PNG;
        } else if (strcmp(argv[argi], "-a") == 0) {
          res = pull_float_args(ambient, 3, &argi, argc, argv);
        } else if (strcmp(argv[argi], "-b") == 0) {
          res = pull_float_args(background, 3, &argi, argc, argv);
        } else if (strcmp(argv[argi], "-q") == 0) {
          float quat[4];
          res = pull_float_args(quat, 4, &argi, argc, argv);
          /* normalize */if (res == EXIT_SUCCESS) {
            double const len = sqrt(quat[0]*(double)quat[0]
              + quat[1]*(double)quat[1]
              + quat[2]*(double)quat[2]
              + quat[3]*(double)quat[3]);
            if (len > FLT_EPSILON) {
              quat[0] /= len;
              quat[1] /= len;
              quat[2] /= len;
              quat[3] /= len;
            } else continue;
          }
          /* apply */{
            /* based on https://en.wikipedia.org/wiki/Rotation_matrix */
            float const xx = quat[0]*quat[0];
            float const yy = quat[1]*quat[1];
            float const zz = quat[2]*quat[2];
            float const xy = quat[0]*quat[1];
            float const zw = quat[2]*quat[3];
            float const xz = quat[0]*quat[2];
            float const yw = quat[1]*quat[3];
            float const yz = quat[1]*quat[2];
            float const xw = quat[0]*quat[3];
            float const turn[16] = {
                1.f-2.f*yy-2.f*zz,2.f*xy+2.f*zw,2.f*xz-2.f*yw,0.f,
                2.f*xy-2.f*zw,1.f-2.f*zz-2.f*xx,2.f*yz+2.f*xw,0.f,
                2.f*xz+2.f*yw,2.f*yz-2.f*xw,1.f-2.f*xx-2.f*yy,0.f,
                0.f,0.f,0.f,1.f
              };
            matrix_mult(modelview, turn, modelview);
          }
        } else if (strcmp(argv[argi], "-?") == 0
            ||  strcmp(argv[argi], "-h") == 0)
        {
          help_tf = 1;
        } else if (strcmp(argv[argi], "-v") == 0) {
          diagnose_tf = 1;
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
      fputs("usage: ./qbvoxel_render [options] (infile) (outfile)\n\n"
          "arguments:\n"
          "  (infile)     input file\n"
          "  (outfile)    output file\n\n"
          "options:\n"
          "  -?           print help message\n"
          "  -a (r) (g) (b)\n"
          "               ambient color, each component in range [0, 1]\n"
          "  -b (r) (g) (b)\n"
          "               background color, each component in range [0, 1]\n"
          "  -c (i)\n"
          "               center on matrix (i) (zero-index)\n"
          "               (applies before all other transforms)\n"
          "  -d (width) (height)\n"
          "               set width and height of output image\n"
          "  -l (x) (y) (z)\n"
          "               position the light\n"
          "  --png\n"
          "               force PNG output\n"
          "  --ppm\n"
          "               force PPM output\n"
          "  -q (x) (y) (z) (w)\n"
          "               rotation quaternion\n"
          "  -s (scale)\n"
          "               isotropic scale\n"
          "  -v           output some diagnostic information\n"
          "  -x (translate)\n"
          "               translate along the x-axis\n"
          "  -y (translate)\n"
          "               translate along the y-axis\n"
          "  -z (translate)\n"
          "               translate along the z-axis\n"
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
    righthand_tf = ((qbvoxel_api_get_flags(&st)&QBVoxel_FlagRightHand)!=0u);
    qbvoxel_parse_clear(&st);
  }
  /* center */if (center_index < qma.count) {
    struct cb_matrix const *const voxels = qma.matrices+center_index;
    if (voxels->active_tf) {
      unsigned int const maxdim =
          (voxels->width > voxels->height && voxels->width > voxels->depth)
        ? voxels->width
        : ((voxels->height > voxels->depth) ? voxels->height : voxels->depth);
      float const center_point[3] = {
          voxels->x + voxels->width*0.5f,
          voxels->y + voxels->height*0.5f,
          voxels->z + voxels->depth*0.5f,
        };
      float const center_scale[3] = {
          0.625f/(float)maxdim,
          0.625f/(float)maxdim,
          0.625f/(float)maxdim,
        };
      float const center_matrix[16] = {
          center_scale[0], 0.f, 0.f, 0.f,
          0.f, center_scale[1], 0.f, 0.f,
          0.f, 0.f, center_scale[2], 0.f,
          /* */
          -center_scale[0]*center_point[0], -center_scale[1]*center_point[1],
              -center_scale[2]*center_point[2], 1.f
        };
      matrix_mult(modelview, modelview, center_matrix);
    }
  }
  /* adjust the projection matrix */{
    if (width < 0)
      width = 64;
    if (height < 0)
      height = 64;
    if (width < height) {
      projection[5] = width/(float)height;
    } else if (width > height) {
      projection[0] = height/(float)width;
    }
  }
  /* construct the frame buffer(s) */if (res == EXIT_SUCCESS) {
    unsigned int const use_width = (unsigned int)width;
    unsigned int const use_height = (unsigned int)height;
    int const resize_err = cb_resize(&framebuffers, 1u);
    if (resize_err != 0) {
      fprintf(stderr, "framebuffer creation error %i\n", resize_err);
      res = EXIT_FAILURE;
    } else {
      int set_err;
      struct qbvoxel_matrix_info info = {{0}};
      info.size_x = use_width;
      info.size_y = use_height;
      info.size_z = 1u;
      set_err = cb_set_matrix(&framebuffers, 0u, &info);
      if (set_err != 0) {
        fprintf(stderr, "framebuffer creation error %i\n", set_err);
        res = EXIT_FAILURE;
      }
    }
    if (res == EXIT_SUCCESS) {
      struct qbvoxel_voxel *const frame = framebuffers.matrices[0].data;
      /* adjust for handedness of file */
      float const z_near = righthand_tf ? +1.f : -1.f;
      float const z_far = righthand_tf ? -1.f : +1.f;
      /* clear the buffer */{
        struct qbvoxel_voxel const clear_color = {
            (unsigned char)clamp255(background[0]),
            (unsigned char)clamp255(background[1]),
            (unsigned char)clamp255(background[2]),
            255u
          };
        size_t const data_count = use_height*(size_t)use_width;
        size_t data_pos;
        for (data_pos = 0u; data_pos < data_count; ++data_pos) {
          frame[data_pos] = clear_color;
        }
      }
      /* ray trace */{
        float inv_matrix[16];
        unsigned int j;
        float const hf = (float)(2.0/use_height);
        float const wf = (float)(2.0/use_width);
        size_t k;
        /* */{
          float fwd_matrix[16];
          matrix_mult(fwd_matrix, projection, modelview);
          /* */if (diagnose_tf) {
            size_t m_i;
            fprintf(stderr, "fwd_matrix = {");
            for (m_i = 0u; m_i < 16; ++m_i) {
              fprintf(stderr, "%f,", fwd_matrix[m_i]);
            }
            fprintf(stderr, "}\n");
          }
          /* */{
            int const inv_res = matrix_invert(inv_matrix, fwd_matrix);
            if (inv_res) {
              if (diagnose_tf) {
                size_t m_i;
                fprintf(stderr, "inv_matrix = {");
                for (m_i = 0u; m_i < 16; ++m_i) {
                  fprintf(stderr, "%f,", inv_matrix[m_i]);
                }
                fprintf(stderr, "}\n");
              }
            } else fprintf(stderr, "invert failed!\n");
          }
        }
        for (k = 0u; k < qma.count; ++k) {
          struct cb_matrix const *const voxels = qma.matrices+k;
          if (!voxels->active_tf)
            continue;
          if (voxels->width == 0u || voxels->height == 0u
          ||  voxels->depth == 0u)
            continue;
          if (diagnose_tf) {
            fprintf(stderr, "matrices[%lu] = {%ux%ux%u at %i,%i,%i}\n",
              (long unsigned int)k,
              voxels->width, voxels->height, voxels->depth,
              voxels->x, voxels->y, voxels->z);
          }
          for (j = 0u; j < use_height; ++j) {
            unsigned int i;
            float const y = -(j*hf-1.f);
            for (i = 0u; i < use_width; ++i) {
              float const x = i*wf-1.f;
              float ray_start[4] = {x,y,z_near,1.f};
              float ray_end[4] = {x,y,z_far,1.f};
              size_t const data_pos = i+j*use_width;
              matrix_vec_multw(ray_start, inv_matrix, ray_start);
              matrix_vec_multw(ray_end, inv_matrix, ray_end);
              /* trace the ray */{
                float color[4] = {0.f,0.f,0.f,0.f};
                float normal[3];
                float const z =
                  trace_ray(color, normal, voxels, ray_start, ray_end);
                unsigned char const z_value = clamp254(z);
                if (color[3] <= 0.f || z < 0.f)
                  continue;
                /* z test */{
                  if (z_value > frame[data_pos].a)
                    continue;
                }
                /* compute lighting */{
                  float raster_pos[3];
                  ray_interpolate(raster_pos, z, ray_start, ray_end);
                  pixel_shade
                    (color, raster_pos, light_pos, ambient, normal, &qma);
                }
                /* post fragment */{
                  struct qbvoxel_voxel next_pixel =
                     { (unsigned char)clamp255(color[0]),
                       (unsigned char)clamp255(color[1]),
                       (unsigned char)clamp255(color[2]), z_value};
                  frame[data_pos] = next_pixel;
                }
              }/* end trace the ray */
            }
          }
        }
      }/* end ray trace */
    }
  }
  /* generate */if (res == EXIT_SUCCESS) {
    FILE* outf = out_filename !=NULL
      ? ((strcmp(out_filename,"-") == 0) ? stdout : fopen(out_filename, "wb"))
      : NULL;
    if (out_filename == NULL) {
      fputs("missing output file name\n", stderr);
    } else if (outf == NULL) {
      perror("can not open file");
      res = EXIT_FAILURE;
    } else /* generate */ {
      if (out_form == Output_guess && out_filename) {
        /* guess the desired format */
        size_t const len = strlen(out_filename);
        char suffix[5] = {'\0'};
        if (len > 4) {
          size_t suffix_i;
          for (suffix_i = 0; suffix_i < 4; ++suffix_i)
            suffix[suffix_i] = tolower(out_filename[len-4+suffix_i]&255u);
        }
        if (strncmp(suffix,".png",sizeof(suffix)) == 0)
          out_form = Output_PNG;
      }
      switch (out_form) {
      case Output_PNG:
        write_to_png(outf, framebuffers.matrices[0].width,
          framebuffers.matrices[0].height, framebuffers.matrices[0].data);
        break;
      default:
        write_to_ppm(outf, framebuffers.matrices[0].width,
          framebuffers.matrices[0].height, framebuffers.matrices[0].data);
        break;
      }
    }
    if (outf != NULL && outf != stdout)
      fclose(outf);
  }
  cb_resize(&framebuffers,0);
  cb_resize(&qma,0);
  return res;
}
