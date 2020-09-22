/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/gen.h
 * @brief Generator backend for the Qubicle parser library
 */
#if !defined(hg_QBVoxel_gen_h_)
#define hg_QBVoxel_gen_h_

#include "api.h"

#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus*/

/**
 * @brief Initialize a state for generating.
 * @param s the state structure to initialize
 * @param cb the callback interface to use
 * @return zero on success
 */
QBVoxel_API
int qbvoxel_gen_init(struct qbvoxel_state *s, struct qbvoxel_i* cb);

/**
 * @brief Close a state for generating.
 * @param s the state structure to close
 */
QBVoxel_API
void qbvoxel_gen_clear(struct qbvoxel_state *s);

/**
 * @brief Generate the next piece of a binary stream.
 * @param s generator state
 * @param sz size of block to receive generated bytes
 * @param[out] buf block of generated bytes
 * @return the number of bytes generated
 */
QBVoxel_API
unsigned int qbvoxel_gen_do
  (struct qbvoxel_state *s, unsigned int sz, unsigned char* buf);

#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBVoxel_gen_h_*/