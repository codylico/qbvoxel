/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/parse.h
 * @brief Parser backend for the Qubicle parser library
 */
#if !defined(hg_QBVoxel_parse_h_)
#define hg_QBVoxel_parse_h_

#include "api.h"

#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus*/

/**
 * @brief Initialize a state for parsing.
 * @param s the state structure to initialize
 * @param cb the callback interface to use
 * @return zero on success
 */
QBVoxel_API
int qbvoxel_parse_init(struct qbvoxel_state *s, struct qbvoxel_i* cb);

/**
 * @brief Close a state for parsing.
 * @param s the state structure to close
 */
QBVoxel_API
void qbvoxel_parse_clear(struct qbvoxel_state *s);

/**
 * @brief Parse the next piece of a binary stream.
 * @param s parser state
 * @param sz size of block to parse
 * @param buf block of bytes to parse
 * @return the number of bytes parsed
 */
QBVoxel_API
unsigned int qbvoxel_parse_do
  (struct qbvoxel_state *s, unsigned int sz, unsigned char const* buf);

#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBVoxel_parse_h_*/
