/* SPDX-License-Identifier: Unlicense */
/**
 * @file qbvoxel/api.h
 * @brief Base file for the Qubicle parser library
 */
#if !defined(hg_QBVoxel_api_h_)
#define hg_QBVoxel_api_h_

#if defined(_WIN32) && defined(QBVoxel_WIN32_DLL)
#  if defined(QBVoxel_API_Impl)
#    define QBVoxel_API __declspec(dllexport)
#  else
#    define QBVoxel_API __declspec(dllimport)
#  endif /*QBVoxel_API_Impl*/
#else
#  define QBVoxel_API
#endif /*_WIN32*/

#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus*/

/**
 * @brief Error codes.
 */
enum qbvoxel_error {
  /**
   * @brief All good. (zero)
   */
  QBVoxel_Ok = 0,
  /**
   * @brief Version not supported.
   */
  QBVoxel_ErrVersion = 1,
  /**
   * @brief Color type not supported.
   */
  QBVoxel_ErrColorType = 2,
  /**
   * @brief Orientation not supported.
   */
  QBVoxel_ErrOrientation = 3,
  /**
   * @brief Mask format not supported.
   */
  QBVoxel_ErrMaskFormat = 4,
  /**
   * @brief Compression format not supported.
   */
  QBVoxel_ErrCompress = 5,
  /**
   * @brief Index out of range.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_ErrOutOfRange = 6,
  /**
   * @brief Out of memory.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_ErrMemory = 7,
  /**
   * @brief File not found.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_ErrIONotFound = 8,
  /**
   * @brief Generic input/output error.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_ErrIOGeneric = 9,
  /**
   * @brief Matrix count not supported.
   * @note Useful in applications expecting a single matrix per file.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_ErrMatrixCount = 10,
  /**
   * @brief First error code guaranteed unused by this library.
   * @note This code not emitted directly by this library.
   */
  QBVoxel_UserError = 32
};

/**
 * @brief Flags for the state machine.
 */
enum qbvoxel_flags {
  /**
   * @b Set: Data encoded as BGRA; @b Clear: Data encoded as RGBA
   */
  QBVoxel_FlagBGRA = 1u,
  /**
   * @b Set: Right-handed coordinate system;
   * @b Clear: Left-handed coordinate system
   */
  QBVoxel_FlagRightHand = 2u,
  /**
   * @b Set: Run length encoding of matrix data
   * @b Clear: Direct storage of matrix data
   */
  QBVoxel_FlagRLE = 4u,
  /**
   * @b Set: Visibility masks encoded in alpha channel
   * @b Clear: Alpha channel is either full opaque or full transparent
   */
  QBVoxel_FlagSideMasks = 8u
};



/**
 * @brief Voxel information.
 */
typedef struct qbvoxel_voxel {
    /**
     * @brief Red channel.
     */
    unsigned char r;
    /**
     * @brief Green channel.
     */
    unsigned char g;
    /**
     * @brief Blue channel.
     */
    unsigned char b;
    /**
     * @brief Visibility information.
     */
    unsigned char a;
} qbvoxel_voxel;

/**
 * @brief QBVoxel matrix information structure.
 */
typedef struct qbvoxel_matrix_info {
  /**
   * @brief Matrix name, `'\0'`-terminated.
   */
  char name[256];
  /**
   * @brief x-coordinate of matrix in space.
   */
  long int pos_x;
  /**
   * @brief y-coordinate of matrix in space.
   */
  long int pos_y;
  /**
   * @brief z-coordinate of matrix in space.
   */
  long int pos_z;
  /**
   * @brief Width of matrix in voxels.
   */
  unsigned long int size_x;
  /**
   * @brief Height of matrix in voxels.
   */
  unsigned long int size_y;
  /**
   * @brief Depth of matrix in voxels.
   */
  unsigned long int size_z;
} qbvoxel_matrix_info;

/**
 * @brief QBVoxel interface
 */
typedef struct qbvoxel_i {
  /**
   * @brief Callback data.
   */
  void* p;
  /**
   * @brief Resize the voxel matrix store.
   * @param p this instance
   * @param n number of matrices
   * @return 0 on success, nonzero otherwise
   */
  int (*resize)(void* p, unsigned long int n);
  /**
   * @brief Query the number of voxel matrices.
   * @param p this instance
   * @return a voxel matrix count
   */
  unsigned long int (*size)(void const* p);
  /**
   * @brief Query information for a matrix.
   * @param p this instance
   * @param i a matrix array index
   * @param[out] mi matrix information structure to fill
   * @return 0 on success, nonzero otherwise
   */
  int (*get_matrix)
    (void const* p, unsigned long int i, struct qbvoxel_matrix_info *mi);
  /**
   * @brief Configure a matrix.
   * @param p this instance
   * @param i a matrix array index
   * @param mi matrix information structure to use for configuration
   * @return 0 on success, nonzero otherwise
   */
  int (*set_matrix)
    (void* p, unsigned long int i, struct qbvoxel_matrix_info const* mi);
  /**
   * @brief Query information for a matrix.
   * @param p this instance
   * @param i a matrix array index
   * @param x matrix-local x-coordinate
   * @param y matrix-local y-coordinate
   * @param z matrix-local z-coordinate
   * @param[out] v voxel structure to receive channel data
   * @return 0 on success, nonzero otherwise
   */
  int (*read_voxel)
    ( void const* p, unsigned long int i,
      unsigned long int x,unsigned long int y,unsigned long int z,
      struct qbvoxel_voxel* v);
  /**
   * @brief Query information for a matrix.
   * @param p this instance
   * @param i a matrix array index
   * @param x matrix-local x-coordinate
   * @param y matrix-local y-coordinate
   * @param z matrix-local z-coordinate
   * @param v voxel structure providing channel data
   * @return 0 on success, nonzero otherwise
   */
  int (*write_voxel)
    ( void* p, unsigned long int i,
      unsigned long int x,unsigned long int y,unsigned long int z,
      struct qbvoxel_voxel const* v);
} qbvoxel_i;

/**
 * @brief Parser/generator state.
 */
typedef struct qbvoxel_state {
  int last_error;
  unsigned char state;
  unsigned char flags;
  unsigned short pos;
  unsigned long int i;
  unsigned long int x;
  unsigned long int y;
  unsigned long int z;
  unsigned long int width;
  unsigned long int height;
  unsigned long int depth;
  unsigned char buffer[32];
  struct qbvoxel_i* cb;
  unsigned char name_buffer[256];
} qbvoxel_state;

/**
 * @brief Get a version string.
 */
QBVoxel_API
char const* qbvoxel_api_version(void);

/**
 * @brief Extract an unsigned 32-bit integer from a byte array.
 * @param b byte array
 * @return the unsigned integer
 */
QBVoxel_API
unsigned long int qbvoxel_api_from_u32(unsigned char const* b);

/**
 * @brief Encode an unsigned 32-bit integer to a byte array.
 * @param b byte array
 * @param v the unsigned integer
 */
QBVoxel_API
void qbvoxel_api_to_u32(unsigned char* b, unsigned long int v);

/**
 * @brief Extract a signed 32-bit integer to a byte array.
 * @param b byte array
 * @return the signed integer
 */
QBVoxel_API
long int qbvoxel_api_from_i32(unsigned char const* b);

/**
 * @brief Encode a signed 32-bit integer to a byte array.
 * @param b byte array
 * @param v the signed integer
 */
QBVoxel_API
void qbvoxel_api_to_i32(unsigned char* b, long int v);

/**
 * @brief Query a state for the last error code.
 * @param s the state structure to query
 * @return zero if no error, nonzero otherwise
 */
QBVoxel_API
int qbvoxel_api_get_error(struct qbvoxel_state const* s);

/**
 * @brief Allocate some memory.
 * @param sz number of bytes to allocate
 * @return pointer to memory on success, NULL on failure
 */
QBVoxel_API
void* qbvoxel_api_malloc(unsigned int sz);

/**
 * @brief Release some memory.
 * @param p pointer to memory to free
 */
QBVoxel_API
void qbvoxel_api_free(void* p);

/**
 * @brief Allocate enough memory for a single state.
 * @param sz number of bytes to allocate
 * @return pointer to memory on success, NULL on failure
 * @note Release with `qbvoxel_api_free`.
 */
QBVoxel_API
struct qbvoxel_state* qbvoxel_api_alloc_state(void);

/**
 * @brief Query a state for the encode flags.
 * @param s the state structure to query
 * @return encode flags
 */
QBVoxel_API
unsigned int qbvoxel_api_get_flags(struct qbvoxel_state const* s);

/**
 * @brief Query a state for the encode flags.
 * @param s the state structure to query
 * @param f encode flags
 */
QBVoxel_API
void qbvoxel_api_set_flags(struct qbvoxel_state* s, unsigned int f);


#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBVoxel_api_h_*/
