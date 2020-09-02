/**
 * @file qbparse/api.h
 * @brief Base file for the Qubicle parser library
 */
#if !defined(hg_QBParse_api_h_)
#define hg_QBParse_api_h_

#if defined(_WIN32)
#  if defined(QBParse_API_Impl)
#    define QBParse_API __declspec(dllexport)
#  else
#    define QBParse_API __declspec(dllimport)
#  endif /*QBParse_API_Impl*/
#else
#  define QBParse_API
#endif /*_WIN32*/

#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus*/

/**
 * @brief Error codes.
 */
enum qbparse_error {
  /**
   * @brief All good. (zero)
   */
  QBParse_Ok = 0,
  /**
   * @brief Version not supported.
   */
  QBParse_ErrVersion = 1,
  /**
   * @brief Color type not supported.
   */
  QBParse_ErrColorType = 2,
  /**
   * @brief Orientation not supported.
   */
  QBParse_ErrOrientation = 3,
  /**
   * @brief Mask format not supported.
   */
  QBParse_ErrMaskFormat = 4,
  /**
   * @brief Compression format not supported.
   */
  QBParse_ErrCompress = 5
};

/**
 * @brief Flags for the state machine.
 */
enum qbparse_flags {
  /**
   * @b Set: Data encoded as BGRA; @b Clear: Data encoded as RGBA
   */
  QBParse_FlagBGRA = 1u,
  /**
   * @b Set: Right-handed coordinate system;
   * @b Clear: Left-handed coordinate system
   */
  QBParse_FlagRightHand = 2u,
  /**
   * @b Set: Run length encoding of matrix data
   * @b Clear: Direct storage of matrix data
   */
  QBParse_FlagRLE = 4u,
  /**
   * @b Set: Visibility masks encoded in alpha channel
   * @b Clear: Alpha channel is either full opaque or full transparent
   */
  QBParse_FlagSideMasks = 8u
};



/**
 * @brief Voxel information.
 */
typedef struct qbparse_voxel {
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
} qbparse_voxel;

/**
 * @brief QBParse matrix information structure.
 */
typedef struct qbparse_matrix_info {
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
} qbparse_matrix_info;

/**
 * @brief QBParse interface
 */
typedef struct qbparse_i {
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
    (void const* p, unsigned long int i, struct qbparse_matrix_info *mi);
  /**
   * @brief Configure a matrix.
   * @param p this instance
   * @param i a matrix array index
   * @param mi matrix information structure to use for configuration
   * @return 0 on success, nonzero otherwise
   */
  int (*set_matrix)
    (void* p, unsigned long int i, struct qbparse_matrix_info const* mi);
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
      struct qbparse_voxel* v);
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
      struct qbparse_voxel const* v);
} qbparse_i;

typedef struct qbparse_state {
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
  struct qbparse_i* cb;
} qbparse_state;

/**
 * @brief Get a version string.
 */
QBParse_API
char const* qbparse_api_version(void);

/**
 * @brief Extract an unsigned 32-bit integer from a byte array.
 * @param b byte array
 * @return the unsigned integer
 */
QBParse_API
unsigned long int qbparse_api_from_u32(unsigned char const* b);

/**
 * @brief Encode an unsigned 32-bit integer to a byte array.
 * @param b byte array
 * @param v the unsigned integer
 */
QBParse_API
void qbparse_api_to_u32(unsigned char* b, unsigned long int v);

/**
 * @brief Extract a signed 32-bit integer to a byte array.
 * @param b byte array
 * @return the signed integer
 */
QBParse_API
long int qbparse_api_from_i32(unsigned char const* b);

/**
 * @brief Encode a signed 32-bit integer to a byte array.
 * @param b byte array
 * @param v the signed integer
 */
QBParse_API
void qbparse_api_to_i32(unsigned char* b, long int v);

/**
 * @brief Query a state for the last error code.
 * @param s the state structure to query
 * @return zero if no error, nonzero otherwise
 */
QBParse_API
int qbparse_api_get_error(struct qbparse_state const* s);

/**
 * @brief Allocate some memory.
 * @param sz number of bytes to allocate
 * @return pointer to memory on success, NULL on failure
 */
QBParse_API
void* qbparse_api_malloc(unsigned int sz);

/**
 * @brief Release some memory.
 * @param p pointer to memory to free
 */
QBParse_API
void qbparse_api_free(void* p);

/**
 * @brief Allocate enough memory for a single state.
 * @param sz number of bytes to allocate
 * @return pointer to memory on success, NULL on failure
 * @note Release with `qbparse_api_free`.
 */
QBParse_API
struct qbparse_state* qbparse_api_alloc_state(void);


#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBParse_api_h_*/
