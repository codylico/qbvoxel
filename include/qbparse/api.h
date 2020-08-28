/**
 * @file qbparse/api.h
 * @brief Base file for the Qubicle parser library
 */
#if !defined(hg_QBParse_QBBase_h_)
#define hg_QBParse_QBBase_h_

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

#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBParse_QBBase_h_*/
