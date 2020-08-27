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

#if defined(__cplusplus)
};
#endif /*__cplusplus*/

#endif /*hg_QBParse_QBBase_h_*/
