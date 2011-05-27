/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatbasictypes.h
    @brief This file contains only definitions of basic types such as
    integers, floats, pointers, booleans. */
/****************************************************************************/

#ifndef ECATBASICTYPES_H
#define ECATBASICTYPES_H

#include "ecatslaveconfig.h"

#define IN  /*!< @brief Empty macro used to mark input  parameters. */
#define OUT /*!< @brief Empty macro used to mark output parameters. */

#ifndef _WIN32
#   define __cdecl
#endif

/** @defgroup ecatslave_basictypes Basic data types */
/** @addtogroup ecatslave_basictypes
    @{ */

/* Integer data types */

typedef unsigned char       UINT8;              /*!< 8-bit unsigned integer.    */
typedef signed char         INT8;               /*!< 8-bit signed integer.      */

typedef unsigned short      UINT16;             /*!< 16-bit unsigned integer.   */
typedef signed short        INT16;              /*!< 16-bit signed integer.     */

typedef unsigned int        UINT32;             /*!< 32-bit unsigned integer.   */
typedef signed int          INT32;              /*!< 32-bit signed integer.     */

#ifdef _WIN32
typedef unsigned __int64    UINT64;             /*!< 64-bit unsigned integer.   */
typedef signed __int64      INT64;              /*!< 64-bit signed integer.     */
#else /* LINUX */
typedef unsigned long long  UINT64;             /*!< 64-bit unsigned integer.   */
typedef signed long long    INT64;              /*!< 64-bit signed integer.     */
#endif

/*--------------------------------------------------------------------------*/

#define ECAT_TRUE           1               /*!< Logical true.  */
#define ECAT_FALSE          0               /*!< Logical false. */

/*! @brief Boolean (#ECAT_TRUE or #ECAT_FALSE). */
typedef UINT32              ECAT_BOOL;

/*! @brief 32-bit floating-point number.        */
typedef float               ECAT_REAL32;
/*! @brief 64-bit floating-point number.        */
typedef double              ECAT_REAL64;


typedef char                ECAT_CHAR;      /*!< Char (8bit). */
typedef ECAT_CHAR*          ECAT_PCHAR;
typedef ECAT_CHAR*          ECAT_STR;
typedef ECAT_CHAR const *   ECAT_CSTR;

typedef unsigned short      ECAT_WCHAR;     /*!< Wide char(16 bit unsigned). */
typedef ECAT_WCHAR*         ECAT_PWCHAR;
typedef ECAT_WCHAR*         ECAT_WSTR;
typedef ECAT_WCHAR const *  ECAT_CWSTR;

typedef unsigned char       ECAT_BYTE;      /*!< Byte (8bit unsigned). */
typedef ECAT_BYTE*          ECAT_PBYTE;

typedef short               ECAT_SWORD;     /*!< Word (16bit signed).  */
typedef unsigned short      ECAT_WORD;      /*!< Word (16bit unsigned).*/
typedef ECAT_WORD*          ECAT_PWORD;

typedef unsigned long       ECAT_DWORD;     /*!< Double Word (32bit unsigned). */
typedef ECAT_DWORD*         ECAT_PDWORD;

typedef long                ECAT_SDWORD;    /*!< Signed double Word (32bit signed). */
typedef ECAT_SDWORD*        ECAT_PSDWORD;

typedef unsigned char       ECAT_UINT8;     /*!< 8-bit unsigned integer.    */
typedef signed char         ECAT_INT8;      /*!< 8-bit signed integer.      */

typedef unsigned short      ECAT_UINT16;    /*!< 16-bit unsigned integer.   */
typedef signed short        ECAT_INT16;     /*!< 16-bit signed integer.     */

typedef signed long         ECAT_INT32;     /*!< Signed integer (32bit signed). */
typedef unsigned long       ECAT_UINT32;    /*!< Unsigned integer (32 bit unsigned). */
typedef ECAT_UINT32         ECAT_SIZE;

#ifdef _WIN32
typedef __int64                 ECAT_LONGLONG;  /*!< Signed 64-bit integer.   */
typedef unsigned __int64        ECAT_ULONGLONG; /*!< Unsigned 64-bit integer. */
#else
typedef long long int           ECAT_LONGLONG;  /*!< Signed 64-bit integer.   */
typedef unsigned long long int  ECAT_ULONGLONG; /*!< Unsigned 64-bit integer. */
#endif

typedef ECAT_ULONGLONG      ECAT_UINT64;    /*!< Signed 64-bit integer.   */
typedef ECAT_LONGLONG       ECAT_INT64;     /*!< Unsigned 64-bit integer. */

#define ECAT_QWORD          ECAT_UINT64     /*!< Unsigned quad-word (64-bit integer). */


typedef void*               ECAT_HMODULE;

#ifdef __cplusplus
#define ECAT_NULL           0               /*!< A null pointer constant. */
#else
#define ECAT_NULL           ((void *)0)     /*!< A null pointer constant. */
#endif

typedef void*               ECAT_HANDLE;    /*!< A handle. */
typedef ECAT_HANDLE*        ECAT_PHANDLE;
#define ECAT_INVALID_HANDLE ((void*)0)

typedef ECAT_DWORD          ECAT_TIME_MS;   /*!< Represents number of milliseconds. */
typedef ECAT_ULONGLONG      ECAT_TIME_US;   /*!< Represents number of microseconds. */
typedef ECAT_ULONGLONG      ECAT_TIME_NS;   /*!< Represents number of nanoseconds.  */

typedef void*               ECAT_LPVOID;
typedef void                ECAT_VOID;


/*! @typedef ECAT_IOMEM
    Type of pointer to data that(data) must not be cashed. Data stored at the
    location the pointer points to may be changed outside the application. */
typedef volatile void*      ECAT_IOMEM;

/*! @} marks end of group */

/*! @def ECAT_INLINE
    @brief Platform-independent marker for a function
    which code must be inlined in a place where a function is called. */
#ifndef ECAT_INLINE
#   if defined(_WIN32)
#       if defined(__cplusplus)
#           define ECAT_INLINE inline
#       else
#           define ECAT_INLINE __inline
#       endif
#   else
#       define ECAT_INLINE static inline
#   endif
#endif /* #ifndef ECAT_INLINE */


#endif /* ECATBASICTYPES_H */
