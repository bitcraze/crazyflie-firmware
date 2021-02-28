/*-------------------------------------------*
 * Integer type definitions for FatFs module *
 *-------------------------------------------*/

#ifndef _FF_INTEGER
#define _FF_INTEGER

/* Integer types used for FatFs API */

#ifdef _WIN32

#include <windows.h>
typedef unsigned __int64 QWORD;

#else

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L) || defined(__cplusplus)

#include <stdint.h>
typedef uint8_t   BYTE; /* MUST be 8-bit */
typedef uint16_t  WORD; /* MUST be 16-bit */
typedef uint32_t DWORD; /* MUST be 32-bit */
typedef uint64_t QWORD; /* MUST be 64-bit */

#else

typedef unsigned char       BYTE; /* MUST be 8-bit */
typedef unsigned short      WORD; /* MUST be 16-bit */
typedef unsigned long      DWORD; /* MUST be 32-bit */
/* No QWORD because 'unsigned long long' not part of ANSI C89 / ISO C90 */
#define FF_QWORD_UNSUPPORTED

#endif

typedef WORD WCHAR; /* UTF-16 character type */

typedef          short SHORT;
typedef          int     INT;
typedef unsigned int    UINT;
typedef          long   LONG;

#endif

#endif /* _FF_INTEGER */
