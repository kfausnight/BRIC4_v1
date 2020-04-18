/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/
#include <stdint.h>

#ifndef _INTEGER
#define _INTEGER

#ifdef _WIN32	/* FatFs development platform */

#include <windows.h>
#include <tchar.h>

#else			/* Embedded platform */

/* These types must be 16-bit, 32-bit or larger integer */
//typedef int				INT;
//typedef unsigned int	UINT;
typedef int32_t		INT;
//typedef uint32_t	UINT;//Deleted Kfausnight 11/24/2018


/* These types must be 8-bit integer */
//typedef char			CHAR;
//typedef unsigned char	UCHAR;
//typedef unsigned char	BYTE;
typedef uint8_t			CHAR;
typedef uint8_t		UCHAR;
typedef uint8_t		BYTE;

/* These types must be 16-bit integer */
//typedef short			SHORT;
//typedef unsigned short	USHORT;
//typedef unsigned short	WORD;
//typedef unsigned short	WCHAR;
typedef int16_t			SHORT;
typedef uint16_t	USHORT;
typedef uint16_t	WORD;
typedef uint16_t	WCHAR;

/* These types must be 32-bit integer */
//typedef long			LONG;
//typedef unsigned long	ULONG;
//typedef unsigned long	DWORD;
typedef int32_t			LONG;
typedef uint32_t	ULONG;
typedef uint32_t	DWORD;

#endif

#endif
