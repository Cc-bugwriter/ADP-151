/*
******************************************************************************
**  CarMaker - Version 6.0.2
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#ifndef _GLOBAL_H__
#define _GLOBAL_H__

#if defined(DSPACE) || defined(LVRT)
# include <ipgrt.h>
#endif

#if defined(INTIME) || defined(ARTE_INTIME)
# if defined(__GNUC__)
#  include <intime.h>
# endif
#endif

#if defined(__GNUC__)
# ifndef CM_PRINTF
#  if __GNUC__ >= 5 || __GNUC__ >= 4 && __GNUC_MINOR__ >= 4
#   define CM_PRINTF gnu_printf
#  else
#   define CM_PRINTF printf
#  endif
# endif
#else
# if !defined(__inline__)
#  if defined(_MCCPPC)
#   define __inline__		inline
#  else
#   define __inline__		/* nothing */
#  endif
# endif
# if !defined(__attribute__)
#  define __attribute__(x)	/* nothing */
# endif

#endif

#ifdef __cplusplus
extern "C" {
#endif


/* ... */


#ifdef __cplusplus
}
#endif

#endif  /* !defined(_GLOBAL_H__) */

