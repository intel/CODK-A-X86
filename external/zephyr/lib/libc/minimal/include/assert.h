/**
 assert.h
*/

#ifndef __INC_assert_h__
#define __INC_assert_h__

#include <stdio.h>

void __assert_hook(const char *, int , const char *, const char * );

#undef assert

#ifdef NDEBUG           /* required by ANSI standard */
#define assert(__e) ((void)0)
#else                   /* !NDEBUG */
#define assert(__e) ((__e) ? (void)0 : __assert_hook(__FILE__, __LINE__, \
						       __func__, #__e))
#endif

#endif /* __INC_assert_h__ */
