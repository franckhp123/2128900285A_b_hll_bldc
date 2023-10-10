/*
 * Copyright (C) 2014 Renesas Electronics Corporation
 * RENESAS ELECTRONICS CONFIDENTIAL AND PROPRIETARY.
 * This program must be used solely for the purpose for which
 * it was furnished by Renesas Electronics Corporation. No part of this
 * program may be reproduced or disclosed to others, in any
 * form, without the prior written permission of Renesas Electronics
 * Corporation.
 */

/*****************************************************************************/
/* CC-RL library                                                             */
/*****************************************************************************/

#ifndef _STRING_H
#define _STRING_H

#include <stddef.h>

void __near * __far memcpy(void __near *s1, const void __near *s2, size_t n);
void __far * __far _COM_memcpy_ff(void __far *s1, const void __far *s2, size_t n);

void __near * __far memmove(void __near *s1, const void __near *s2, size_t n);
void __far * __far _COM_memmove_ff(void __far *s1, const void __far *s2, size_t n);

char __near * __far strcpy(char __near *s1, const char __near *s2);
char __far * __far _COM_strcpy_ff(char __far *s1, const char __far *s2);

char __near * __far strncpy(char __near *s1, const char __near *s2, size_t n);
char __far * __far _COM_strncpy_ff(char __far *s1, const char __far *s2, size_t n);

char __near * __far strcat(char __near *s1, const char __near *s2);
char __far * __far _COM_strcat_ff(char __far *s1, const char __far *s2);

char __near * __far strncat(char __near *s1, const char __near *s2, size_t n);
char __far * __far _COM_strncat_ff(char __far *s1, const char __far *s2, size_t n);

int __far memcmp(const void __near *s1, const void __near *s2, size_t n);
int __far _COM_memcmp_ff(const void __far *s1, const void __far *s2, size_t n);

int __far strcmp(const char __near *s1, const char __near *s2);
int __far _COM_strcmp_ff(const char __far *s1, const char __far *s2);

int __far strncmp(const char __near *s1, const char __near *s2, size_t n);
int __far _COM_strncmp_ff(const char __far *s1, const char __far *s2, size_t n);

void __near * __far memchr(const void __near *s, int c, size_t n);
void __far * __far _COM_memchr_f(const void __far *s, int c, size_t n);

char __near * __far strchr(const char __near *s, int c);
char __far * __far _COM_strchr_f(const char __far *s, int c);

size_t __far strcspn(const char __near *s1, const char __near *s2);
size_t __far _COM_strcspn_ff(const char __far *s1, const char __far *s2);

char __near * __far strpbrk(const char __near *s1, const char __near *s2);
char __far * __far _COM_strpbrk_ff(const char __far *s1, const char __far *s2);

char __near * __far strrchr(const char __near *s, int c);
char __far * __far _COM_strrchr_f(const char __far *s, int c);

size_t __far strspn(const char __near *s1, const char __near *s2);
size_t __far _COM_strspn_ff(const char __far *s1, const char __far *s2);

char __near * __far strstr(const char __near *s1, const char __near *s2);
char __far * __far _COM_strstr_ff(const char __far *s1, const char __far *s2);

char __far * __far strtok(char __far *s1, const char __far *s2);

void __near * __far memset(void __near *s, int c, size_t n);
void __far * __far _COM_memset_f(void __far *s, int c, size_t n);

char __far * __far strerror(int errnum);

size_t __far strlen(const char __near *s);
size_t __far _COM_strlen_f(const char __far *s);

#if defined(__FAR_ROM__)
#define memcpy(s1, s2, n)   _COM_memcpy_ff(s1, s2, n)
#define memmove(s1, s2, n)  _COM_memmove_ff(s1, s2, n)
#define strcpy(s1, s2)      _COM_strcpy_ff(s1, s2)
#define strncpy(s1, s2, n)  _COM_strncpy_ff(s1, s2, n)
#define strcat(s1, s2)      _COM_strcat_ff(s1, s2)
#define strncat(s1, s2, n)  _COM_strncat_ff(s1, s2, n)
#define memcmp(s1, s2, n)   _COM_memcmp_ff(s1, s2, n)
#define strcmp(s1, s2)      _COM_strcmp_ff(s1, s2)
#define strncmp(s1, s2, n)  _COM_strncmp_ff(s1, s2, n)
#define memchr(s, c, n)     _COM_memchr_f(s, c, n)
#define strchr(s, c)        _COM_strchr_f(s, c)
#define strcspn(s1, s2)     _COM_strcspn_ff(s1, s2)
#define strpbrk(s1, s2)     _COM_strpbrk_ff(s1, s2)
#define strrchr(s, c)       _COM_strrchr_f(s, c)
#define strspn(s1, s2)      _COM_strspn_ff(s1, s2)
#define strstr(s1, s2)      _COM_strstr_ff(s1, s2)
#define memset(s, c, n)     _COM_memset_f(s, c, n)
#define strlen(s)           _COM_strlen_f(s)
#endif  /* __FAR_ROM__ */

#endif  /* !_STRING_H */
