/*
 * Copyright (C) 2014, 2015 Renesas Electronics Corporation
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

#ifndef _STDLIB_H
#define _STDLIB_H

#include <stddef.h>
#define EXIT_SUCCESS    0
#define EXIT_FAILURE    1
#define RAND_MAX        0x7fff

typedef struct {
    int quot;
    int rem;
} div_t;

typedef struct {
    long int quot;
    long int rem;
} ldiv_t;

double __far atof(const char __near *nptr);
double __far _COM_atof_f(const char __far *nptr);

float __far atoff(const char __near *nptr);
float __far _COM_atoff_f(const char __far *nptr);

int __far atoi(const char __near *nptr);
int __far _COM_atoi_f(const char __far *nptr);

long int __far atol(const char __near *nptr);
long int __far _COM_atol_f(const char __far *nptr);

double __far strtod(const char __near *nptr, char __near * __near *endptr);
double __far _COM_strtod_ff(const char __far *nptr, char __far * __far *endptr);

float __far strtof(const char __near *nptr, char __near * __near *endptr);
float __far _COM_strtof_ff(const char __far *nptr, char __far * __far *endptr);

long int __far strtol(const char __near *nptr, char __near * __near *endptr, int base);
long int __far _COM_strtol_ff(const char __far *nptr, char __far * __far *endptr, int base);

unsigned long int __far strtoul(const char __near *nptr, char __near * __near *endptr, int base);
unsigned long int __far _COM_strtoul_ff(const char __far *nptr, char __far * __far *endptr, int base);

int __far rand(void);
void __far srand(unsigned int seed);

void __near * __far calloc(size_t nmemb, size_t size);
void __far free(void __near *ptr);
void __near * __far malloc(size_t size);
void __near * __far realloc(void __near *ptr, size_t size);

void __far abort(void);

void __near * __far bsearch(const void __near *key, const void __near *base, size_t nmemb, size_t size, int (__far *compar)(const void __near *, const void __near *));
void __far * __far _COM_bsearch_f(const void __far *key, const void __far *base, size_t nmemb, size_t size, int (__far *compar)(const void __far *, const void __far *));

void __far qsort(void __near *base, size_t nmemb, size_t size, int (__far *compar)(const void __near *, const void __near *));
void __far _COM_qsort_f(void __far *base, size_t nmemb, size_t size, int (__far *compar)(const void __far *, const void __far *));

int __far abs(int j);
div_t __far div(int numer, int denom);
long int __far labs(long int j);
ldiv_t __far ldiv(long int numer, long int denom);

#if defined(__CNV_IAR__)
#define strtold(nptr, endptr)           strtod(nptr, endptr)
#endif  /* __CNV_IAR__ */

#if defined(__FAR_ROM__)
#if defined(__DBL4)
#define atof(nptr)                      _COM_atoff_f(nptr)
#define _COM_atof_f(nptr)               _COM_atoff_f(nptr)
#define strtod(nptr, endptr)            _COM_strtof_ff(nptr, endptr)
#define _COM_strtod_ff(nptr, endptr)    _COM_strtof_ff(nptr, endptr)
#else   /* __DBL4 */
#define atof(nptr)                      _COM_atof_f(nptr)
#define strtod(nptr, endptr)            _COM_strtod_ff(nptr, endptr)
#endif  /* __DBL4 */
#define atoff(nptr)                     _COM_atoff_f(nptr)
#define atoi(nptr)                      _COM_atoi_f(nptr)
#define atol(nptr)                      _COM_atol_f(nptr)
#define strtof(nptr, endptr)            _COM_strtof_ff(nptr, endptr)
#define strtol(nptr, endptr, base)      _COM_strtol_ff(nptr, endptr, base)
#define strtoul(nptr, endptr, base)     _COM_strtoul_ff(nptr, endptr, base)
#define bsearch(key, base, nmemb, size, compar) _COM_bsearch_f(key, base, nmemb, size, compar)
#define qsort(base, nmemb, size, compar)        _COM_qsort_f(base, nmemb, size, compar)
#else   /* __FAR_ROM__ */
#if defined(__DBL4)
#define atof(nptr)                      atoff(nptr)
#define _COM_atof_f(nptr)               _COM_atoff_f(nptr)
#define strtod(nptr, endptr)            strtof(nptr, endptr)
#define _COM_strtod_ff(nptr, endptr)    _COM_strtof_ff(nptr, endptr)
#endif  /* __DBL4 */
#endif  /* __FAR_ROM__ */

#endif  /* !_STDLIB_H */
