// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_TEST_COMMON_H
#define HDDC2B_TEST_COMMON_H

#include <check.h>

#ifdef ck_assert_double_eq_tol
#  define ck_assert_dbl_eq(X, Y) ck_assert_double_eq_tol(X, Y, 0.0001)
#else
#include <math.h>
#  define ck_assert_dbl_eq(X, Y) do { \
     double _dist = fabs((double)(X) - (double)(Y)); \
     ck_assert_msg(_dist < (0.0001), "Assertion '%s' failed: %s == %f, %s == %f", #X" == "#Y, #X, (X), #Y, (Y)); \
   } while (0)
#endif

#endif
