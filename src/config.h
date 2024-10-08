// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_SRC_CONFIG_H
#define HDDC2B_SRC_CONFIG_H


#ifdef __cplusplus
extern "C" {
#endif


// Offsets into any array that features measurements from or commands to a drive
// which, in turn, contains these quantities for the left and right wheel. The
// Kelo drive API uses the definition that the right wheel has identifier "1"
// ("index 0") and the left wheel identifier "2" ("index 1").
extern const int OFFSET_LEFT;
extern const int OFFSET_RIGHT;


#ifdef __cplusplus
}
#endif

#endif
