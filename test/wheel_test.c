// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/wheel.h>
#include "common.h"

#include <math.h>

#define NUM_DRV       4
#define NUM_GND_COORD 2
#define NUM_WHL_COORD 2

static double wheel_diameter[NUM_DRV * NUM_WHL_COORD] = {
    0.115, 0.115,   // fl-l, fl-r
    0.115, 0.115,   // rl-l, rl-r
    0.115, 0.115,   // rr-l, rr-r
    0.115, 0.115    // fr-l, fr-r
};


START_TEST(test_hddc2b_whl_frc_hub_to_gnd)
{
    double tau_hub[NUM_DRV * NUM_WHL_COORD] = {
        -0.007188,  0.007188,   // fl-r, fl-l
         0.007188, -0.007188,   // rl-r, rl-l
         0.007188,  0.007188,   // rr-r, rr-l
        -0.013964,  0.024900    // fr-r, fr-l
    };
    double f_whl[NUM_DRV * NUM_GND_COORD];
    double res[NUM_DRV * NUM_GND_COORD] = {
        -0.125,     0.125,      // fl-r, fl-l
         0.125,    -0.125,      // rl-r, rl-l
         0.125,     0.125,      // rr-r, rr-l
        -0.242863,  0.433052    // fr-r, fr-l
    };

    hddc2b_whl_frc_hub_to_gnd(
            NUM_DRV,
            wheel_diameter,
            tau_hub,
            f_whl);

    for (int i = 0; i < NUM_DRV * NUM_GND_COORD; i++) {
        ck_assert_dbl_eq(f_whl[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_whl_frc_gnd_to_hub)
{
    double f_whl[NUM_DRV * NUM_GND_COORD] = {
        -0.125,     0.125,      // fl-r, fl-l
         0.125,    -0.125,      // rl-r, rl-l
         0.125,     0.125,      // rr-r, rr-l
        -0.242863,  0.433052    // fr-r, fr-l
    };
    double tau_hub[NUM_DRV * NUM_WHL_COORD];
    double res[NUM_DRV * NUM_WHL_COORD] = {
        -0.007188,  0.007188,   // fl-r, fl-l
         0.007188, -0.007188,   // rl-r, rl-l
         0.007188,  0.007188,   // rr-r, rr-l
        -0.013964,  0.024900    // fr-r, fr-l
    };

    hddc2b_whl_frc_gnd_to_hub(
            NUM_DRV,
            wheel_diameter,
            f_whl,
            tau_hub);

    for (int i = 0; i < NUM_DRV * NUM_WHL_COORD; i++) {
        ck_assert_dbl_eq(tau_hub[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_whl_vel_hub_to_gnd)
{
    double omega_hub[NUM_DRV * NUM_GND_COORD] = {
        -0.125,  0.125,         // fl-r, fl-l
         0.125, -0.125,         // rl-r, rl-l
         0.125,  0.125,         // rr-r, rr-l
        -0.125, -0.125          // fr-r, fr-l
    };
    double xd_whl[NUM_DRV * NUM_WHL_COORD];
    double res[NUM_DRV * NUM_WHL_COORD] = {
        -0.007188,  0.007188,   // fl-r, fl-l
         0.007188, -0.007188,   // rl-r, rl-l
         0.007188,  0.007188,   // rr-r, rr-l
        -0.007188, -0.007188    // fr-r, fr-l
    };

    hddc2b_whl_vel_hub_to_gnd(
            NUM_DRV,
            wheel_diameter,
            omega_hub,
            xd_whl);

    for (int i = 0; i < NUM_DRV * NUM_WHL_COORD; i++) {
        ck_assert_dbl_eq(xd_whl[i], res[i]);
    }
}
END_TEST


START_TEST(test_power_must_be_equal_in_both_spaces)
{
    double omega_hub[NUM_DRV * NUM_WHL_COORD] = {
         1.45187253, 25.25703391,   // fl-r, fl-l
        94.49104517,  5.58535653,   // fl-r, fl-l
        59.33541863, 35.61239572,   // rr-r, rr-l
        26.92297075,  6.2835019     // fr-r, fr-l
    };
    double f_whl[NUM_DRV * NUM_GND_COORD] = {
        70.95956555, 36.06250027,   // fl-x, fl-y
        77.00068869, 82.98443704,   // rl-x, rl-y
        87.41332935, 50.29720317,   // rr-x, rr-y
        23.6478908 , 75.43688388    // fr-x, fr-y
    };
    double xd_whl[NUM_DRV * NUM_GND_COORD];
    double tau_hub[NUM_DRV * NUM_WHL_COORD];
    double p_whl[NUM_DRV];
    double p_hub[NUM_DRV];

    hddc2b_whl_vel_hub_to_gnd(
            NUM_DRV,
            wheel_diameter,
            omega_hub,
            xd_whl);

    hddc2b_whl_frc_gnd_to_hub(
            NUM_DRV,
            wheel_diameter,
            f_whl,
            tau_hub);

    for (int i = 0; i < NUM_DRV; i++) {
        p_whl[i] = 0.0;
        p_hub[i] = 0.0;
        for (int j = 0; j < NUM_WHL_COORD; j++) {
            p_whl[i] += xd_whl[i * NUM_WHL_COORD + j]
                        * f_whl[i * NUM_WHL_COORD + j];
        }
        for (int j = 0; j < NUM_GND_COORD; j++) {
            p_hub[i] += omega_hub[i * NUM_GND_COORD + j]
                        * tau_hub[i * NUM_GND_COORD + j];
        }
        ck_assert_dbl_eq(p_whl[i], p_hub[i]);
    }
}
END_TEST


TCase *hddc2b_wheel_test(void)
{
    TCase *tc = tcase_create("wheel");

    tcase_add_test(tc, test_hddc2b_whl_frc_hub_to_gnd);
    tcase_add_test(tc, test_hddc2b_whl_frc_gnd_to_hub);
    tcase_add_test(tc, test_hddc2b_whl_vel_hub_to_gnd);
    tcase_add_test(tc, test_power_must_be_equal_in_both_spaces);

    return tc;
}
