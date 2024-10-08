// SPDX-License-Identifier: LGPL-3.0
#include "solver.h"     // Automatically generated at test's build time
#include <hddc2b/functions/platform.h>
#include "common.h"

#include <math.h>


#define NUM_DRV_REDU       4
#define NUM_DRV_SING       1
#define NUM_DRV_COORD      2
#define NUM_PLTF_COORD     3
#define NUM_G_COORD        ((NUM_DRV_COORD) * (NUM_PLTF_COORD))
#define EPS                0.001


START_TEST(test_ex_frc_pltf_to_pvt_sing)
{
    double pos_drv[NUM_DRV_SING * NUM_DRV_COORD] = {
         0.1, 0.1
    };
    double g[NUM_DRV_SING * NUM_G_COORD];
    double q_pvt[NUM_DRV_SING] = { 0.0 };
    double w_pltf[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    double f_pltf[NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0
    };
    double f_drv[NUM_DRV_SING * NUM_DRV_COORD];
    double res[NUM_DRV_SING * NUM_DRV_COORD] = {
        0.9902, 0.0098
    };

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV_SING,
            pos_drv,
            q_pvt,
            g);
   ex_frc_pltf_to_pvt_sing_dls(
            NUM_DRV_SING,
            EPS,
            1.0,
            g,
            w_pltf,
            f_pltf,
            f_drv);

    for (int i = 0; i < NUM_DRV_SING * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_ex_vel_pvt_to_pltf_sing_dls)
{
    double pos_drv[NUM_DRV_SING * NUM_DRV_COORD] = {
         0.1, 0.1
    };
    double g[NUM_DRV_SING * NUM_G_COORD];
    double q_pvt[NUM_DRV_SING] = { 0.0 };
    double w_pltf_inv_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    double xd_drv[NUM_DRV_SING * NUM_DRV_COORD] = {
        1.0, 0.0
    };
    double xd_pltf[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        0.99019608, 0.00980392, -0.09803922
    };

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV_SING,
            pos_drv,
            q_pvt,
            g);
   ex_vel_pvt_to_pltf_sing_dls(
            NUM_DRV_SING,
            EPS,
            1.0,
            g,
            xd_drv,
            w_pltf_inv_sqrt,
            xd_pltf);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(xd_pltf[i], res[i]);
    }
}
END_TEST


START_TEST(test_reference_value_in_nullspace)
{
    double pos_drv[NUM_DRV_REDU * NUM_DRV_COORD] = {
         0.1,  0.1,
        -0.1,  0.1,
        -0.1, -0.1,
         0.1, -0.1
    };
    double g[NUM_DRV_REDU * NUM_G_COORD];
    double q_pvt[NUM_DRV_REDU] = {
        M_PI_4, -M_PI_4, M_PI_4, -M_PI_4
    };
    double w_drv[NUM_DRV_REDU * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0
    };
    double f_pltf[NUM_PLTF_COORD] = {
        0.0, 0.0, 0.0
    };
    double f_ref[NUM_DRV_REDU * NUM_DRV_COORD] = {
         1.0, 0.0,
         1.0, 0.0,
        -1.0, 0.0,
        -1.0, 0.0
    };
    double f_drv[NUM_DRV_REDU * NUM_DRV_COORD] = {
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
    };
    double res[NUM_DRV_REDU * NUM_DRV_COORD] = {
         1.0, 0.0,
         1.0, 0.0,
        -1.0, 0.0,
        -1.0, 0.0
    };

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV_REDU,
            pos_drv,
            q_pvt,
            g);
    ex_frc_pltf_to_pvt_redu_ref_pinv(
            NUM_DRV_REDU,
            EPS,
            g,
            f_pltf,
            w_drv,
            f_ref,
            f_drv);

    for (int i = 0; i < NUM_DRV_REDU * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_reference_value_not_in_nullspace)
{
    double pos_drv[NUM_DRV_REDU * NUM_DRV_COORD] = {
         0.1,  0.1,
        -0.1,  0.1,
        -0.1, -0.1,
         0.1, -0.1
    };
    double g[NUM_DRV_REDU * NUM_G_COORD];
    double q_pvt[NUM_DRV_REDU] = {
        3.0 * M_PI_2 + M_PI_4,
        0.0 * M_PI_2 + M_PI_4,
        1.0 * M_PI_2 + M_PI_4,
        2.0 * M_PI_2 + M_PI_4
    };
    double w_drv[NUM_DRV_REDU * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0
    };
    double f_pltf[NUM_PLTF_COORD] = {
        0.0, 0.0, 0.0
    };
    double f_ref[NUM_DRV_REDU * NUM_DRV_COORD] = {
        1.0, 0.0,
        1.0, 0.0,
        1.0, 0.0,
        1.0, 0.0
    };
    double f_drv[NUM_DRV_REDU * NUM_DRV_COORD];
    double res[NUM_DRV_REDU * NUM_DRV_COORD] = {
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
    };

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV_REDU,
            pos_drv,
            q_pvt,
            g);
    ex_frc_pltf_to_pvt_redu_ref_pinv(
            NUM_DRV_REDU,
            EPS,
            g,
            f_pltf,
            w_drv,
            f_ref,
            f_drv);

    for (int i = 0; i < NUM_DRV_REDU * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_badly_conditioned_w_pltf)
{
    double pos_drv[NUM_DRV_REDU * NUM_DRV_COORD] = {
         0.175,  0.1605,
         0.175, -0.1605,
        -0.175, -0.1605,
        -0.175,  0.1605
    };
    double g[NUM_DRV_REDU * NUM_G_COORD];
    double q_pvt[NUM_DRV_REDU] = {
        0.0, 0.0, 0.0, 0.0
    };
    double w_pltf[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 0.01
    };
    double w_drv[NUM_DRV_REDU * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0
    };
    double f_pltf_in[NUM_PLTF_COORD] = {
        -1.0, 1.0, 1.0
    };
    double f_drv[NUM_DRV_REDU * NUM_DRV_COORD];
    double f_pltf_out[NUM_PLTF_COORD];

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV_REDU,
            pos_drv,
            q_pvt,
            g);
    ex_frc_pltf_to_pvt_redu_sing_pinv(
            NUM_DRV_REDU,
            EPS,
            g,
            w_pltf,
            f_pltf_in,
            w_drv,
            f_drv);
    hddc2b_pltf_frc_pvt_to_pltf(
            NUM_DRV_REDU,
            g,
            f_drv,
            f_pltf_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(f_pltf_out[i], f_pltf_in[i]);
    }
}
END_TEST


TCase *hddc2b_solver_test(void)
{
    TCase *tc = tcase_create("solver");

    tcase_add_test(tc, test_ex_frc_pltf_to_pvt_sing);
    tcase_add_test(tc, test_ex_vel_pvt_to_pltf_sing_dls);
    tcase_add_test(tc, test_reference_value_in_nullspace);
    tcase_add_test(tc, test_reference_value_not_in_nullspace);
    tcase_add_test(tc, test_badly_conditioned_w_pltf);

    return tc;
}
