// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/platform.h>
#include "common.h"

#include <math.h>

#define NUM_DRV        4
#define NUM_DRV_COORD  2
#define NUM_PLTF_COORD 3
#define NUM_G_COORD    ((NUM_DRV_COORD) * (NUM_PLTF_COORD))

static double pos_drv[NUM_DRV * NUM_DRV_COORD] = {
     0.175,  0.1605,            // fl-x, fl-y
    -0.175,  0.1605,            // rl-x, rl-y
    -0.175, -0.1605,            // rr-x, rr-y
     0.175, -0.1605             // fr-x, fr-y
};


START_TEST(test_hddc2b_pltf_drv_algn_dst)
{
    double q_pvt[NUM_DRV] = {
        0.0, 0.0, 0.0, M_PI_2   // fl, rl, rr, fr
    };
    double f_pltf[NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0           // fx, fy, mz
    };
    double w[NUM_DRV_COORD * NUM_DRV] = {
        1.0, 1.0,               // fl-wa, fl-wl
        1.0, 1.0,               // rl-wa, rl-wl
        1.0, 1.0,               // rr-wa, rr-wl
        1.0, 1.0                // fr-wa, fr-wl
    };
    double dst[NUM_DRV * NUM_DRV_COORD] = {
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
    };
    double res[NUM_DRV * NUM_DRV_COORD] = {
        0.0,  0.0,              // dummy, fl
        0.0,  0.0,              // dummy, rl
        0.0,  0.0,              // dummy, rr
        0.0, -1.5708            // dummy, fr
    };

    hddc2b_pltf_drv_algn_dst(
            NUM_DRV,
            pos_drv,
            w,
            q_pvt,
            f_pltf,
            &dst[1],
            2);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(dst[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_comp_mat)
{
    double q_pvt[NUM_DRV] = {
        0.0, 0.0, 0.0, 0.0      // fl, rl, rr, fr
    };
    double g[NUM_DRV * NUM_G_COORD];
    double res[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -0.1605,
        0.0, 1.0,  0.175,
        1.0, 0.0, -0.1605,
        0.0, 1.0, -0.175,
        1.0, 0.0,  0.1605,
        0.0, 1.0, -0.175,
        1.0, 0.0,  0.1605,
        0.0, 1.0,  0.175
    };

    hddc2b_pltf_frc_comp_mat(
            NUM_DRV,
            pos_drv,
            q_pvt,
            g);

    for (int i = 0; i < NUM_DRV * NUM_G_COORD; i++) {
        ck_assert_dbl_eq(g[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_pvt_to_pltf)
{
    double g[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double f_drv[NUM_DRV * NUM_DRV_COORD] = {
        0.25, 0.0,                  // fl-x, fl-y
        0.25, 0.0,                  // rl-x, rl-y
        0.25, 0.0,                  // rr-x, rr-y
        0.25, 0.0                   // fr-x, fr-y
    };
    double f_pltf[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0
    };

    hddc2b_pltf_frc_pvt_to_pltf(
            NUM_DRV,
            g,
            f_drv,
            f_pltf);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(f_pltf[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_w_pltf_sqrt)
{
    double w_pltf[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0
    };
    double w_pltf_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        0.89678427, 0.64641982, 0.58149787,
        0.64641982, 1.47828214, 1.22791769,
        0.58149787, 1.22791769, 2.12470196
    };

    hddc2b_pltf_frc_w_pltf_sqrt(
            w_pltf,
            w_pltf_sqrt);

    for (int i = 0; i < NUM_PLTF_COORD * NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(w_pltf_sqrt[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_w_drv_inv_sqrt)
{
    double w_drv[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        0.00001, 0.0,
        0.0,     0.00001,

        0.0, 0.0,
        0.0, 0.0
    };
    double w_drv_inv_sqrt[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
           1.623597, -  0.703155,
        -  0.703155,    0.920442,

           1.623597, -  0.703155,
        -  0.703155,    0.920442,

         316.227766,    0.0,
           0.0     ,  316.227766,

           0.0,         0.0,
           0.0,         0.0
    };

    hddc2b_pltf_frc_w_drv_inv_sqrt(
            NUM_DRV,
            w_drv,
            w_drv_inv_sqrt);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(w_drv_inv_sqrt[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_sing_wgh)
{
    double g_in[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double f_pltf_in[NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0
    };
    double w_pltf_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0
    };
    double g_out[NUM_DRV * NUM_G_COORD];
    double f_pltf_out[NUM_PLTF_COORD];
    double g_res[NUM_DRV * NUM_G_COORD] = {
        -2.0, -3.0, - 3.0,
         5.0,  9.0,  11.0,
        -2.0, -3.0, - 3.0,
        -1.0, -1.0, - 1.0,
         4.0,  7.0,   9.0,
        -1.0, -1.0, - 1.0,
         4.0,  7.0,   9.0,
         5.0,  9.0,  11.0
    };
    double f_pltf_res[NUM_PLTF_COORD] = {
        14.0, 25.0, 31.0
    };

    hddc2b_pltf_frc_sing_wgh(
            NUM_DRV,
            g_in,
            f_pltf_in,
            w_pltf_sqrt,
            g_out,
            f_pltf_out);

    for (int i = 0; i < NUM_DRV * NUM_G_COORD; i++) {
        ck_assert_dbl_eq(g_out[i], g_res[i]);
    }

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(f_pltf_out[i], f_pltf_res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_redu_ref_init)
{
    double g[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double f_pltf_in[NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0
    };
    double f_drv_ref[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 2.0,
        3.0, 4.0,
        5.0, 6.0,
        7.0, 8.0
    };
    double f_pltf_out[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        -15.0, -18.0, -5.0
    };

    hddc2b_pltf_frc_redu_ref_init(
            NUM_DRV,
            g,
            f_pltf_in,
            f_drv_ref,
            f_pltf_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(f_pltf_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_redu_wgh_init)
{
    double g_in[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double w_drv_inv_sqrt[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0
    };
    double g_out[NUM_DRV * NUM_G_COORD];
    double res[NUM_DRV * NUM_G_COORD] = {
        1.0,  2.0,  1.0,
        2.0,  3.0,  1.0,
        1.0,  2.0, -3.0,
        2.0,  3.0, -5.0,
        1.0,  2.0, -1.0,
        2.0,  3.0, -1.0,
        1.0,  2.0,  3.0,
        2.0,  3.0,  5.0,
    };

    hddc2b_pltf_frc_redu_wgh_init(
            NUM_DRV,
            g_in,
            w_drv_inv_sqrt,
            g_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(g_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_redu_wgh_fini)
{
    double f_drv_in[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 2.0,
        3.0, 4.0,
        5.0, 6.0,
        7.0, 8.0
    };
    double w_drv_inv_sqrt[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 0.0,
        0.0, 1.0,

        1.0, 1.0,
        1.0, 1.0,

        2.0, 0.0,
        0.0, 3.0,

        0.0, 0.5,
        0.1, 0.0
    };
    double f_drv_out[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  2.0,
         7.0,  7.0,
        10.0, 18.0,
         0.8,  3.5
    };

    hddc2b_pltf_frc_redu_wgh_fini(
            NUM_DRV,
            f_drv_in,
            w_drv_inv_sqrt,
            f_drv_out);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_redu_ref_fini)
{
    double in1[NUM_DRV * NUM_DRV_COORD] = {
        1.1, 1.2,
        2.2, 2.3,
        3.3, 3.4,
        4.4, 4.5
    };
    double in2[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 0.2,
        2.0, 0.3,
        3.0, 0.4,
        4.0, 0.5
    };
    double out[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
        2.1, 1.4,
        4.2, 2.6,
        6.3, 3.8,
        8.4, 5.0
    };

    hddc2b_pltf_frc_redu_ref_fini(
            NUM_DRV,
            in1,
            in2,
            out);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_frc_slv)
{
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    double s_inv[NUM_PLTF_COORD] = {
        0.3536, 0.5, 0.5
    };
    double vt[NUM_DRV * NUM_G_COORD] = {
        -0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5,
        -0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5
    };
    double f_pltf[NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0
    };
    double f_drv[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0
    };

    hddc2b_pltf_frc_slv(
            NUM_DRV,
            u,
            s_inv,
            vt,
            f_pltf,
            f_drv);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_w_pltf_inv_sqrt)
{
    double w_pltf[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0
    };
    double w_pltf_inv_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
         1.63312709, -0.65941795, -0.06586757,
        -0.65941795,  1.56725952, -0.72528552,
        -0.06586757, -0.72528552,  0.90784157
    };

    hddc2b_pltf_vel_w_pltf_inv_sqrt(
            w_pltf,
            w_pltf_inv_sqrt);

    for (int i = 0; i < NUM_PLTF_COORD * NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(w_pltf_inv_sqrt[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_w_drv_sqrt)
{
    double w_drv[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        0.00001, 0.0,
        0.0,     0.00001,

        0.0, 0.0,
        0.0, 0.0
    };
    double w_drv_sqrt[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        0.92044207, 0.70315517,
        0.70315517, 1.62359723,

        0.92044207, 0.70315517,
        0.70315517, 1.62359723,

        0.00316228, 0.0,
        0.0,        0.00316228,

        0.0, 0.0,
        0.0, 0.0
    };

    hddc2b_pltf_vel_w_drv_sqrt(
            NUM_DRV,
            w_drv,
            w_drv_sqrt);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(w_drv_sqrt[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_sing_wgh)
{
    double g_in[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double xd_drv_in[NUM_DRV * NUM_DRV_COORD] = {
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0
    };
    double w_drv_sqrt[NUM_DRV * NUM_DRV_COORD * NUM_DRV_COORD] = {
        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0,

        1.0, 2.0,
        2.0, 3.0
    };
    double g_out[NUM_DRV * NUM_G_COORD];
    double xd_drv_out[NUM_DRV * NUM_DRV_COORD];
    double g_res[NUM_DRV * NUM_G_COORD] = {
        1.0, 2.0,  1.0,
        2.0, 3.0,  1.0,
        1.0, 2.0, -3.0,
        2.0, 3.0, -5.0,
        1.0, 2.0, -1.0,
        2.0, 3.0, -1.0,
        1.0, 2.0,  3.0,
        2.0, 3.0,  5.0
    };
    double xd_drv_res[NUM_DRV * NUM_DRV_COORD] = {
        0.25, 0.5,
        0.25, 0.5,
        0.25, 0.5,
        0.25, 0.5
    };

    hddc2b_pltf_vel_sing_wgh(
            NUM_DRV,
            g_in,
            xd_drv_in,
            w_drv_sqrt,
            g_out,
            xd_drv_out);

    for (int i = 0; i < NUM_DRV * NUM_G_COORD; i++) {
        ck_assert_dbl_eq(g_out[i], g_res[i]);
    }

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(xd_drv_out[i], xd_drv_res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_redu_ref_init)
{
    double g[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double xd_drv_in[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 2.0,
        3.0, 4.0,
        5.0, 6.0,
        7.0, 8.0
    };
    double xd_pltf_ref[NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0
    };
    double xd_drv_out[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
        3.0, -3.0,
        5.0,  5.0,
        1.0,  7.0,
        3.0,  3.0
    };

    hddc2b_pltf_vel_redu_ref_init(
            NUM_DRV,
            g,
            xd_drv_in,
            xd_pltf_ref,
            xd_drv_out);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(xd_drv_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_redu_wgh_init)
{
    double g_in[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double w_pltf_inv_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0
    };
    double g_out[NUM_DRV * NUM_G_COORD];
    double res[NUM_DRV * NUM_G_COORD] = {
        -2.0, -3.0, - 3.0,
         5.0,  9.0,  11.0,
        -2.0, -3.0, - 3.0,
        -1.0, -1.0, - 1.0,
        -2.0, -3.0, - 3.0,
         5.0,  9.0,  11.0,
         4.0,  7.0,   9.0,
         5.0,  9.0,  11.0
    };

    hddc2b_pltf_vel_redu_wgh_init(
            NUM_DRV,
            g_in,
            w_pltf_inv_sqrt,
            g_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(g_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_redu_wgh_fini)
{
    double xd_pltf_in[NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0
    };
    double w_pltf_inv_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0
    };
    double xd_pltf_out[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
         14.0, 25.0, 31.0
    };

    hddc2b_pltf_vel_redu_wgh_fini(
            NUM_DRV,
            xd_pltf_in,
            w_pltf_inv_sqrt,
            xd_pltf_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(xd_pltf_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_redu_ref_fini)
{
    double in1[NUM_PLTF_COORD] = {
        1.1, 1.2, 2.2
    };
    double in2[NUM_PLTF_COORD] = {
        1.0, 0.2, 2.0
    };
    double out[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        2.1, 1.4, 4.2
    };

    hddc2b_pltf_vel_redu_ref_fini(
            in1,
            in2,
            out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_vel_slv)
{
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    double s_inv[NUM_PLTF_COORD] = {
        0.3536, 0.5, 0.5
    };
    double vt[NUM_DRV * NUM_G_COORD] = {
        -0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5,
        -0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5
    };
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0,
        0.25, 0.0
    };
    double xd_pltf[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        0.25, 0.0, 0.0
    };

    hddc2b_pltf_vel_slv(
            NUM_DRV,
            u,
            s_inv,
            vt,
            xd_drv,
            xd_pltf);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(xd_pltf[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_dcmp)
{
    double g[NUM_DRV * NUM_G_COORD] = {
        1.0, 0.0, -1.0,
        0.0, 1.0,  1.0,
        1.0, 0.0, -1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0, -1.0,
        1.0, 0.0,  1.0,
        0.0, 1.0,  1.0
    };
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[NUM_DRV * NUM_G_COORD];
    double res_u[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0
    };
    double res_s[NUM_PLTF_COORD] = {
        2.8284, 2.0, 2.0
    };
    double res_vt[NUM_DRV * NUM_G_COORD] = {
        -0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5,
        -0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
        -0.3536, 0.0, 0.5,
         0.3536, 0.5, 0.0,
         0.3536, 0.0, 0.5
    };

    hddc2b_pltf_dcmp(
            NUM_DRV,
            g,
            u,
            s,
            vt);

    for (int i = 0; i < NUM_PLTF_COORD * NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(u[i], res_u[i]);
    }

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(s[i], res_s[i]);
    }

    for (int i = 0; i < NUM_DRV * NUM_G_COORD; i++) {
        ck_assert_dbl_eq(vt[i], res_vt[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_pinv)
{
    const double EPS = 0.00001;
    double s[NUM_PLTF_COORD] = {
        2.0, 1.0, 0.1 * EPS
    };
    double s_inv[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        0.5, 1.0, 0.0
    };

    hddc2b_pltf_pinv(
            NUM_DRV,
            EPS,
            s,
            s_inv);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(s_inv[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_pltf_dmp)
{
    const double EPS = 0.00001;
    const double LAMBDA = 0.1;
    double s[NUM_PLTF_COORD] = {
        2.0, 1.0, 0.1 * EPS
    };
    double s_inv[NUM_PLTF_COORD];
    double res[NUM_PLTF_COORD] = {
        0.4988, 0.9902, 0.0001
    };

    hddc2b_pltf_dmp(
            NUM_DRV,
            EPS,
            LAMBDA,
            s,
            s_inv);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        ck_assert_dbl_eq(s_inv[i], res[i]);
    }
}
END_TEST


TCase *hddc2b_platform_test(void)
{
    TCase *tc = tcase_create("platform");

    tcase_add_test(tc, test_hddc2b_pltf_drv_algn_dst);
    tcase_add_test(tc, test_hddc2b_pltf_frc_comp_mat);

    tcase_add_test(tc, test_hddc2b_pltf_frc_w_pltf_sqrt);
    tcase_add_test(tc, test_hddc2b_pltf_frc_w_drv_inv_sqrt);
    tcase_add_test(tc, test_hddc2b_pltf_frc_sing_wgh);
    tcase_add_test(tc, test_hddc2b_pltf_frc_redu_ref_init);
    tcase_add_test(tc, test_hddc2b_pltf_frc_redu_wgh_init);
    tcase_add_test(tc, test_hddc2b_pltf_frc_redu_wgh_fini);
    tcase_add_test(tc, test_hddc2b_pltf_frc_redu_ref_fini);
    tcase_add_test(tc, test_hddc2b_pltf_frc_slv);
    tcase_add_test(tc, test_hddc2b_pltf_frc_pvt_to_pltf);

    tcase_add_test(tc, test_hddc2b_pltf_vel_w_pltf_inv_sqrt);
    tcase_add_test(tc, test_hddc2b_pltf_vel_w_drv_sqrt);
    tcase_add_test(tc, test_hddc2b_pltf_vel_sing_wgh);
    tcase_add_test(tc, test_hddc2b_pltf_vel_redu_ref_init);
    tcase_add_test(tc, test_hddc2b_pltf_vel_redu_wgh_init);
    tcase_add_test(tc, test_hddc2b_pltf_vel_redu_wgh_fini);
    tcase_add_test(tc, test_hddc2b_pltf_vel_redu_ref_fini);
    tcase_add_test(tc, test_hddc2b_pltf_vel_slv);

    tcase_add_test(tc, test_hddc2b_pltf_dcmp);
    tcase_add_test(tc, test_hddc2b_pltf_pinv);
    tcase_add_test(tc, test_hddc2b_pltf_dmp);

    return tc;
}
