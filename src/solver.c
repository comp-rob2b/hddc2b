// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/platform.h>
#include <config.h>

#include <assert.h>
#include <string.h>
#include <cblas.h>


static const int NUM_DRV_COORD  = 2;
static const int NUM_PLTF_COORD = 3;



void hddc2b_pltf_frc_redu_ref_init2(
        int num_drv,
        const double *g,
        const double *f_drv_ref,
        double *f_pltf_out)
{
    assert(num_drv >= 0);
    assert(g);
    assert(f_drv_ref);
    assert(f_pltf_out);

    const int LDG = NUM_PLTF_COORD;
    const int INC = 1;

    // F_p'[3x1] = G[3xNC] F_d'[NCx1]
    cblas_dgemv(CblasColMajor, CblasNoTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD,
            1.0, g, LDG,
            f_drv_ref, INC,
            0.0, f_pltf_out, INC);
}


void hddc2b_pltf_frc_redu_wgh_fini_accu(
        int num_drv,
        const double *f_drv_in,
        const double *w_drv_inv_sqrt,
        double *f_drv_out)
{
    assert(num_drv >= 0);
    assert(f_drv_in);
    assert(w_drv_inv_sqrt);
    assert(f_drv_out);

    const int LDWO = NUM_DRV_COORD * NUM_DRV_COORD; // outer array
    const int LDWI = NUM_DRV_COORD;                 // inner array
    const int LDF  = 2;
    const int INC  = 1;

    for (int i = 0; i < num_drv; i++) {
        // F_d'[2x1] += W_d^{-1/2}[2x2] F_d[2x1]
        cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_DRV_COORD, NUM_DRV_COORD,
                -1.0, &w_drv_inv_sqrt[i * LDWO], LDWI,
                &f_drv_in[i * LDF], INC,
                1.0, &f_drv_out[i * LDF], INC);
    }
}


void hddc2b_pltf_frc_pltf_to_drv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv_prim,
        double *f_drv_scnd)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;

    // W_p^{0.5} = sqrt(W_p)
    double w_pltf_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    hddc2b_pltf_frc_w_pltf_sqrt(w_pltf, w_pltf_sqrt);

    // W_d^{-0.5} = sqrt(W_d)^-1
    double w_drv_inv_sqrt[num_drv * NUM_DRV_COORD * NUM_DRV_COORD];
    hddc2b_pltf_frc_w_drv_inv_sqrt(num_drv, w_drv, w_drv_inv_sqrt);

    // G'   = W_p^{0.5} G
    // F_p' = W_p^{0.5} F_p
    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf_sqrt, g2, f_pltf2);

    // F_d' = G' F_d
    double f_drv2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_redu_ref_init2(num_drv, g2, f_drv_ref, f_drv2);

    // G'' = G' W_d^{-0.5}
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g2, w_drv_inv_sqrt, g3);

    // U, S, V^T = svd(G'')
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    // S+ = S^{-1}
    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);


    // F_prim' = (V S+ U^T) F_p'
    double f_drv_prim2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf2, f_drv_prim2);

    // F_prim = W_d^{-0.5} F_prim'
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv_prim2, w_drv_inv_sqrt, f_drv_prim);


    // F_scnd' = (V S+ U^T) F_d'
    double f_drv_scnd2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_drv2, f_drv_scnd2);

    // F_scnd = F_d - W_d^{-0.5} F_scnd'
    memcpy(f_drv_scnd, f_drv_ref, num_drv * NUM_DRV_COORD * sizeof(double));
    hddc2b_pltf_frc_redu_wgh_fini_accu(num_drv, f_drv_scnd2, w_drv_inv_sqrt, f_drv_scnd);
}
