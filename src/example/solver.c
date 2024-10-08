// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/platform.h>
#include <solver.h>
#include <assert.h>


void hddc2b_example_frc(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;

    double w_pltf_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    hddc2b_pltf_frc_w_pltf_sqrt(w_pltf, w_pltf_sqrt);
    
    double w_drv_inv_sqrt[num_drv * NUM_DRV_COORD * NUM_DRV_COORD];
    hddc2b_pltf_frc_w_drv_inv_sqrt(num_drv, w_drv, w_drv_inv_sqrt);
    
    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf, g2, f_pltf2);
    
    double f_pltf3[NUM_PLTF_COORD];
    hddc2b_pltf_frc_redu_ref_init(num_drv, g2, f_pltf2, f_drv_ref, f_pltf3);
    
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g2, w_drv_inv_sqrt, g3);
    
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    
    double f_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf3, f_drv2);
    
    double f_drv3[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv2, w_drv_inv_sqrt, f_drv3);
        
    hddc2b_pltf_frc_redu_ref_fini(num_drv, f_drv_ref, f_drv3, f_drv);
}


void hddc2b_example_vel(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;

    
    double g2[num_drv * NUM_G_COORD];
    double xd_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_vel_sing_wgh(num_drv, g, xd_drv, w_drv_sqrt, g2, xd_drv2);
    
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_vel_redu_wgh_init(num_drv, g2, w_pltf_inv_sqrt, g3);
    
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    
    double xd_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_vel_slv(num_drv, u, s_inv, vt, xd_drv2, xd_pltf2);
    
    hddc2b_pltf_vel_redu_wgh_fini(num_drv, xd_pltf2, w_pltf_inv_sqrt, xd_pltf);
}
