// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_FUNCTIONS_SOLVER_H
#define HDDC2B_FUNCTIONS_SOLVER_H


#ifdef __cplusplus
extern "C" {
#endif

void hddc2b_pltf_frc_pltf_to_drv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv_prim,
        double *f_drv_scnd);


#ifdef __cplusplus
}
#endif

#endif
