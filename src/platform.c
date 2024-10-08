// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/platform.h>
#include <config.h>

#include <assert.h>
#include <math.h>
#include <string.h>
#include <cblas.h>
#include <lapacke.h>


#define MIN(a, b) ((a) < (b) ? (a) : (b))

static const int NUM_DRV_COORD  = 2;
static const int NUM_PLTF_COORD = 3;
static const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;

// Tolerance for when to consider the vector length close to zero
static const double EPS = 0.00001;


void hddc2b_pltf_drv_algn_dst(
        int num_drv,
        const double *pos_drv,
        const double *w,
        const double *q_pvt,
        const double *f_pltf,
        double *dst,
        int inc_dst)
{
    assert(num_drv >= 0);
    assert(pos_drv);
    assert(w);
    assert(q_pvt);
    assert(f_pltf);
    assert(dst);
    assert(inc_dst >= 0);

    const int LDP = 2;  // leading dimension of "pos_drv" matrix
    const int LDW = 2;  // leading dimension of "w" matrix

    for (int i = 0; i < num_drv; i++) {
        // Orientation of the platform with respect to the drive (the same
        // "direction" that the pivot encoder measures)
        double c = cos(q_pvt[i]);
        double s = sin(q_pvt[i]);
        double r_piv[4] = {
             c, s,
            -s, c
        };

        // Construct an orientation frame for the angular task:
        // - y-vector: direction from platform's origin to attachment's origin
        // - x-vector: orthogonal to y-vector (treating the y-vector as a
        //             radius, this is a tangent vector to the circle around the
        //             platform's origin)
        double px = pos_drv[i * LDP + 0];
        double py = pos_drv[i * LDP + 1];
        double p_len = sqrt(px * px + py * py);
        double r_ang[4] = {
            -py / p_len, px / p_len,
             px / p_len, py / p_len
        };

        // Compute an angular velocity for the angular task:
        // 1. R_diff = R_piv^T R_ang    | \in SO(2)
        // 2. [w]_x = log(R_diff)       | \in so(2)
        // 3. w = vee([w]_x)            | \in |R^1
        double r_piv_ang[4];
        cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 2, 2, 2,
            1.0, r_piv, 2,
            r_ang, 2,
            0.0, r_piv_ang, 2);
        double w_piv_ang = atan2(r_piv_ang[0 * 2 + 1], r_piv_ang[0 * 2 + 0]);

        // Construct an orientation frame for the linear task:
        // - x-vector: direction of the platform task's linear force
        // - y-vector: orthogonal to x-vector
        double fx = f_pltf[0];
        double fy = f_pltf[1];
        double w_piv_lin = 0.0;
        double f_len = sqrt(fx * fx + fy * fy);

        if (fabs(f_len) > EPS) {
            double r_lin[4] = {
                 fx / f_len, fy / f_len,
                -fy / f_len, fx / f_len
            };

            // Compute an angular velocity for the linear task
            // 1. R_diff = R_piv^T R_lin   | \in SO(2)
            // 2. [w]_x = log(R_diff)      | \in so(2)
            // 3. w = vee([w]_x)           | \in |R^1
            double r_piv_lin[4];
            cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 2, 2, 2,
                1.0, r_piv, 2,
                r_lin, 2,
                0.0, r_piv_lin, 2);
            w_piv_lin = atan2(r_piv_lin[0 * 2 + 1], r_piv_lin[0 * 2 + 0]);
        }

        // 1. Map both angular velocities to torques (via damping)
        // 2. Add both torques
        int idx = i * inc_dst;
        dst[idx] = w[i * LDW + 0] * f_pltf[2] * w_piv_ang
                 + w[i * LDW + 1] *   f_len   * w_piv_lin;
    }
}


void hddc2b_pltf_frc_comp_mat(
        int num_drv,
        const double *pos_drv,
        const double *q_pvt,
        double *g)
{
    assert(num_drv >= 0);
    assert(pos_drv);
    assert(q_pvt);
    assert(g);

    const int LDP = 2;  // leading dimension of "pos_drv" matrix
    const int LDG = 6;  // leading dimension of "g" matrix

    for (int i = 0; i < num_drv; i++) {
        double c = cos(q_pvt[i]);
        double s = sin(q_pvt[i]);
        double xi = pos_drv[i * LDP + 0];
        double yi = pos_drv[i * LDP + 1];

        g[i * LDG + 0] =        c;
        g[i * LDG + 1] =        s;
        g[i * LDG + 2] = xi * s - yi * c;

        g[i * LDG + 3] =       -s;
        g[i * LDG + 4] =        c;
        g[i * LDG + 5] = xi * c + yi * s;
    }
}


void hddc2b_pltf_frc_pvt_to_pltf(
        int num_drv,
        const double *g,
        const double *f_drv,
        double *f_pltf)
{
    assert(num_drv >= 0);
    assert(g);
    assert(f_drv);
    assert(f_pltf);

    const int INC = 1;

    cblas_dgemv(CblasColMajor, CblasNoTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD,
            1.0, g, NUM_PLTF_COORD, f_drv, INC,
            0.0, f_pltf, INC);
}


void hddc2b_pltf_frc_w_pltf_sqrt(
        const double *w_pltf,
        double *w_pltf_sqrt)
{
    assert(w_pltf);
    assert(w_pltf_sqrt);

    const int LDW = NUM_PLTF_COORD;

    //
    // Z_p[3x3], L_p[3x1] = eig(W_p[3x3])
    //
    const char JOB  = 'V';  // compute eigenvalues and eigenvectors
    const char UPLO = 'U';  // use upper triangular matrix
    const int LDZ = NUM_PLTF_COORD;
    double z[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double l[NUM_PLTF_COORD];
    const int LWORK = 15;
    double work[LWORK];

    memcpy(z, w_pltf, NUM_PLTF_COORD * NUM_PLTF_COORD * sizeof(double));
    LAPACKE_dsyev_work(LAPACK_COL_MAJOR, JOB, UPLO, NUM_PLTF_COORD,
            z, LDW, l, work, LWORK);


    //
    // W_p^{1/2}[3x3] = Z_p[3x3] S_p^{1/2}[3x1] Z_p^T[3x3]
    //

    // Z_p[3x3] * diag(sqrt(L_p[3x1]))
    double z_lsqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    for (int row = 0; row < NUM_PLTF_COORD; row++) {
        for (int col = 0; col < NUM_PLTF_COORD; col++) {
            // For an eigenvalue "\lambda" and an eigenvector "v" it holds that
            //   \lambda v = -\lambda -v
            // Hence, the eigendecomposition can produce either positive or
            // negative eigenvalues. Since in either case the magnitude remains
            // the same, we use the absolute eigenvalue here.
            z_lsqrt[row * LDZ + col] = z[row * LDZ + col] * sqrt(fabs(l[row]));
        }
    }

    // * Z_p^T[3x3]
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
            NUM_PLTF_COORD, NUM_PLTF_COORD, NUM_PLTF_COORD,
            1.0, z_lsqrt, LDZ,
            z, LDZ,
            0.0, w_pltf_sqrt, LDW);
}


void hddc2b_pltf_frc_w_drv_inv_sqrt(
        int num_drv,
        const double *w_drv,
        double *w_drv_inv_sqrt)
{
    assert(num_drv >= 0);
    assert(w_drv);
    assert(w_drv_inv_sqrt);

    const int LDWO = NUM_DRV_COORD * NUM_DRV_COORD; // outer array
    const int LDWI = NUM_DRV_COORD;                 // inner array

    for (int i = 0; i < num_drv; i++) {
        //
        // Z_d[2x2], L_d[2x1] = eig(W_d[2x2])
        //
        const char JOB  = 'V';  // compute eigenvalues and eigenvectors
        const char UPLO = 'U';  // use upper triangular matrix
        const int LDZ = NUM_DRV_COORD;
        double z[NUM_DRV_COORD * NUM_DRV_COORD];
        double l[NUM_DRV_COORD];
        const int LWORK = 10;
        double work[LWORK];

        memcpy(z, &w_drv[i * LDWO],
                NUM_DRV_COORD * NUM_DRV_COORD * sizeof(double));
        LAPACKE_dsyev_work(LAPACK_COL_MAJOR, JOB, UPLO, NUM_DRV_COORD,
                z, LDZ, l, work, LWORK);


        //
        // W_d^{-1/2}[2x2] = Z_d[2x2] L_d^{-1/2}[2x1] Z_d^T[2x2]
        //

        // Z_d[2x2] * diag(1 / sqrt(L_d[2x1]))
        double z_linvsqrt[NUM_DRV_COORD * NUM_DRV_COORD];
        for (int row = 0; row < NUM_DRV_COORD; row++) {
            for (int col = 0; col < NUM_DRV_COORD; col++) {
                // For an eigenvalue "\lambda" and an eigenvector "v" it holds
                // that
                //   \lambda v = -\lambda -v
                // Hence, the eigendecomposition can produce either positive or
                // negative eigenvalues. Since in either case the magnitude
                // remains the same, we use the absolute eigenvalue here.
                double l_abs = fabs(l[row]);
                double l_inv = (l_abs < EPS) ? 0.0 : 1.0 / sqrt(l_abs);
                z_linvsqrt[row * LDZ + col] = z[row * LDZ + col] * l_inv;
            }
        }

        // * Z_d^T[2x2]
        cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
                NUM_DRV_COORD, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, z_linvsqrt, LDZ,
                z, LDZ,
                0.0, &w_drv_inv_sqrt[i * LDWO], LDWI);
    }
}


void hddc2b_pltf_frc_sing_wgh(
        int num_drv,
        const double *g_in,
        const double *f_pltf_in,
        const double *w_pltf_sqrt,
        double *g_out,
        double *f_pltf_out)
{
    assert(num_drv >= 0);
    assert(g_in);
    assert(f_pltf_in);
    assert(w_pltf_sqrt);
    assert(g_out);
    assert(f_pltf_out);

    const int LDG = NUM_PLTF_COORD;
    const int INC = 1;

    // G'[3xNC] = W_p^{1/2}[3x3] G[3xNC]
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD, NUM_PLTF_COORD,
            1.0, w_pltf_sqrt, NUM_PLTF_COORD,
            g_in, LDG,
            0.0, g_out, LDG);

    // F_p'[3x1] = W_p^{1/2}[3x3] F_p[3x1]
    cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_PLTF_COORD, NUM_PLTF_COORD,
            1.0, w_pltf_sqrt, NUM_PLTF_COORD,
            f_pltf_in, INC,
            0.0, f_pltf_out, INC);
}


void hddc2b_pltf_frc_redu_ref_init(
        int num_drv,
        const double *g,
        const double *f_pltf_in,
        const double *f_drv_ref,
        double *f_pltf_out)
{
    assert(num_drv >= 0);
    assert(g);
    assert(f_pltf_in);
    assert(f_drv_ref);
    assert(f_pltf_out);

    const int LDG = NUM_PLTF_COORD;
    const int INC = 1;

    // F_p'[3x1] = F_p[3x1] - G[3xNC] F_d'[NCx1]
    memcpy(f_pltf_out, f_pltf_in, NUM_PLTF_COORD * sizeof(double));
    cblas_dgemv(CblasColMajor, CblasNoTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD,
            -1.0, g, LDG,
            f_drv_ref, INC,
            1.0, f_pltf_out, INC);
}


void hddc2b_pltf_frc_redu_wgh_init(
        int num_drv,
        const double *g_in,
        const double *w_drv_inv_sqrt,
        double *g_out)
{
    assert(num_drv >= 0);
    assert(g_in);
    assert(w_drv_inv_sqrt);
    assert(g_out);

    const int LDWO = NUM_DRV_COORD * NUM_DRV_COORD;     // outer array
    const int LDWI = NUM_DRV_COORD;                     // inner array
    const int LDGO = NUM_PLTF_COORD * NUM_DRV_COORD;    // outer array
    const int LDGI = NUM_PLTF_COORD;                    // inner array

    for (int i = 0; i < num_drv; i++) {
        // G'[3x2] = G[3x2] W_d^{1/2}[2x2]
        cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                NUM_PLTF_COORD, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, &g_in[i * LDGO], LDGI,
                &w_drv_inv_sqrt[i * LDWO], LDWI,
                0.0, &g_out[i * LDGO], LDGI);
    }
}


void hddc2b_pltf_frc_redu_wgh_fini(
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
        // F_d'[2x1] = W_d^{-1/2}[2x2] F_d[2x1]
        cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, &w_drv_inv_sqrt[i * LDWO], LDWI,
                &f_drv_in[i * LDF], INC,
                0.0, &f_drv_out[i * LDF], INC);
    }
}


void hddc2b_pltf_frc_redu_ref_fini(
        int num_drv,
        const double *f_drv_in1,
        const double *f_drv_in2,
        double *f_drv_out)
{
    assert(num_drv >= 0);
    assert(f_drv_in1);
    assert(f_drv_in2);
    assert(f_drv_out);

    for (int i = 0; i < num_drv * NUM_DRV_COORD; i++) {
        f_drv_out[i] = f_drv_in1[i] + f_drv_in2[i];
    }
}


void hddc2b_pltf_frc_slv(
        int num_drv,
        const double *u,
        const double *s_inv,
        const double *vt,
        const double *f_pltf,
        double *f_drv)
{
    assert(num_drv >= 0);
    assert(u);
    assert(s_inv);
    assert(vt);
    assert(f_pltf);
    assert(f_drv);

    const int NUM_EIG = MIN(NUM_PLTF_COORD, num_drv * NUM_DRV_COORD);
    const int LDU     = NUM_PLTF_COORD;

    //
    // V[NCxNE] S^{-1}[NEx1] U^T[NExNE] F_p[NCx1]
    //
    double utfp[NUM_EIG];
    double sutfp[NUM_EIG];
    const int INC = 1;

    // U^T[NExNE] *
    cblas_dgemv(CblasColMajor, CblasTrans, NUM_EIG, NUM_EIG,
            1.0, u, LDU, f_pltf, INC,
            0.0, utfp, INC);

    // S^{-1}[NEx1] *
    for (int i = 0; i < NUM_EIG; i++) {
        sutfp[i] = utfp[i] * s_inv[i];
    }

    // V[NCxNE] *
    cblas_dgemv(CblasColMajor, CblasTrans, NUM_EIG, num_drv * NUM_DRV_COORD,
            1.0, vt, NUM_EIG, sutfp, INC,
            0.0, f_drv, INC);
}




void hddc2b_pltf_vel_w_pltf_inv_sqrt(
        const double *w_pltf,
        double *w_pltf_inv_sqrt)
{
    assert(w_pltf);
    assert(w_pltf_inv_sqrt);

    const int LDW = NUM_PLTF_COORD;

    //
    // Z_p[3x3], L_p[3x1] = eig(W_p[3x3])
    //
    const char JOB  = 'V';  // compute eigenvalues and eigenvectors
    const char UPLO = 'U';  // use upper triangular matrix
    const int LDZ = NUM_PLTF_COORD;
    double z[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double l[NUM_PLTF_COORD];
    const int LWORK = 15;
    double work[LWORK];

    memcpy(z, w_pltf, NUM_PLTF_COORD * NUM_PLTF_COORD * sizeof(double));
    LAPACKE_dsyev_work(LAPACK_COL_MAJOR, JOB, UPLO, NUM_PLTF_COORD,
            z, LDW, l, work, LWORK);


    //
    // W_p^{-1/2}[3x3] = Z_p[3x3] S_p^{-1/2}[3x1] Z_p^T[3x3]
    //

    // Z_p[3x3] * diag(1 / sqrt(L_p[3x1]))
    double z_lsqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    for (int row = 0; row < NUM_PLTF_COORD; row++) {
        for (int col = 0; col < NUM_PLTF_COORD; col++) {
            // For an eigenvalue "\lambda" and an eigenvector "v" it holds that
            //   \lambda v = -\lambda -v
            // Hence, the eigendecomposition can produce either positive or
            // negative eigenvalues. Since in either case the magnitude remains
            // the same, we use the absolute eigenvalue here.
            double l_abs = fabs(l[row]);
            double l_inv = (l_abs < EPS) ? 0.0 : 1.0 / sqrt(l_abs);
            z_lsqrt[row * LDZ + col] = z[row * LDZ + col] * l_inv;
        }
    }

    // * Z_p^T[3x3]
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
            NUM_PLTF_COORD, NUM_PLTF_COORD, NUM_PLTF_COORD,
            1.0, z_lsqrt, LDZ,
            z, LDZ,
            0.0, w_pltf_inv_sqrt, LDW);
}


void hddc2b_pltf_vel_w_drv_sqrt(
        int num_drv,
        const double *w_drv,
        double *w_drv_sqrt)
{
    assert(num_drv >= 0);
    assert(w_drv);
    assert(w_drv_sqrt);

    const int LDWO = NUM_DRV_COORD * NUM_DRV_COORD; // outer array
    const int LDWI = NUM_DRV_COORD;                 // inner array

    for (int i = 0; i < num_drv; i++) {
        //
        // Z_d[2x2], L_d[2x1] = eig(W_d[2x2])
        //
        const char JOB  = 'V';  // compute eigenvalues and eigenvectors
        const char UPLO = 'U';  // use upper triangular matrix
        const int LDZ = NUM_DRV_COORD;
        double z[NUM_DRV_COORD * NUM_DRV_COORD];
        double l[NUM_DRV_COORD];
        const int LWORK = 10;
        double work[LWORK];

        memcpy(z, &w_drv[i * LDWO],
                NUM_DRV_COORD * NUM_DRV_COORD * sizeof(double));
        LAPACKE_dsyev_work(LAPACK_COL_MAJOR, JOB, UPLO, NUM_DRV_COORD,
                z, LDZ, l, work, LWORK);


        //
        // W_d^{1/2}[2x2] = Z_d[2x2] L_d^{1/2}[2x1] Z_d^T[2x2]
        //

        // Z_d[2x2] * diag(sqrt(L_d[2x1]))
        double z_linvsqrt[NUM_DRV_COORD * NUM_DRV_COORD];
        for (int row = 0; row < NUM_DRV_COORD; row++) {
            for (int col = 0; col < NUM_DRV_COORD; col++) {
                // For an eigenvalue "\lambda" and an eigenvector "v" it holds
                // that
                //   \lambda v = -\lambda -v
                // Hence, the eigendecomposition can produce either positive or
                // negative eigenvalues. Since in either case the magnitude
                // remains the same, we use the absolute eigenvalue here.
                double l_abs = sqrt(fabs(l[row]));
                z_linvsqrt[row * LDZ + col] = z[row * LDZ + col] * l_abs;
            }
        }

        // * Z_d^T[2x2]
        cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
                NUM_DRV_COORD, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, z_linvsqrt, LDZ,
                z, LDZ,
                0.0, &w_drv_sqrt[i * LDWO], LDWI);
    }
}


void hddc2b_pltf_vel_sing_wgh(
        int num_drv,
        const double *g_in,
        const double *xd_drv_in,
        const double *w_drv_sqrt,
        double *g_out,
        double *xd_drv_out)
{
    assert(num_drv >= 0);
    assert(g_in);
    assert(xd_drv_in);
    assert(w_drv_sqrt);
    assert(g_out);
    assert(xd_drv_out);

    const int LDWO = NUM_DRV_COORD * NUM_DRV_COORD;     // outer array
    const int LDWI = NUM_DRV_COORD;                     // inner array
    const int LDGO = NUM_PLTF_COORD * NUM_DRV_COORD;    // outer array
    const int LDGI = NUM_PLTF_COORD;                    // inner array
    const int LDXD = 2;
    const int INC  = 1;

    for (int i = 0; i < num_drv; i++) {
        // G'[3x2] = G[3x2] W_d^{1/2}[2x2]
        cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
                NUM_PLTF_COORD, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, &g_in[i * LDGO], LDGI,
                &w_drv_sqrt[i * LDWO], LDWI,
                0.0, &g_out[i * LDGO], LDGI);

        // Xd_d'[2x1] = W_d^{1/2}[2x2] Xd_d[2x1]
        cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_DRV_COORD, NUM_DRV_COORD,
                1.0, &w_drv_sqrt[i * LDWO], LDWI,
                &xd_drv_in[i * LDXD], INC,
                0.0, &xd_drv_out[i * LDXD], INC);
    }
}


void hddc2b_pltf_vel_redu_ref_init(
        int num_drv,
        const double *g,
        const double *xd_drv_in,
        const double *xd_pltf_ref,
        double *xd_drv_out)
{
    assert(num_drv >= 0);
    assert(g);
    assert(xd_drv_in);
    assert(xd_pltf_ref);
    assert(xd_drv_out);

    const int LDG = NUM_PLTF_COORD;
    const int INC = 1;

    // Xd_d[NCx1] = Xd_d'[NCx1] - G^T[NCx3] Xd_p[3x1]
    memcpy(xd_drv_out, xd_drv_in, num_drv * NUM_DRV_COORD * sizeof(double));
    cblas_dgemv(CblasColMajor, CblasTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD,
            -1.0, g, LDG,
            xd_pltf_ref, INC,
            1.0, xd_drv_out, INC);
}


void hddc2b_pltf_vel_redu_wgh_init(
        int num_drv,
        const double *g_in,
        const double *w_pltf_inv_sqrt,
        double *g_out)
{
    assert(num_drv >= 0);
    assert(g_in);
    assert(w_pltf_inv_sqrt);
    assert(g_out);

    const int LDG = NUM_PLTF_COORD;
    const int LDW = NUM_PLTF_COORD;

    // G'[3xNC] = W_p^{-1/2}[3x3] G[3xNC]
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD, NUM_PLTF_COORD,
            1.0, w_pltf_inv_sqrt, LDW,
            g_in, LDG,
            0.0, g_out, LDG);
}


void hddc2b_pltf_vel_redu_wgh_fini(
        int num_drv,
        const double *xd_pltf_in,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf_out)
{
    assert(num_drv >= 0);
    assert(xd_pltf_in);
    assert(w_pltf_inv_sqrt);
    assert(xd_pltf_out);

    const int LDW = 3;
    const int INC = 1;

    // Xd_p'[3x1] = W_p^{-1/2}[3x3] Xd_p[3x1]
    cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_PLTF_COORD, NUM_PLTF_COORD,
            1.0, w_pltf_inv_sqrt, LDW,
            xd_pltf_in, INC,
            0.0, xd_pltf_out, INC);
}


void hddc2b_pltf_vel_redu_ref_fini(
        const double *xd_pltf_in1,
        const double *xd_pltf_in2,
        double *xd_pltf_out)
{
    assert(xd_pltf_in1);
    assert(xd_pltf_in2);
    assert(xd_pltf_out);

    for (int i = 0; i < NUM_PLTF_COORD; i++) {
        xd_pltf_out[i] = xd_pltf_in1[i] + xd_pltf_in2[i];
    }
}


void hddc2b_pltf_vel_slv(
        int num_drv,
        const double *u,
        const double *s_inv,
        const double *vt,
        const double *xd_drv,
        double *xd_pltf)
{
    assert(num_drv >= 0);
    assert(u);
    assert(s_inv);
    assert(vt);
    assert(xd_drv);
    assert(xd_pltf);

    const int NUM_EIG = MIN(NUM_PLTF_COORD, num_drv * NUM_DRV_COORD);
    const int LDU     = NUM_PLTF_COORD;
    const int LDVT    = NUM_PLTF_COORD;

    //
    // U[3xNE] S^{-1}[NEx1] V^T[NExNC] Xd_d[NCx1]
    //
    double vtxdd[NUM_EIG];
    double sivtxdd[NUM_EIG];
    const int INC = 1;

    // V^T[NExNC] * Xd_d[NCx1]
    cblas_dgemv(CblasColMajor, CblasNoTrans, NUM_EIG, num_drv * NUM_DRV_COORD,
            1.0, vt, LDVT, xd_drv, INC,
            0.0, vtxdd, INC);

    // S^{-1}[NEx1] *
    for (int i = 0; i < NUM_EIG; i++) {
        sivtxdd[i] = vtxdd[i] * s_inv[i];
    }

    // U[3xNE] *
    cblas_dgemv(CblasColMajor, CblasNoTrans,
            NUM_PLTF_COORD, NUM_EIG,
            1.0, u, LDU, sivtxdd, INC,
            0.0, xd_pltf, INC);
}




void hddc2b_pltf_dcmp(
        int num_drv,
        const double *g,
        double *u,
        double *s,
        double *vt)
{
    assert(num_drv >= 0);
    assert(g);
    assert(u);
    assert(s);
    assert(vt);

    const int LDG  = NUM_PLTF_COORD;
    const int LDU  = NUM_PLTF_COORD;
    const int LDVT = MIN(NUM_PLTF_COORD, num_drv * NUM_DRV_COORD);

    //
    // U[3xNE] S[NE] V^T[NExNC] = svd(G[3xNC])
    //
    const char JOBU  = 'S';                     // compute a ..
    const char JOBVT = 'S';                     // ... "thin SVD"
    const int LWORK = 15 + ((num_drv * NUM_DRV_COORD) - 4);
    double work[LWORK];

    // M=3, N=NC
    double g_[num_drv * NUM_G_COORD];
    memcpy(g_, g, num_drv * NUM_G_COORD * sizeof(double));
    LAPACKE_dgesvd_work(LAPACK_COL_MAJOR, JOBU, JOBVT,
            NUM_PLTF_COORD, num_drv * NUM_DRV_COORD, g_, LDG,
            s, u, LDU, vt, LDVT, work, LWORK);
}


void hddc2b_pltf_pinv(
        int num_drv,
        double eps,
        const double *s,
        double *s_inv)
{
    assert(num_drv >= 0);
    assert(eps >= 0.0);
    assert(s);
    assert(s_inv);

    // For a singular platform there are only 2 eigenvalues, else 3 eigenvalues
    const int NUM_EIG = MIN(NUM_PLTF_COORD, num_drv * NUM_DRV_COORD);

    for (int i = 0; i < NUM_EIG; i++) {
        s_inv[i] = (fabs(s[i]) < eps) ? 0.0 : 1.0 / s[i];
    }
}


void hddc2b_pltf_dmp(
        int num_drv,
        double eps,
        double lambda,
        const double *s,
        double *s_inv)
{
    assert(num_drv >= 0);
    assert(eps >= 0.0);
    assert(lambda >= 0.0);
    assert(s);
    assert(s_inv);

    // For a singular platform there are only 2 eigenvalues, else 3 eigenvalues
    const int NUM_EIG = MIN(NUM_PLTF_COORD, num_drv * NUM_DRV_COORD);

    double s_min = s[NUM_EIG - 1];
    double lambda_sqr = 0.0;

    // Activate damping only if the smallest eigenvalue is less than epsilon
    if (s_min < eps) {
        double scale = s_min / eps;
        double scale_sqr = scale * scale;
        lambda_sqr = (1.0 - scale_sqr) * lambda * lambda;
    }

    for (int i = 0; i < NUM_EIG; i++) {
        double s_sqr = s[i] * s[i];
        s_inv[i] = s[i] / (s_sqr + lambda_sqr);
    }
}
