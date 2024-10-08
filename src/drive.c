// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/drive.h>
#include <config.h>

#include <math.h>
#include <assert.h>


void hddc2b_drv_frc_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_whl,
        double *f_drv)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(f_whl);
    assert(f_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(l) > 0.0);

        f_drv[x] =      1.0     * (f_whl[left] - f_whl[right]);
        f_drv[y] = -0.5 * r / l * (f_whl[left] + f_whl[right]);
    }
}


void hddc2b_drv_frc_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_drv,
        double *f_whl)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(f_whl);
    assert(f_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(r) > 0.0);

        f_whl[right] = -0.5 * f_drv[x] - l / r * f_drv[y];
        f_whl[left ] =  0.5 * f_drv[x] - l / r * f_drv[y];
    }
}


void hddc2b_drv_vel_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_whl,
        double *xd_drv)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(xd_whl);
    assert(xd_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(l) > 0.0);

        xd_drv[x] =   0.5  * (xd_whl[left] - xd_whl[right]);
        xd_drv[y] = -l / r * (xd_whl[left] + xd_whl[right]);
    }
}
