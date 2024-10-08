// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/wheel.h>
#include <config.h>

#include <math.h>
#include <assert.h>


void hddc2b_whl_frc_hub_to_gnd(
        int num_drv,
        const double *whl_dia,
        const double *tau_whl,
        double *f_whl)
{
    assert(num_drv >= 0);
    assert(whl_dia);
    assert(tau_whl);
    assert(f_whl);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;

        f_whl[right] = tau_whl[right] / (whl_dia[right] / 2.0);
        f_whl[left ] = tau_whl[left ] / (whl_dia[left ] / 2.0);
    }
}


void hddc2b_whl_frc_gnd_to_hub(
        int num_drv,
        const double *whl_dia,
        const double *f_whl,
        double *tau_whl)
{
    assert(num_drv >= 0);
    assert(whl_dia);
    assert(f_whl);
    assert(tau_whl);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;

        tau_whl[right] = (whl_dia[right] / 2.0) * f_whl[right];
        tau_whl[left ] = (whl_dia[left ] / 2.0) * f_whl[left ];
    }
}


void hddc2b_whl_vel_hub_to_gnd(
        int num_drv,
        const double *whl_dia,
        const double *omega_whl,
        double *xd_whl)
{
    assert(num_drv >= 0);
    assert(whl_dia);
    assert(omega_whl);
    assert(xd_whl);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;

        xd_whl[right] = (whl_dia[right] / 2.0) * omega_whl[right];
        xd_whl[left ] = (whl_dia[left ] / 2.0) * omega_whl[left ];
    }
}
