// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/platform.h>
#include <hddc2b/functions/drive.h>
#include <hddc2b/functions/wheel.h>
#include <solver.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


void print_matrix(
        int rows,
        int cols,
        const double *a)
{
    printf("[");
    for (int m_ = 0; m_ < rows; m_++) {
        printf("[");
        for (int n_ = 0; n_ < cols; n_++) {
            printf("%10f", a[m_ + n_ * rows]);
            if (n_ != cols - 1) printf(", ");
        }
        printf("]");
        if (m_ != rows - 1) printf(",\n ");
    }
    printf("]");
}


#define NUM_DRV        4
#define NUM_WHL_COORD  2
#define NUM_GND_COORD  2
#define NUM_DRV_COORD  2
#define NUM_PLTF_COORD 3
#define NUM_G_COORD    ((NUM_DRV_COORD) * (NUM_PLTF_COORD))
#define EPS            0.001


int main(void)
{
    // Attachment position of each drive unit w.r.t. the platform's origin.
    // The values here originate from the KELO Robotics DT500 platform.
    double drive_attachment[NUM_DRV * 2] = {    // [m]
         0.175,  0.1605,                        // fl-x, fl-y
        -0.175,  0.1605,                        // rl-x, rl-y
        -0.175, -0.1605,                        // rr-x, rr-y
         0.175, -0.1605                         // fr-x, fr-y
    };

    // Diameter of each wheel.
    double wheel_diameter[NUM_DRV * 2] = {      // [m]
        0.115, 0.115,                           // fl-r, fl-l
        0.115, 0.115,                           // rl-r, rl-l
        0.115, 0.115,                           // rr-r, rr-l
        0.115, 0.115                            // fr-r, fr-l
    };

    // Kinematic parameters of the differential drive part:
    // Distance of the wheels from the centre between the wheels.
    double wheel_distance[NUM_DRV] = {          // [m]
        0.0775, 0.0775, 0.0775, 0.0775          // fl, rl, rr, fr
    };

    // Kinematic parameters of the castor drive part:
    // Distance of the axle from the pivot joint's axis
    double castor_offset[NUM_DRV] = {           // [m]
        0.01, 0.01, 0.01, 0.01                  // fl, rl, rr, fr
    };

    // The pivot angles describing the orientation of each drive unit w.r.t to
    // the platform:
    // - front-left:  push configuration (for positive linear-x platform force)
    // - rear-left:   worst efficiency (misaligned for linear-x platform force)
    // - rear-right:  pull configuration (for positive linear-x platform force)
    // - front-right: best alignment for positive angular platform force
    double pivot_angle[NUM_DRV] = {             // [rad]
        0.0,                                    // fl
        1.0 * M_PI_2,                           // rl
        2.0 * M_PI_2,                           // rr
        atan(0.175 / 0.1605)                    // fr
    };


    // The actually desired platform-level force.
    double f_platform[NUM_PLTF_COORD] = {       // [N], [N], [Nm]
        1.0, 0.0, 0.0                           // x, y, mz
    };

    // For _singular_ platforms the relative weight between the platform-level
    // forces, i.e. for the platform in this example it has no impact on the
    // force distribution.
    double w_platform[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        // [1/N^2], [1/(N Nm)], [1/(Nm)^2]
        1.0, 0.0, 0.0,                          // xx, xy, xm
        0.0, 1.0, 0.0,                          // yx, yy, ym
        0.0, 0.0, 1.0                           // mx, my, mm
    };

    // For _redundant_ platforms the relative weight between the drive-level
    // forces. The entries here disable the front-right drive unit.
    double w_drive[NUM_DRV * 4] = {             // [1/N^2]
        1.0, 0.0, 0.0, 1.0,                     // fl-xx, fl-xy, fl-yx, fl-yy
        1.0, 0.0, 0.0, 1.0,                     // rl-xx, rl-xy, rl-yx, rl-yy
        1.0, 0.0, 0.0, 1.0,                     // rr-xx, rr-xy, rr-yx, rr-yy
        0.0, 0.0, 0.0, 0.0                      // fr-xx, fr-xy, fr-yx, fr-yy
    };

    // Reference drive forces that will
    // (i) be computed from the "misalignment" of the drive units w.r.t. the
    //     platform-level task; and
    // (ii) projected into the nullspace of the platform-level task.
    double f_drive_ref[NUM_DRV * NUM_DRV_COORD] = {
        0.0, 0.0,                               // fl-x, fl-y
        0.0, 0.0,                               // rl-x, rl-y
        0.0, 0.0,                               // rr-x, rr-y
        0.0, 0.0                                // fr-x, fr-y
    };

    // Weight of the angular and linear alignment distance, respectively.
    // The weight for the front-right drive unit means that it will always have
    // a "zero" alignment distance.
    double w_align[NUM_DRV * 2] = {             //
        1.0, 1.0,                               // fl-ang, fl-lin
        1.0, 1.0,                               // rl-ang, rl-lin
        1.0, 1.0,                               // rr-ang, rr-lin
        0.0, 0.0                                // fr-ang, fr-lin
    };

    // Force composition matrix (from drive forces to platform force)
    double g[NUM_DRV * NUM_G_COORD];

    // Drive force that results from distributing the platform-level force
    double f_drive[NUM_DRV * NUM_DRV_COORD];    // [N]
    bzero(f_drive, NUM_DRV * NUM_GND_COORD * sizeof(double));

    // Force at the wheel-ground contact point
    double f_wheel[NUM_DRV * NUM_GND_COORD];    // [N]

    // Actuator torque
    double tau_wheel[NUM_DRV * NUM_WHL_COORD];  // [Nm]

    // Re-composed drive force which should equal the input, platform-level
    // force
    double f_platform_out[NUM_PLTF_COORD];      // [N], [N], [Nm]

    double omega_hub[NUM_DRV * NUM_DRV_COORD] = {   // [rad/s], [rad/s]
        -17.39130435,  17.39130435,                 // fl-r, fl-l
         67.39130435,  67.39130435,                 // rl-r, rl-l
         17.39130435, -17.39130435,                 // rr-r, rr-l
         37.91094792,  61.42104555                  // fr-r, fr-l
    };
    double xd_ground[NUM_DRV * NUM_DRV_COORD];      // [m/s], [m/s]
    double xd_drive[NUM_DRV * NUM_DRV_COORD];       // [m/s], [m/s]
    double xd_platform[NUM_PLTF_COORD];             // [m/s], [m/s], [rad/s]


    //
    // Compute an alignment torque for the pivot joints that is used as the "y"
    // coordinate of the reference force for the drive units
    //
    hddc2b_pltf_drv_algn_dst(NUM_DRV,
            drive_attachment,
            w_align,
            pivot_angle,
            f_platform,
            &f_drive_ref[1],
            2);
    printf("f_drive_ref:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_drive_ref);


    //
    // Force distribution to wheels ...
    //

    hddc2b_pltf_frc_comp_mat(NUM_DRV,
            drive_attachment,
            pivot_angle,
            g);
    printf("\ng:\n");
    print_matrix(NUM_PLTF_COORD, NUM_DRV * NUM_DRV_COORD, g);

    hddc2b_example_frc(NUM_DRV,
            EPS,
            g,
            w_platform,
            f_platform,
            w_drive,
            f_drive_ref,
            f_drive);
    printf("\nf_drive:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_drive);

    hddc2b_drv_frc_pvt_to_gnd(NUM_DRV,
            wheel_distance,
            castor_offset,
            f_drive,
            f_wheel);
    printf("\nf_wheel:\n");
    print_matrix(NUM_GND_COORD, NUM_DRV, f_wheel);

    hddc2b_whl_frc_gnd_to_hub(NUM_DRV,
            wheel_diameter,
            f_wheel,
            tau_wheel);
    printf("\ntau_wheel:\n");
    print_matrix(NUM_WHL_COORD, NUM_DRV, tau_wheel);


    //
    // ... and force composition back to the platform
    //

    hddc2b_whl_frc_hub_to_gnd(NUM_DRV,
            wheel_diameter,
            tau_wheel,
            f_wheel);
    printf("\nf_wheel:\n");
    print_matrix(NUM_GND_COORD, NUM_DRV, f_wheel);

    hddc2b_drv_frc_gnd_to_pvt(NUM_DRV,
            wheel_distance,
            castor_offset,
            f_wheel,
            f_drive);
    printf("\nf_drive:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_drive);

    hddc2b_pltf_frc_pvt_to_pltf(NUM_DRV,
            g,
            f_drive,
            f_platform_out);
    printf("\nf_platform_out:\n");
    print_matrix(1, NUM_PLTF_COORD, f_platform_out);
    printf("\n");



    //
    // Velocity composition from wheels to platform
    //

    hddc2b_whl_vel_hub_to_gnd(NUM_DRV,
            wheel_diameter,
            omega_hub,
            xd_ground);
    printf("\nxd_ground:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, xd_ground);

    hddc2b_drv_vel_gnd_to_pvt(NUM_DRV,
            wheel_distance,
            castor_offset,
            xd_ground,
            xd_drive);
    printf("\nxd_drive:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, xd_drive);

    hddc2b_example_vel(NUM_DRV,
            EPS,
            g,
            w_drive,
            xd_drive,
            w_platform,
            xd_platform);
    printf("\nxd_platform:\n");
    print_matrix(NUM_PLTF_COORD, 1, xd_platform);
    printf("\n");

    return 0;
}
