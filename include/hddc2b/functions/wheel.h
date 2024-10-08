// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_FUNCTIONS_WHEEL_H
#define HDDC2B_FUNCTIONS_WHEEL_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Solve the wheels' forward force kinematics problem, i.e. from actuator torque
 * to (linear) force at the wheel-ground contact point.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dia An array with two rows and @p num_drv columns where the
 *                    rows contain the diameter of the right and left wheel,
 *                    respectively. The array is arranged as @f[
 *                    \begin{bmatrix}
 *                      d_{1,r} & \ldots & d_{n,r} \\
 *                      d_{1,l} & \ldots & d_{n,l}
 *                    \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[in] tau_hub A matrix with two rows and @p num_drv columns where the
 *                    rows contain the actuator torque around the right and left
 *                    wheel axle, respectively. The matrix is arranged as @f[
 *                    \begin{bmatrix}
 *                      \tau_{1,r} & \ldots & \tau_{n,r} \\
 *                      \tau_{1,l} & \ldots & \tau_{n,l}
 *                    \end{bmatrix}
 *                     @f] and must be provided in column-major order.
 * @param[out] f_whl A matrix with two rows and @p num_drv columns where the
 *                   rows contain the linear force at the right and left
 *                   wheel-ground contact point, respectively. The matrix is
 *                   arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,r} & \ldots & f_{n,r} \\
 *                     f_{1,l} & \ldots & f_{n,l}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_whl_frc_hub_to_gnd(
        int num_drv,
        const double *whl_dia,
        const double *tau_hub,
        double *f_whl);


/**
 * Solve the wheels' inverse force kinematics problem, i.e. from (linear) force
 * at the wheel-ground contact point to actuator torque.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dia An array with two rows and @p num_drv columns where the
 *                    rows contain the diameter of the right and left wheel,
 *                    respectively. The array is arranged as @f[
 *                    \begin{bmatrix}
 *                      d_{1,r} & \ldots & d_{n,r} \\
 *                      d_{1,l} & \ldots & d_{n,l}
 *                    \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[in] f_whl A matrix with two rows and @p num_drv columns where the rows
 *                  contain the linear force at the right and left wheel-ground
 *                  contact point, respectively. The matrix is arranged as @f[
 *                  \begin{bmatrix}
 *                    f_{1,r} & \ldots & f_{n,r} \\
 *                    f_{1,l} & \ldots & f_{n,l}
 *                  \end{bmatrix}
 *                  @f] and must be provided in column-major order.
 * @param[out] tau_hub A matrix with two rows and @p num_drv columns where the
 *                     rows contain the actuator torque around the right and
 *                     left wheel axle, respectively. The matrix is arranged as
 *                     @f[
 *                     \begin{bmatrix}
 *                       \tau_{1,r} & \ldots & \tau_{n,r} \\
 *                       \tau_{1,l} & \ldots & \tau_{n,l}
 *                     \end{bmatrix}
 *                     @f] and will be provided in column-major order.
 */
void hddc2b_whl_frc_gnd_to_hub(
        int num_drv,
        const double *whl_dia,
        const double *f_whl,
        double *tau_hub);


/**
 * Solve the wheels' forward velocity kinematics problem, i.e. from actuator
 * velocity to linear velocity at the wheel-ground contact point.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dia An array with two rows and @p num_drv columns where the
 *                    rows contain the diameter of the right and left wheel,
 *                    respectively. The array is arranged as @f[
 *                    \begin{bmatrix}
 *                      d_{1,r} & \ldots & d_{n,r} \\
 *                      d_{1,l} & \ldots & d_{n,l}
 *                    \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[in] omega_hub A matrix with two rows and @p num_drv columns where the
 *                      rows contain the angular velocity around the right and
 *                      left wheel axle, respectively. The matrix is arranged as
 *                      @f[
 *                      \begin{bmatrix}
 *                        \omega_{1,r} & \ldots & \omega_{n,r} \\
 *                        \omega_{1,l} & \ldots & \omega_{n,l}
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
 * @param[out] xd_whl A matrix with two rows and @p num_drv columns where the
 *                    rows contain the linear velocity at the right and left
 *                    wheel-ground contact point, respectively. The matrix is
 *                    arranged as @f[
 *                    \begin{bmatrix}
 *                      \dot{X}_{1,r} & \ldots & \dot{X}_{n,r} \\
 *                      \dot{X}_{1,l} & \ldots & \dot{X}_{n,l}
 *                    \end{bmatrix}
 *                    @f] and will be provided in column-major order.
 */
void hddc2b_whl_vel_hub_to_gnd(
        int num_drv,
        const double *whl_dia,
        const double *omega_hub,
        double *xd_whl);


#ifdef __cplusplus
}
#endif

#endif
