// SPDX-License-Identifier: LGPL-3.0
#ifndef SRC_EXAMPLE_SOLVER_H
#define SRC_EXAMPLE_SOLVER_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units), compute the weight decomposition internally and
 * include a reference drive force in the nullspace.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] g The force composition matrix @f$\vect{G}@f$ with three rows and
 *              @f$2 \times {}@f$ @p num_drv columns, measured in the platform's
 *              origin and its coordinates expressed in the platform's frame.
 *              The matrix is arranged as @f[
 *              \begin{bmatrix}
 *                \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *              \end{bmatrix}
 *              @f] and must be provided in column-major order. Here,
 *              @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the platform
 *              forces and torque, respectively. @f$f_{i,x}@f$ and @f$f_{i,y}@f$
 *              are the drive's forces.
 * @param[in] w_pltf The platform's weight matrix @f$\vect{W}_p@f$, a
 *                   positive-definite @f$3 \times 3@f$ matrix. The weight is
 *                   measured in the platform's origin and its coordinates are
 *                   expressed in the platform frame. The matrix is arranged as
 *                   @f[
 *                   \begin{bmatrix}
 *                     w_{p,xx} & w_{p,yx} & w_{p,mx} \\
 *                     w_{p,xy} & w_{p,yy} & w_{p,my} \\
 *                     w_{p,xm} & w_{p,ym} & w_{p,mm}
 *                   \end{bmatrix}
 *                   @f] where @f$w_{p,yx} = w_{p,xy}@f$,
 *                   @f$w_{p,mx} = w_{p,xm}@f$ and @f$w_{p,my} = w_{p,ym}@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[in] w_drv An array that consists of @p num_drv positive-definite
 *                  @f$2 \times 2@f$ weight matrices @f$\vect{W}_d@f$, each
 *                  measured in the respective drive's attachment point and its
 *                  coordinates expressed in the drive's attachment frame. The
 *                  array is arranged as @f[
 *                  \begin{bmatrix}
 *                    w_{1,xx} & \ldots & w_{n,xx} \\
 *                    w_{1,yx} & \ldots & w_{n,yx} \\
 *                    w_{1,xy} & \ldots & w_{n,xy} \\
 *                    w_{1,yy} & \ldots & w_{n,yy}
 *                  \end{bmatrix}
 *                  @f] where @f$w_{i,yx} = w_{i,xy}@f$ and must be provided in
 *                  column-major order.
 * @param[in] f_drv_ref The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      force reference components that are projected into the
 *                      nullspace of the platform task. The forces' coordinates
 *                      are expressed in the individual drives' pivot frames.
 *                      The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and will be provided in column-major order.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_example_frc(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv);


/**
 * Find a solution to the velocity composition problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units).
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] g The force composition matrix @f$\vect{G}@f$ with three rows and
 *              @f$2 \times {}@f$ @p num_drv columns, measured in the platform's
 *              origin and its coordinates expressed in the platform's frame.
 *              The matrix is arranged as @f[
 *              \begin{bmatrix}
 *                \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *              \end{bmatrix}
 *              @f] and must be provided in column-major order. Here,
 *              @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the platform
 *              forces and torque, respectively. @f$f_{i,x}@f$ and @f$f_{i,y}@f$
 *              are the drive's forces.
 * @param[in] w_pltf_inv_sqrt The inverse square root of the platform's weight
 *                            matrix @f$\vect{W}_p^{-\frac{1}{2}}@f$, a
 *                            positive-definite @f$3 \times 3@f$ matrix. The
 *                            weight is measured in the platform's origin and
 *                            its coordinates are expressed in the platform
 *                            frame. The matrix is arranged as @f[
 *                            \begin{bmatrix}
 *                              w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                              w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                              w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                            \end{bmatrix}
 *                            @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                            @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                            @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[in] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                   @p num_drv columns where the rows represent the linear
 *                   velocity components of each drive. The velocities'
 *                   reference point is the origin of the individual drives'
 *                   pivot frames and the coordinates are expressed in these
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 * @param[in] w_drv_sqrt An array that consists of @p num_drv positive-definite
 *                       @f$2 \times 2@f$ weight matrices' square roots
 *                       @f$\vect{W}_d^{\frac{1}{2}}@f$, each measured in the
 *                       respective drive's attachment point and its coordinates
 *                       expressed in the drive's attachment frame. The array is
 *                       arranged as @f[
 *                       \begin{bmatrix}
 *                         w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                         w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                         w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                         w_{1,yy}' & \ldots & w_{n,yy}'
 *                       \end{bmatrix}
 *                       @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                       provided in column-major order.
 * @param[out] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                     that represent the linear and angular velocity of the
 *                     platform. The linear velocitie's reference point is the
 *                     platform's origin. The coordinates are expressed in the
 *                     platform frame. The vector is arranged as @f$
 *                     \begin{bmatrix}
 *                       v_{p,x} & v_{p,y} & \omega_p
 *                     \end{bmatrix}@f$.
 */
void hddc2b_example_vel(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf);


#ifdef __cplusplus
}
#endif

#endif
