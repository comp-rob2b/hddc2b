// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_FUNCTIONS_PLATFORM_H
#define HDDC2B_FUNCTIONS_PLATFORM_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Compute a signed and weighted distance between each drive's orientation with
 * respect to the platform-level task. This distance can be interpreted as a
 * pivot torque that aligns a drive with the platform-level task. It is modeled
 * as two competing tasks: one for the alignment with the angular part (the
 * torque) of the task and the other for the alginment with the linear part (the
 * force) of the task. The final command is the weighted sum of these two tasks.
 *
 * Each drive has a particular orientation with respect to the platform - its
 * _actual_ configuration. For both tasks there exists a _desired_
 * configuration:
 * - For the alignment with the linear part of the task, we simply interpret
 *   that linear part as a direction vector (complemented to a right-handed
 *   frame).
 * - For the alignment with the angular part of the task, we assume a best-case
 *   solution to produce a torque about the platform's origin. Here, the drive
 *   should be orthogonal to the vector from the platform's origin to the
 *   attachment's origin. In other words, the "pushing" vector should be
 *   tangential to a circle around the platform's origin with a radius equal to
 *   the length of the platform-attachment vector.
 *
 * For each of the two desired configurations we can determine the orientation
 * (rather an angular distance or "difference") with respect to the actual
 * configuration. This difference is to be interpreted as an angular velocity
 * (to be executed over one unit of time). A "damping" or "weight" maps this
 * angular velocity into a torque. The sum of the torque due to the angular and
 * the linear task is added and returned by this function.
 *
 * Better solution: don't let the two tasks compete and coordinate implicitly
 * (via a weighted sum) but reify the two tasks (each in their custom activity)
 * and let them cooperate via explicit coordination (in yet another activity)!
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] pos_drv An array of @p num_drv position vectors that represent the
 *                    position of each drive's attachment point with respect to
 *                    the platform's origin. The coordinates are expressed in
 *                    the platform frame. The array is arranged as @f[
 *                    \begin{bmatrix}
 *                      x_1 & \ldots & x_n \\
 *                      y_1 & \ldots & y_n
 *                    \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[in] w An array that represents a weight for the angular and the linear
 *              alignment tasks for each drive. The array is arranged as @f[
 *              \begin{bmatrix}
 *                w_{ang,1} & \ldots & w_{ang,n} \\
 *                w_{lin,1} & \ldots & w_{lin,n}
 *              \end{bmatrix}
 *              @f] and must be provided in column-major order.
 * @param[in] q_pvt The array @f$\vect{q}_{pvt}@f$ of @p num_drv pivot angles
 *                  arranged as @f$
 *                  \begin{bmatrix}
 *                    q_1 & \ldots & q_n
 *                  \end{bmatrix}@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[out] dst An array with @p num_drv elements that represent the pivot
 *                 distances of each drive with respect to the platform-level
 *                 task. The array is arranged as @f$
 *                 \begin{bmatrix}
 *                   dst_1 & \ldots & dst_n
 *                 \end{bmatrix}@f$.
 * @param[in] inc_dst Increment of the @p dst array.
 */
void hddc2b_pltf_drv_algn_dst(
        int num_drv,
        const double *pos_drv,
        const double *w,
        const double *q_pvt,
        const double *f_pltf,
        double *dst,
        int inc_dst);


/**
 * The force composition matrix @f$\vect{G}(\vect{q}_{pvt}, \vec{\vect{p}}_d)@f$
 * depends on the pivot angles @f$\vect{q}_{pvt}@f$ as well as the drive
 * attachment vectors @f$\vec{\vect{p}}_d@f$ and encodes (i) the change of the
 * moment's action point; and (ii) the change of coordinates, in both cases from
 * the individual drives' pivot frames to the platform frame.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] pos_drv An array of @p num_drv position vectors
 *                    @f$\vec{\vect{p}}_d@f$ that represent the position
 *                    of each drive's attachment point with respect to the
 *                    platform's origin. The coordinates are expressed in the
 *                    platform frame. The array is arranged as @f[
 *                    \begin{bmatrix}
 *                      x_1 & \ldots & x_n \\
 *                      y_1 & \ldots & y_n
 *                    \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[in] q_pvt The array @f$\vect{q}_{pvt}@f$ of @p num_drv pivot angles
 *                  arranged as @f$
 *                  \begin{bmatrix}
 *                    q_1 & \ldots & q_n
 *                  \end{bmatrix}@f$.
 * @param[out] g The force composition matrix @f$\vect{G}@f$ with three rows and
 *               @f$2 \times {}@f$ @p num_drv columns, measured in the
 *               platform's origin and its coordinates expressed in the
 *               platform's frame. The matrix is arranged as @f[
 *               \begin{bmatrix}
 *                 \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                   & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                   & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                   & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                 \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                   & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                   & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                   & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                 \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                   & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                   & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                   & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *               \end{bmatrix}
 *               @f] and will be provided in column-major order. Here,
 *               @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the platform
 *               forces and torque, respectively. @f$f_{i,x}@f$ and
 *               @f$f_{i,y}@f$ are the drive's forces.
 */
void hddc2b_pltf_frc_comp_mat(
        int num_drv,
        const double *pos_drv,
        const double *q_pvt,
        double *g);


/**
 * Compute the platform-level force given drive-level forces. Since HDDC
 * platforms are parallel kinematic chains, this mapping is unique. It is solved
 * via the following formula:
 * @f[
 *   \vect{F}_p = \vect{G} \vect{F}_d
 * @f]
 *
 * The input drive forces _add_ to produce the desired platform force.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
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
 * @param[in] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                  columns where the rows represent the linear force components
 *                  that each drive exerts on the platform in the longitudinal
 *                  and transverse direction, respectively. The forces'
 *                  coordinates are expressed in the individual drives' pivot
 *                  frames. The matrix is arranged as @f[
 *                  \begin{bmatrix}
 *                    f_{1,x} & \ldots & f_{n,x} \\
 *                    f_{1,y} & \ldots & f_{n,y}
 *                  \end{bmatrix}
 *                  @f] and must be provided in column-major order.
 * @param[out] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                    represent force and torque applied to the platform due to
 *                    the drives' force contributions. The torque's reference
 *                    point is the platform's origin. The coordinates are
 *                    expressed in the platform frame. The vector is arranged as
 *                    @f$
 *                    \begin{bmatrix}
 *                      f_{p,x} & f_{p,y} & m_p
 *                    \end{bmatrix}@f$.
 */
void hddc2b_pltf_frc_pvt_to_pltf(
        int num_drv,
        const double *g,
        const double *f_drv,
        double *f_pltf);


/**
 * @f[
 *   \vect{Z}_p, \vect{\Lambda}_p
 *     &= \operatorname{dsyev}(\vect{W}_p) \\
 *   \vect{W}_p^{\frac{1}{2}}
 *     &= \vect{Z}_p \vect{\Lambda}_p^{\frac{1}{2}} \vect{Z}_p^T
 * @f]
 *
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
 *                   @f] where
 *                   @f$w_{p,yx} = w_{p,xy}@f$, @f$w_{p,mx} = w_{p,xm}@f$
 *                   and @f$w_{p,my} = w_{p,ym}@f$.
 * @param[out] w_pltf_sqrt The square root of the platform's weight matrix
 *                         @f$\vect{W}_p^{\frac{1}{2}}@f$, a positive-definite
 *                         @f$3 \times 3@f$ matrix. The weight is measured in
 *                         the platform's origin and its coordinates are
 *                         expressed in the platform frame. The matrix is
 *                         arranged as @f[
 *                         \begin{bmatrix}
 *                           w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                           w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                           w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                         \end{bmatrix}
 *                         @f] where
 *                         @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                         @f$w_{p,mx}' = w_{p,xm}'@f$
 *                         and @f$w_{p,my}' = w_{p,ym}'@f$.
 */
void hddc2b_pltf_frc_w_pltf_sqrt(
        const double *w_pltf,
        double *w_pltf_sqrt);


/**
 * @f[
 *   \vect{Z}_d, \vect{\Lambda}_d
 *     &= \operatorname{dsyev}(\vect{W}_d) \\
 *   \vect{W}_d^{-\frac{1}{2}}
 *     &= \vect{Z}_d \vect{\Lambda}_d^{-\frac{1}{2}} \vect{Z}_d^T
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
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
 * @param[out] w_drv_inv_sqrt An array that consists of @p num_drv
 *                            positive-definite @f$2 \times 2@f$ weight
 *                            matrices' square roots
 *                            @f$\vect{W}_d^{-\frac{1}{2}}@f$, each measured in
 *                            the respective drive's attachment point and its
 *                            coordinates expressed in the drive's attachment
 *                            frame. The array is arranged as @f[
 *                            \begin{bmatrix}
 *                              w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                              w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                              w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                              w_{1,yy}' & \ldots & w_{n,yy}'
 *                            \end{bmatrix}
 *                            @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                            provided in column-major order.
 */
void hddc2b_pltf_frc_w_drv_inv_sqrt(
        int num_drv,
        const double *w_drv,
        double *w_drv_inv_sqrt);


/**
 * @f[
 *   \vect{G}'   &= \vect{W}_p^{\frac{1}{2}} \vect{G} \\
 *   \vect{F}_p' &= \vect{W}_p^{\frac{1}{2}} \vect{F}_p
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g_in The force composition matrix @f$\vect{G}@f$ with three rows
 *                 and @f$2 \times {}@f$ @p num_drv columns, measured in the
 *                 platform's origin and its coordinates expressed in the
 *                 platform's frame. The matrix is arranged as @f[
 *                 \begin{bmatrix}
 *                   \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                   \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                   \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *                 \end{bmatrix}
 *                 @f] and must be provided in column-major order. Here,
 *                 @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the
 *                 platform forces and torque, respectively. @f$f_{i,x}@f$ and
 *                 @f$f_{i,y}@f$ are the drive's forces.
 * @param[in] f_pltf_in The vector @f$\vect{F}_p@f$ with three elements that
 *                      represent a force and torque on the platform. The
 *                      torque's reference point is the platform's origin. The
 *                      coordinates are expressed in the platform frame. The
 *                      vector is arranged as @f$
 *                      \begin{bmatrix}
 *                        f_{p,x} & f_{p,y} & m_p
 *                      \end{bmatrix}@f$.
 * @param[in] w_pltf_sqrt The square root of the platform's weight matrix
 *                        @f$\vect{W}_p^{\frac{1}{2}}@f$, a positive-definite
 *                        @f$3 \times 3@f$ matrix. The weight is measured in the
 *                        platform's origin and its coordinates are expressed in
 *                        the platform frame. The matrix is arranged as @f[
 *                        \begin{bmatrix}
 *                          w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                          w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                          w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                        \end{bmatrix}
 *                        @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                        @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                        @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[out] g_out The weighted force composition matrix @f$\vect{G}'@f$ with
 *                   three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                   matrix will be provided in column-major order.
 * @param[out] f_pltf_out The weighted force vector @f$\vect{F}_p'@f$ with three
 *                        elements.
 */
void hddc2b_pltf_frc_sing_wgh(
        int num_drv,
        const double *g_in,
        const double *f_pltf_in,
        const double *w_pltf_sqrt,
        double *g_out,
        double *f_pltf_out);


/**
 * @f[
 *   \vect{F}_p' = \vect{F}_p - \vect{G} \bar{\vect{F}}_d
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g (A version of) the force composition matrix @f$\vect{G}@f$ with
 *              three rows and @f$2 \times {}@f$ @p num_drv columns. The matrix
 *              must be provided in column-major order.
 * @param[in] f_pltf_in The vector @f$\vect{F}_p@f$ with three elements that
 *                      represent a force and torque on the platform. The
 *                      torque's reference point is the platform's origin. The
 *                      coordinates are expressed in the platform frame. The
 *                      vector is arranged as @f$
 *                      \begin{bmatrix}
 *                        f_{p,x} & f_{p,y} & m_p
 *                      \end{bmatrix}@f$.
 * @param[in] f_drv_ref The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      force reference components in the longitudinal and
 *                      transverse direction, respectively, that are projected
 *                      into the nullspace of the platform task. The forces'
 *                      coordinates are expressed in the individual drives'
 *                      pivot frames. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and will be provided in column-major order.
 * @param[out] f_pltf_out The force vector @f$\vect{F}_p'@f$ with three
 *                        elements.
 */
void hddc2b_pltf_frc_redu_ref_init(
        int num_drv,
        const double *g,
        const double *f_pltf_in,
        const double *f_drv_ref,
        double *f_pltf_out);


/**
 * @f[
 *   \vect{G}' = \vect{G} \vect{W}_d^{-\frac{1}{2}}
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g_in (A version of) the force composition matrix @f$\vect{G}@f$
 *                 with three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                 matrix must be provided in column-major order.
 * @param[in] w_drv_inv_sqrt An array that consists of @p num_drv
 *                           positive-definite @f$2 \times 2@f$ weight matrices'
 *                           square roots @f$\vect{W}_d^{-\frac{1}{2}}@f$, each
 *                           measured in the respective drive's attachment point
 *                           and its coordinates expressed in the drive's
 *                           attachment frame. The array is arranged as @f[
 *                           \begin{bmatrix}
 *                             w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                             w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                             w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                             w_{1,yy}' & \ldots & w_{n,yy}'
 *                           \end{bmatrix}
 *                           @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and must be
 *                           provided in column-major order.
 * @param[out] g_out The weighted force composition matrix @f$\vect{G}'@f$ with
 *                   three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                   matrix will be provided in column-major order.
 */
void hddc2b_pltf_frc_redu_wgh_init(
        int num_drv,
        const double *g_in,
        const double *w_drv_inv_sqrt,
        double *g_out);


/**
 * @f[
 *   \vect{F}_d = \vect{W}_d^{-\frac{1}{2}} \vect{F}_d'
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] f_drv_in The matrix @f$\vect{F}_d'@f$ with two rows and @p num_drv
 *                     columns where the rows represent the linear force
 *                     components that each drive exerts on the platform in the
 *                     longitudinal and transverse direction, respectively. The
 *                     forces' coordinates are expressed in the individual
 *                     drives' pivot frames. The matrix is arranged as @f[
 *                     \begin{bmatrix}
 *                       f_{1,x}' & \ldots & f_{n,x}' \\
 *                       f_{1,y}' & \ldots & f_{n,y}'
 *                     \end{bmatrix}
 *                     @f] and must be provided in column-major order.
 * @param[in] w_drv_inv_sqrt An array that consists of @p num_drv
 *                           positive-definite @f$2 \times 2@f$ weight matrices'
 *                           inverse square roots
 *                           @f$\vect{W}_d^{-\frac{1}{2}}@f$, each measured in
 *                           the respective drive's attachment point and its
 *                           coordinates expressed in the drive's attachment
 *                           frame. The array is arranged as @f[
 *                           \begin{bmatrix}
 *                             w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                             w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                             w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                             w_{1,yy}' & \ldots & w_{n,yy}'
 *                           \end{bmatrix}
 *                           @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and must be
 *                           provided in column-major order.
 * @param[out] f_drv_out The weighted matrix @f$\vect{F}_d@f$ with two rows and
 *                       @p num_drv columns. The matrix is arranged as @f[
 *                       \begin{bmatrix}
 *                         f_{1,x} & \ldots & f_{n,x} \\
 *                         f_{1,y} & \ldots & f_{n,y}
 *                       \end{bmatrix}
 *                       @f] and will be provided in column-major order.
 */
void hddc2b_pltf_frc_redu_wgh_fini(
        int num_drv,
        const double *f_drv_in,
        const double *w_drv_inv_sqrt,
        double *f_drv_out);


/**
 * @f[
 *   \vect{F}_d = \bar{\vect{F}}_d + \vect{F}_d'
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] f_drv_in1 The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
 * @param[in] f_drv_in2 The matrix @f$\vect{F}_d'@f$ with two rows and
 *                      @p num_drv columns. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        f_{1,x}' & \ldots & f_{n,x}' \\
 *                        f_{1,y}' & \ldots & f_{n,y}'
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
 * @param[out] f_drv_out The matrix @f$\vect{F}_d@f$ with two rows and
 *                       @p num_drv columns that represents the sum of both
 *                       inputs. The matrix is arranged as @f[
 *                       \begin{bmatrix}
 *                         f_{1,x} & \ldots & f_{n,x} \\
 *                         f_{1,y} & \ldots & f_{n,y}
 *                       \end{bmatrix}
 *                       @f] and will be provided in column-major order.
 */
void hddc2b_pltf_frc_redu_ref_fini(
        int num_drv,
        const double *f_drv_in1,
        const double *f_drv_in2,
        double *f_drv_out);


/**
 * @f[
 *   \vect{F}_d = \vect{V} \vect{S}^{-1} \vect{U}^T \vect{F}_p
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] u The SVD's @f$\vect{U}@f$ matrix with three rows and three
 *              columns.
 * @param[in] s_inv A vector with some inverse of the three eigenvalues.
 * @param[in] vt The SVD's @f$\vect{V}^T@f$ matrix with
 *               @f$2 \times {}@f$ @p num_drv rows and three columns.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a (version of a) force and torque on the
 *                   platform. The torque's reference point is the platform's
 *                   origin. The coordinates are expressed in the platform
 *                   frame. The vector is arranged as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform in the
 *                   longitudinal and transverse direction, respectively. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_pltf_frc_slv(
        int num_drv,
        const double *u,
        const double *s_inv,
        const double *vt,
        const double *f_pltf,
        double *f_drv);




/**
 * @f[
 *   \vect{Z}_p, \vect{\Lambda}_p
 *     &= \operatorname{dsyev}(\vect{W}_p) \\
 *   \vect{W}_p^{-\frac{1}{2}}
 *     &= \vect{Z}_p \vect{\Lambda}_p^{-\frac{1}{2}} \vect{Z}_p^T
 * @f]
 *
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
 *                   @f] where
 *                   @f$w_{p,yx} = w_{p,xy}@f$, @f$w_{p,mx} = w_{p,xm}@f$
 *                   and @f$w_{p,my} = w_{p,ym}@f$.
 * @param[out] w_pltf_inv_sqrt The inverse square root of the platform's weight
 *                             matrix @f$\vect{W}_p^{-\frac{1}{2}}@f$, a
 *                             positive-definite @f$3 \times 3@f$ matrix. The
 *                             weight is measured in the platform's origin and
 *                             its coordinates are expressed in the platform
 *                             frame. The matrix is arranged as @f[
 *                             \begin{bmatrix}
 *                               w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                               w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                               w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                             \end{bmatrix}
 *                             @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                             @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                             @f$w_{p,my}' = w_{p,ym}'@f$.
 */
void hddc2b_pltf_vel_w_pltf_inv_sqrt(
        const double *w_pltf,
        double *w_pltf_inv_sqrt);


/**
 * @f[
 *   \vect{Z}_d, \vect{\Lambda}_d
 *     &= \operatorname{dsyev}(\vect{W}_d) \\
 *   \vect{W}_d^{\frac{1}{2}}
 *     &= \vect{U}_d \vect{S}_d^{\frac{1}{2}} \vect{U}_d^T
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
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
 * @param[out] w_drv_sqrt An array that consists of @p num_drv positive-definite
 *                        @f$2 \times 2@f$ weight matrices' square roots
 *                        @f$\vect{W}_d^{\frac{1}{2}}@f$, each measured in the
 *                        respective drive's attachment point and its
 *                        coordinates expressed in the drive's attachment frame.
 *                        The array is arranged as @f[
 *                        \begin{bmatrix}
 *                          w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                          w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                          w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                          w_{1,yy}' & \ldots & w_{n,yy}'
 *                        \end{bmatrix}
 *                        @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                        provided in column-major order.
 */
void hddc2b_pltf_vel_w_drv_sqrt(
        int num_drv,
        const double *w_drv,
        double *w_drv_sqrt);


/**
 * @f[
 *   \vect{G}'         &= \vect{G} \vect{W}_d^{\frac{1}{2}}\\
 *   \dot{\vect{X}}_d' &= \vect{W}_d^{\frac{1}{2}} \dot{\vect{X}}_d
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g_in The force composition matrix @f$\vect{G}@f$ with three rows
 *                 and @f$2 \times {}@f$ @p num_drv columns, measured in the
 *                 platform's origin and its coordinates expressed in the
 *                 platform's frame. The matrix is arranged as @f[
 *                 \begin{bmatrix}
 *                   \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                   \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                   \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                     & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *                 \end{bmatrix}
 *                 @f] and must be provided in column-major order. Here,
 *                 @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the
 *                 platform forces and torque, respectively. @f$f_{i,x}@f$ and
 *                 @f$f_{i,y}@f$ are the drive's forces.
 * @param[in] xd_drv_in The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      velocity components of the drive's attachment point (to
 *                      the platform) in the longitudinal and transverse
 *                      direction, respectively. The linear velocities'
 *                      reference point is the origin of the respective pivot
 *                      frames. Their coordinates are expressed in these pivot
 *                      frames. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                        \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
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
 *                       @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and must be
 *                       provided in column-major order.
 * @param[out] g_out The weighted force composition matrix @f$\vect{G}'@f$ with
 *                   three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                   matrix will be provided in column-major order.
 * @param[out] xd_drv_out The weighted drive velocity matrix
 *                        @f$\dot{\vect{X}}_d'@f$ with two rows and @p num_drv
 *                        columns that will be provided in column-major order.
 */
void hddc2b_pltf_vel_sing_wgh(
        int num_drv,
        const double *g_in,
        const double *xd_drv_in,
        const double *w_drv_sqrt,
        double *g_out,
        double *xd_drv_out);


/**
 * @f[
 *   \dot{\vect{X}}_d' = \dot{\vect{X}}_d - \vect{G}^T \bar{\dot{\vect{X}}}_p
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g (A version of) the force composition matrix @f$\vect{G}@f$ with
 *              three rows and @f$2 \times {}@f$ @p num_drv columns. The matrix
 *              must be provided in column-major order.
 * @param[in] xd_drv_in The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent (a version
 *                      of) the linear velocity components of the drive's
 *                      attachment point (to the platform) in the longitudinal
 *                      and transverse direction, respectively. The linear
 *                      velocities' reference point is the origin of the
 *                      respective pivot frames. Their coordinates are expressed
 *                      in these pivot frames. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                        \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
 * @param[in] xd_pltf_ref The vector @f$\bar{\dot{\vect{X}}_p}@f$ with three
 *                        elements that represent the platform's linear and
 *                        angular reference velocity that are projected into the
 *                        nullspace of the drive task. The linear velocity's
 *                        reference point is the platform frame's origin. The
 *                        coordinates are expressed in the platform frame. The
 *                        vector is arranged as @f$
 *                        \begin{bmatrix}
 *                          \bar{\dot{X}}_{p,x} &
 *                          \bar{\dot{X}}_{p,y} &
 *                          \bar{\omega}_p
 *                        \end{bmatrix}@f$.
 * @param[out] xd_drv_out The weighted drive velocity matrix
 *                        @f$\dot{\vect{X}}_d'@f$ with two rows and @p num_drv
 *                        columns that will be provided in column-major order.
 */
void hddc2b_pltf_vel_redu_ref_init(
        int num_drv,
        const double *g,
        const double *xd_drv_in,
        const double *xd_pltf_ref,
        double *xd_drv_out);


/**
 * @f[
 *   \vect{G}' = \vect{W}_p^{-\frac{1}{2}} \vect{G}
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g_in (A version of) the force composition matrix @f$\vect{G}@f$
 *                 with three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                 matrix must be provided in column-major order.
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
 * @param[out] g_out The weighted force composition matrix @f$\vect{G}'@f$ with
 *                   three rows and @f$2 \times {}@f$ @p num_drv columns. The
 *                   matrix will be provided in column-major order.
 */
void hddc2b_pltf_vel_redu_wgh_init(
        int num_drv,
        const double *g_in,
        const double *w_pltf_inv_sqrt,
        double *g_out);


/**
 * @f[
 *   \dot{\vect{X}}_p = \vect{W}_p^{-\frac{1}{2}} \dot{\vect{X}}_p'
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[out] xd_pltf_in The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                        that represent (a version of) the platform's linear
 *                        and angular velocity. The linear velocity's reference
 *                        point is the platform frame's origin. The coordinates
 *                        are expressed in the platform frame. The vector is
 *                        arranged as @f$
 *                        \begin{bmatrix}
 *                          \dot{X}_{p,x}' & \dot{X}_{p,y}' & \omega_p'
 *                        \end{bmatrix}@f$.
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
 * @param[out] xd_pltf_out The vector @f$\dot{\vect{X}}_p@f$ with three
 *                         elements. The vector is arranged as @f$
 *                         \begin{bmatrix}
 *                           \dot{X}_{p,x} & \dot{X}_{p,y} & \omega_p
 *                         \end{bmatrix}@f$.
 */
void hddc2b_pltf_vel_redu_wgh_fini(
        int num_drv,
        const double *xd_pltf_in,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf_out);


/**
 * @f[
 *   \dot{\vect{X}}_p = \bar{\dot{\vect{X}}}_p + \dot{\vect{X}}_p'
 * @f]
 *
 * @param[in] xd_pltf_in1 The vector @f$\bar{\dot{\vect{X}}}_p@f$ with three
 *                        elements. The vector is arranged as @f$
 *                        \begin{bmatrix}
 *                          \bar{\dot{X}}_x & \bar{\dot{X}}_y & \bar{\omega}
 *                        \end{bmatrix}@f$.
 * @param[in] xd_pltf_in2 The vector @f$\dot{\vect{X}}_p'@f$ with three
 *                        elements. The vector is arranged as @f$
 *                        \begin{bmatrix}
 *                          \dot{X}_x' & \dot{X}_y' & \omega'
 *                        \end{bmatrix}@f$.
 * @param[out] xd_pltf_out The vector @f$\dot{\vect{X}}_p@f$ with three
 *                        elements. The vector is arranged as @f$
 *                        \begin{bmatrix}
 *                          \dot{X}_x & \dot{X}_y & \omega
 *                        \end{bmatrix}@f$.
 */
void hddc2b_pltf_vel_redu_ref_fini(
        const double *xd_pltf_in1,
        const double *xd_pltf_in2,
        double *xd_pltf_out);


/**
 * Note that the decomposition is of the @f$\vect{G}@f$ matrix, but we need to
 * work with the @f$\vect{G}^T@f$ matrix. Hence, the SVD solving step may look
 * "reversed".
 *
 * @f[
 *   \vect{U}, \vect{S}, \vect{V}^T = \operatorname{desvg}(\vect{G})
 *     &\Leftrightarrow
 *     \vect{V}, \vect{S}, \vect{U}^T = \operatorname{desvg}(\vect{G}^T) \\
 *   &\Rightarrow
 *     \dot{\vect{X}}_p = \vect{U} \vect{S}^{-1} \vect{V}^T \dot{\vect{X}}_d
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] u The SVD's @f$\vect{U}@f$ matrix with three rows and three
 *              columns.
 * @param[in] s_inv A vector with some inverse of the three eigenvalues.
 * @param[in] vt The SVD's @f$\vect{V}^T@f$ matrix with
 *               @f$2 \times {}@f$ @p num_drv rows and three columns.
 * @param[in] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                   @p num_drv columns where the rows represent the linear
 *                   velocity components of the drive's attachment point (to the
 *                   platform) in the longitudinal and transverse direction,
 *                   respectively. The linear velocities' reference point is the
 *                   origin of the respective pivot frames. Their coordinates
 *                   are expressed in these pivot frames. The matrix is arranged
 *                   as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 * @param[out] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                     that represent (a version of) the platform's linear and
 *                     angular velocity. The linear velocity's reference point
 *                     is the platform frame's origin. The coordinates are
 *                     expressed in the platform frame. The vector is arranged
 *                     as @f$
 *                     \begin{bmatrix}
 *                       \dot{X}_x & \dot{X}_y & \omega
 *                     \end{bmatrix}@f$.
 */
void hddc2b_pltf_vel_slv(
        int num_drv,
        const double *u,
        const double *s_inv,
        const double *vt,
        const double *xd_drv,
        double *xd_pltf);




/**
 * Compute a singular value decomposition (SVD) of a force composition matrix
 * @f$\vect{G}@f$.
 *
 * @f[
 *   \vect{U}, \vect{S}, \vect{V}^T = \operatorname{dgesvd}(\vect{G})
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] g (A version of) the force composition matrix @f$\vect{G}@f$ with
 *              three rows and @f$2 \times {}@f$ @p num_drv columns. The matrix
 *              must be provided in column-major order.
 * @param[out] u The SVD's @f$\vect{U}@f$ matrix with three rows and three
 *               columns.
 * @param[out] s The SVD's @f$\vect{S}@f$ vector of eigenvalues with three
 *               elements. The vector entries will be sorted in descending
 *               order.
 * @param[out] vt The SVD's @f$\vect{V}^T@f$ matrix with
 *                @f$2 \times {}@f$ @p num_drv rows and three columns.
 */
void hddc2b_pltf_dcmp(
        int num_drv,
        const double *g,
        double *u,
        double *s,
        double *vt);


/**
 * Compute a pseudoinverse for each (eigen)value in a vector. The pseudoinverse
 * is defined as:
 *
 * @f[
 *   S_i^{-1} =
 *     \begin{cases}
 *       0             & \text{if } S < \epsilon \\
 *       \frac{1}{S_i} & \text{else}
 *     \end{cases}
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] s A vector of three eigenvalues that are either zero or positive.
 * @param[out] s_inv The inverse of the three eigenvalues.
 */
void hddc2b_pltf_pinv(
        int num_drv,
        double eps,
        const double *s,
        double *s_inv);


/**
 * Compute a damped inverse for a vector of eigenvalues. The damping factor
 * @f$\lambda@f$ is only enabled in the vicinity (as measured by @f$\epsilon@f$)
 * of a singularity according to @cite{Chiaverini1991}:
 *
 * @f[
 *   \lambda_s &=
 *     \begin{cases}
 *       0
 *         & \text{if } S_{min} \ge \epsilon \\
 *       \lambda \sqrt{1 - \left(\frac{S_{min}}{\epsilon}\right)^2}
 *         & \text{else}
 *     \end{cases} \\
 *   S_i^{-1} &= \frac{S_i}{S_i^2 + \lambda_s^2}
 * @f]
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to activate the
 *                damping.
 * @param[in] lambda The scalar @f$\lambda@f$ that determines the "amount" of
 *                   damping.
 * @param[in] s A vector of three eigenvalues that are either zero or positive.
 *              The vector entries must be sorted in descending order.
 * @param[out] s_inv A vector with the inverse of the three eigenvalues.
 */
void hddc2b_pltf_dmp(
        int num_drv,
        double eps,
        double lambda,
        const double *s,
        double *s_inv);


#ifdef __cplusplus
}
#endif

#endif
