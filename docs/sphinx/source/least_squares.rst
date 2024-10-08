Pseudoinverse solvers for least squares problems (in robotics)
**************************************************************

Quadratic Programming
=====================

A wide range of optimization problems can be cast into the following shape of a quadratic program.

.. math::
    :nowrap:

    \begin{align}
        & \min_{\vect{x}}
        & & f(\vect{x}) = \frac{1}{2} \sum_{i=1}^{N_o} \|\vect{A}_i \vect{x} - \vect{b}_i\|_{\vect{W}_i}^2 & \\
        & \text{subject to}
        & & \vect{A}_j \vect{x} - \vect{b}_j = \vect{0} & \text{for } j = 1, \ldots, N_{e} \\
        & & & \vect{A}_k \vect{x} - \vect{b}_k \le \vect{0} & \text{for } k = 1, \ldots, N_{i}
    \end{align}

This generic and declarative formulation allows for many types of solvers that are based on *policies* of such as:

* Choice of constants:

  * Some :math:`\vect{A}` may be identity matrices
  * Some :math:`\vect{b}` may be zero vectors
  * Some :math:`\vect{W}` may be identity matrices

* Weight (soft objective) vs. prioritization (hard constraint)

  * For example, represent *physical* system as objective (cf. Stack of Tasks) vs. system as constraint (cf. Gauss principle of least constraint)
  * Multi-level prioritization *hierarchy* (via nullspace projections)

* Number and type of constraints:

  * No equality constraints, no inequality constraints :math:`\rightarrow` analytic solver (unconstrained optimization)
  * Some equality constraints, no inequality constraints :math:`\rightarrow` analytic solver (via Lagrange multipliers to unconstrained optimization)
  * Some inequality constraints :math:`\rightarrow` iterative/numerical solver

* Damping or regularization
* Implementation of matrix decomposition:

  * Choice of solver: LU, Cholesky, QR, SVD, ...
  * Solver-specific choices, e.g. "type of SVD" (Householder reflection vs. Givens rotation), or maximum number of iterations in SVD

Here, we focus on a special family of solvers that afford an analytical solution relying an matrix pseudoinverses.




Variations of pseudoinverse solvers
===================================

We classify the solution approaches along the number of equations and variables in the system of equations.
Then the system can be *overdetermined* (more equations than variables) or *underdetermined* (less equations than variables).
The case where there are the same amount of equations as variables is not discussed here.
Most of the below information can be found in and is based on [BenIsrael2003]_.
A discussion is also available `here <https://math.stackexchange.com/questions/1537880/what-forms-does-the-moore-penrose-inverse-take-under-systems-with-full-rank-ful/2200203#2200203>`__.



Overdetermined systems
----------------------

* The system has *more* equations than unknowns
* :math:`\vect{A}` has full *column* rank (is row-rank deficient)
* There is *no* (exact) solution
* We can find an *approximate* solution (w.r.t. a :math:`\vect{W}`-norm in the ":math:`f(\vect{x})`-space")
* The system enters as an *objective*
* Solve via :math:`\{\vect{W}\}`-weighted, *left* pseudoinverse
* Robotic example: a 5-DoF manipulator performing a 6-DoF Cartesian task or a Kelo platform with a single wheel unit

  * The manipulator is *always* **singular** for that task
  * The ":math:`f(\vect{x})`-space" is the Cartesian velocity-twist space
  * :math:`\vect{W}` is a weight matrix in Cartesian velocity-twist space

(Ordinary) Least squares
^^^^^^^^^^^^^^^^^^^^^^^^

The (ordinary) least squares is only provided for completeness and as a "baseline" for reducing other problems to.
For many physical systems, for example with linear *and* angular quantities, this is not a correct choice because it leads to inconsistent physical units.
The "correct" solution is to introduce (relative) weights for linear and angular quantities.

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}} f(\vect{x}) = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|^2 = \frac{1}{2} (\vect{A} \vect{x} - \vect{b})^T (\vect{A} \vect{x} - \vect{b})
    \end{equation}

Solve the minimization by *(i)* equating the derivative to the zero vector; and *(ii)* solving for :math:`\vect{x}`:

.. math::
    :nowrap:

    \begin{align}
        (\vect{A} \vect{x} - \vect{b})^T &= \vect{0} \\
        \vect{A} \vect{x} - \vect{b} &= \vect{0} \\
        \vect{A}^T (\vect{A} \vect{x} - \vect{b}) &= \vect{0} \\
        \vect{A}^T \vect{A} \vect{x} - \vect{A}^T \vect{b} &= \vect{0} \\
        (\vect{A}^T \vect{A}) \vect{x} &= \vect{A}^T \vect{b} \\
        \vect{x} &= (\vect{A}^T \vect{A})^{-1} \vect{A}^T \vect{b} \\
        \vect{x} &= \vect{A}^\dagger \vect{b}
    \end{align}

where :math:`\vect{A}^\dagger = (\vect{A}^T \vect{A})^{-1} \vect{A}^T` is the Moore-Penrose pseudoinverse (a *left* inverse).


.. _sec:app:over:wlsq:

Weighted least squares
^^^^^^^^^^^^^^^^^^^^^^

Here, we assume an arbitrary, positive-definite weight matrix :math:`\vect{W}` that acts on the ":math:`\vect{b}`-space".
This problem is sometimes also referred to as *generalized* least squares and the term *weighted* least squares is reserved for problems with diagonal weight matrices.

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}} f(\vect{x}) = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2 = (\vect{A} \vect{x} - \vect{b})^T \vect{W} (\vect{A} \vect{x} - \vect{b})
    \end{equation}

Solve similar to previous cases:

.. math::
    :nowrap:

    \begin{align}
        \vect{A}^T \vect{W} (\vect{A} \vect{x} - \vect{b}) &= \vect{0} \\
        \vect{A}^T \vect{W} \vect{A} \vect{x} - \vect{A}^T \vect{W} \vect{b} &= \vect{0} \\
        (\vect{A}^T \vect{W} \vect{A}) \vect{x} &= \vect{A}^T \vect{W} \vect{b} \\
        \vect{x} &= (\vect{A}^T \vect{W} \vect{A})^{-1} \vect{A}^T \vect{W} \vect{b} \\
        \vect{x} &= \vect{A}^\dagger_{\vect{W}} \vect{b}
    \end{align}

where :math:`\vect{A}^\dagger_{\vect{W}} = (\vect{A}^T \vect{W} \vect{A})^{-1} \vect{A}^T \vect{W}` is the :math:`\{\vect{W}\}`-weighted Moore-Penrose pseudoinverse (a left inverse).



Underdetermined systems
-----------------------

* The system has *fewer* equations than unknowns
* :math:`\vect{A}` has full *row* rank (is column-rank deficient)
* There are (infinitely) *many* solutions
* We can find a *minimum-norm* solution among all existing solutions (w.r.t. a :math:`\vect{Q}`-norm in the ":math:`\vect{x}`-space")
* The system enters as a *constraint*
* Solve via :math:`\{\vect{Q}^{-1}\}`-weighted, *right* pseudoinverse
* Robotic example: a 7-DoF manipulator performing a 6-DoF Cartesian task or a Kelo platform with more than one wheel unit

  * The manipulator is *sometimes* **redundant** for that task
  * The ":math:`\vect{x}`-space" is the joint-velocity space
  * :math:`\vect{Q}` is a weight matrix in joint-velocity space


Minimum-norm least squares or least norm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Among the set of Eq. :math:`\eqref{eq:lsq-set}` we may be interested in a specific solution, that of *minimum norm*, i.e. the solution where :math:`\vect{x}(\hat{\vect{x}})` is "shortest".
This can be represented by a *cascaded* or *staged* optimization problem:

.. math::
    :nowrap:

    \begin{align}
        &\text{Stage 1: }
        &\vect{x}(\hat{\vect{x}})
        &= \arg \min_{\vect{x}} f(\vect{x})
         = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|^2 \\
        &\text{Stage 2: }
        &\hat{\vect{x}}^*
        &= \arg \min_{\hat{\vect{x}}} g(\hat{\vect{x}})
         = \frac{1}{2} \|\vect{x}(\hat{\vect{x}})\|^2
    \end{align}

.. todo::
    Is there a better notation for this?


Equivalently, this is represented by the following problem:

.. math::
    :nowrap:

    \begin{equation}
        \begin{aligned}
            & \min_{\vect{x}}
            & & f(\vect{x}) = \frac{1}{2} \|\vect{x}\|^2 \\
            & \text{subject to}
            & & \vect{A} \vect{x} - \vect{b} = \vect{0}
        \end{aligned}
        \label{eq:lsq-min-norm-prob}
    \end{equation}

Construct an unconstrained optimization problem using the Lagrange multiplier :math:`\vect{\lambda}` to embed the constraint into the augmented objective (also known as Lagrangian):

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}, \vect{\lambda}} f_{aug}(\vect{x})
        = \frac{1}{2} \vect{x}^T \vect{x} + \vect{\lambda}^T (\vect{A} \vect{x} - \vect{b})
    \end{equation}

Solve the minimization by *(i)* equating the derivative to the zero vector; *(ii)* substituting the constraint equation; and *(iii)* solving for the Lagrange multiplier:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}^T + \vect{\lambda}^T \vect{A} &= \vect{0}\label{eq:lsq-min-norm-deriv} \\
        \vect{\lambda}^T \vect{A} &= -\vect{x}^T \\
        \vect{\lambda}^T \vect{A} \vect{A}^T &= -\vect{x}^T \vect{A}^T \\
        \vect{\lambda}^T \vect{A} \vect{A}^T &= -\vect{b}^T \\
        \vect{\lambda}^T &= -\vect{b}^T (\vect{A} \vect{A}^T)^{-1}
    \end{align}

Substitute the Lagrange multiplier into Eq. :math:`\eqref{eq:lsq-min-norm-deriv}` and solve for :math:`\vect{x}`:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}^T - \vect{b}^T (\vect{A} \vect{A}^T)^{-1} \vect{A} &= \vect{0} \\
        \vect{x}^T &= \vect{b}^T (\vect{A} \vect{A}^T)^{-1} \vect{A} \\
        \vect{x} &= \vect{A}^{\dagger} \vect{b}\label{eq:lsq-min-norm}
    \end{align}

where :math:`\vect{A}^\dagger = \vect{A}^T (\vect{A} \vect{A}^T)^{-1}` is the Moore-Penrose pseudoinverse (a *right* inverse).
This is the (unique) minimum-norm least-squares solution.
However, there exists a family/set of solutions that satisfy

.. math::
    :nowrap:

    \begin{equation}
        \vect{x}(\bar{\vect{x}}) = \vect{A}^\dagger \vect{b} + \vect{N} \bar{\vect{x}}\label{eq:lsq-set}
    \end{equation}

where :math:`\vect{N} = \vect{I} - \vect{A} \vect{A}^{\dagger}` is an orthogonal nullspace projector and :math:`\bar{\vect{x}}` an arbitrary vector of appropriate dimension.


Minimum-norm weighted least squares
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The underlying optimization problem is formulated as

.. math::
    :nowrap:

    \begin{equation}
        \begin{aligned}
            & \min_{\vect{x}}
            & & f(\vect{x}) = \frac{1}{2} \|\vect{x}\|^2_{\vect{Q}} \\
            & \text{subject to}
            & & \vect{A} \vect{x} - \vect{b} = \vect{0}
        \end{aligned}
    \end{equation}

In analogy to Eq. :math:`\eqref{eq:lsq-min-norm}` the *minimum-norm* weighted least-squares solution for a rank-deficient matrix :math:`\vect{A}` is

.. math::
    :nowrap:
    :label: eq:wlsq-min-norm

    \begin{equation}
        \vect{x} = \vect{A}_{\vect{Q}^{-1}}^\dagger \vect{b}
    \end{equation}

Just like before, this is the minimum-norm solution but there exists an infinite amount of solutions of the form

.. math::
    :nowrap:

    \begin{equation}
        \vect{x}(\bar{\vect{x}}) = \vect{A}^\dagger_{\vect{Q}^{-1}} \vect{b} + \vect{N}_{\vect{Q}^{-1}} \bar{\vect{x}}
    \end{equation}

where :math:`\vect{N}_{\vect{Q}^{-1}} = \vect{I} - \vect{A} \vect{A}^\dagger_{\vect{Q}^{-1}}` is an oblique nullspace projector and :math:`\bar{\vect{x}}` is any vector of appropriate dimension.


.. _sec:app:under:min-ref-wls:

Minimum-reference weighted least squares
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This problem is an extension of the minimum-norm weighted least-squares problem in that it minimizes with respect to a potentially non-zero reference state.

.. math::
    :nowrap:

    \begin{equation}
        \begin{aligned}
            & \min_{\vect{x}}
            & & f(\vect{x}) = \frac{1}{2} \|\vect{x} - \bar{\vect{x}}\|_{\vect{Q}}^2 \\
            & \text{subject to}
            & & \vect{A} \vect{x} - \vect{b} = \vect{0}
        \end{aligned}
    \end{equation}

The augmented, non-constrained problem is analog to before:

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}, \vect{\lambda}} f_{aug}(\vect{x})
        = \frac{1}{2} (\vect{x} - \bar{\vect{x}})^T \vect{Q} (\vect{x} - \bar{\vect{x}}) + \vect{\lambda}^T (\vect{A} \vect{x} - \vect{b})
    \end{equation}

So is the solution for the Lagrange multiplier:

.. math::
    :nowrap:

    \begin{align}
        (\vect{x} - \bar{\vect{x}})^T \vect{Q} + \vect{\lambda}^T \vect{A} &= \vect{0}\label{eq:lsq-ref-deriv} \\
        (\vect{x} - \bar{\vect{x}})^T \vect{Q} &= -\vect{\lambda}^T \vect{A} \\
        (\vect{x} - \bar{\vect{x}})^T \vect{Q} \vect{Q}^{-1} \vect{A}^T &= -\vect{\lambda}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T) \\
        (\vect{x} - \bar{\vect{x}})^T \vect{A}^T &= -\vect{\lambda}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T) \\
        \vect{x}^T \vect{A}^T - \bar{\vect{x}}^T \vect{A}^T &= -\vect{\lambda}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T) \\
        \vect{b}^T - \bar{\vect{x}}^T \vect{A}^T &= -\vect{\lambda}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T) \\
        \vect{\lambda}^T &= (\vect{b}^T - \bar{\vect{x}}^T \vect{A}^T) (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1}
    \end{align}

Substitute the last equation into Eq. :math:`\eqref{eq:lsq-ref-deriv}` (with :math:`\vect{Q}` positive definite, hence :math:`\vect{Q}^T = \vect{Q}`):

.. math::
    :nowrap:

    \begin{align}
        (\vect{x} - \bar{\vect{x}})^T \vect{Q} + [(\vect{b}^T - \bar{\vect{x}}^T \vect{A}^T) (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1}] \vect{A} = \vect{0} \\
        \vect{x}^T - \bar{\vect{x}}^T = -[(\vect{b}^T - \bar{\vect{x}}^T \vect{A}^T) (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1}] \vect{A} \vect{Q}^{-1} \\
        \vect{x} = -\vect{Q}^{-1} \vect{A}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1} (\vect{A} \bar{\vect{x}} - \vect{b}) + \bar{\vect{x}}
    \end{align}

with weighted pseudoinverse (a right inverse) :math:`\vect{A}^{\dagger}_{\vect{Q}^{-1}} = \vect{Q}^{-1} \vect{A}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1}`:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}
        &= \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{b} - \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{A} \bar{\vect{x}} + \bar{\vect{x}} \\
        &= \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{b} + (\vect{I} - \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{A}) \bar{\vect{x}} \\
        &= \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{b} + \vect{N}_{\vect{Q}^{-1}} \bar{\vect{x}}
    \end{align}

where :math:`\vect{N}_{\vect{Q}^{-1}} = \vect{I} - \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{A}` is an oblique nullspace projector.
Note, how this generalized formulation and the derivation of its solution provides a better insight into the origin of the family/set of solutions in Eq. :math:`\eqref{eq:lsq-set}`.

The nullspace projector annihilates any vector in the nullspace (with :math:`\vect{A} \vect{A}^{\dagger}_{\vect{Q}^{-1}} = \vect{I}`):

.. math::
    :nowrap:

    \begin{align}
        \vect{N}_{\vect{Q}^{-1}}
        &= \vect{I} - \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{A} \\
        \vect{A} \vect{N}_{\vect{Q}^{-1}}
        &= \vect{A} \vect{I} - \vect{A} \vect{A}^{\dagger}_{\vect{Q}^{-1}} \vect{A} \\
        &= \vect{A} \vect{I} - \vect{I} \vect{A} \\
        &= \vect{A} - \vect{A} \\
        &= \vect{0}
    \end{align}



Generalization to handle both cases in one formulation
------------------------------------------------------

Weighted-minimum norm weighted least squares
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here, we are interested in a setting *(i)* of weighted-least squares (analogous to :ref:`weighted least squares<sec:app:over:wlsq>`) with the positive-definite matrix :math:`\vect{W}` as the norm in the space where vector :math:`\vect{b}` lives; *(ii)* the matrix :math:`\vect{A}` is not of full rank; and *(iii)* the norm in the space where vector :math:`\vect{x}` lives is given by a positive-definite matrix :math:`\vect{Q}`.

This problem can be represented by the following *cascaded* minimization over, first, the system and, second, the state:

.. math::
    :nowrap:

    \begin{align}
        &\text{Stage 1: }
        &\vect{x}(\hat{\vect{x}})
        &= \arg \min_{\vect{x}} f(\vect{x})
         = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2 \\
        &\text{Stage 2: }
        &\hat{\vect{x}}^*
        &= \arg \min_{\hat{\vect{x}}} g(\hat{\vect{x}})
         = \frac{1}{2} \|\vect{x}(\hat{\vect{x}})\|_{\vect{Q}}^2
    \end{align}

There does not seem to exist a formulation that is equivalent to Eq. :math:`\eqref{eq:lsq-min-norm-prob}`.
Instead, the solution relies on reducing this problem to Eq. :math:`\eqref{eq:lsq-min-norm}` via the following transformations according to [BenIsrael2003]_ (p. 118, Eq. 51):

* :math:`\vect{A}' = \vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}}`
* :math:`\vect{x}' = \vect{Q}^{\frac{1}{2}} \vect{x}`
* :math:`\vect{b}' = \vect{W}^{\frac{1}{2}} \vect{b}`

The following equations show that these transformations do not change the result (i.e. the transformations are "valid"):

.. math::
    :nowrap:

    \begin{align}
        \|\vect{A}' \vect{x}' - \vect{b}'\|^2
        &= \|
            \vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}}
            \vect{Q}^{\frac{1}{2}} \vect{x} - \vect{W}^{\frac{1}{2}} \vect{b}
            \|^2 \\
        &= \|
            \vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{b})
            \|^2 \\
        &= [\vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{b})]^T
            \vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{b}) \\
        &= (\vect{A} \vect{x} - \vect{b})^T \vect{W}^{\frac{1}{2}}
            \vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{b}) \\
        &= (\vect{A} \vect{x} - \vect{b})^T \vect{W}
           (\vect{A} \vect{x} - \vect{b}) \\
        &= \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2
    \end{align}

.. math::
    :nowrap:

    \begin{equation}
        \|\vect{x}'\|^2
        = \|\vect{Q}^{\frac{1}{2}} \vect{x}\|^2
        = (\vect{Q}^{\frac{1}{2}} \vect{x})^T (\vect{Q}^{\frac{1}{2}} \vect{x})
        = \vect{x}^T \vect{Q}^{\frac{1}{2}} \vect{Q}^{\frac{1}{2}} \vect{x}
        = \vect{x}^T \vect{Q} \vect{x}
        = \|\vect{x}\|_{\vect{Q}}^2
    \end{equation}

Substituting these transformations into the minimum-norm least-squares solution from Eq. :math:`\eqref{eq:lsq-min-norm}` yields:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}'
        &= (\vect{A}')^\dagger \vect{b}' \\
        \vect{x}'
        &= \vect{A}_{(W,Q)}^\dagger \vect{b}' \\
        \vect{Q}^{\frac{1}{2}} \vect{x}
        &= (\vect{W}^{\frac{1}{2}}
           \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger
           \vect{W}^{\frac{1}{2}} \vect{b} \\
        \vect{x}
        &= \vect{Q}^{-\frac{1}{2}}
           (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger
           \vect{W}^{\frac{1}{2}} \vect{b}
    \end{align}

where :math:`\vect{A}_{(W,Q)}^\dagger = \vect{Q}^{-\frac{1}{2}} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger \vect{W}^{\frac{1}{2}}` is the :math:`\{\vect{W},\vect{Q}\}`-weighted pseudoinverse.

Unified handling of overdetermined and underdetermined systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The benefit of this formulation is that the *same* formula can compute minimum-norm solutions for overdetermined and underdetermined systems.
In the following we employ the property :math:`(\vect{A} \vect{B} \vect{C})^{-1} = \vect{C}^{-1} \vect{B}^{-1} \vect{A}^{-1}`.

For an overdetermined system the solution reduces to the :math:`\{\vect{W}\}`-weighted, left pseudoinverse:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}
        &= \vect{Q}^{-\frac{1}{2}} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} [(\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^T (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})]^{-1} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^T \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} (\vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W}^{\frac{1}{2}} \vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^{-1} \vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W}^{\frac{1}{2}} \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} (\vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W} \vect{A} \vect{Q}^{-\frac{1}{2}})^{-1} \vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} \vect{Q}^{\frac{1}{2}} (\vect{A}^T \vect{W} \vect{A})^{-1} \vect{Q}^{\frac{1}{2}} \vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W} \vect{b} \\
        &= (\vect{A}^T \vect{W} \vect{A})^{-1} \vect{A}^T \vect{W} \vect{b} \\
        &= \vect{A}_{\vect{W}}^{\dagger} \vect{b}
    \end{align}

For an underdetermined system the solution reduces to the :math:`\{\vect{Q}^{-1}\}`-weighted, right pseudoinverse:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}
        &= \vect{Q}^{-\frac{1}{2}} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^T [(\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}}) (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^T]^{-1} \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-\frac{1}{2}} \vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W}^{\frac{1}{2}} (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}} \vect{Q}^{-\frac{1}{2}} \vect{A}^T \vect{W}^{\frac{1}{2}})^{-1} \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-1} \vect{A}^T \vect{W}^{\frac{1}{2}} \vect{W}^{-\frac{1}{2}} (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1} \vect{W}^{-\frac{1}{2}} \vect{W}^{\frac{1}{2}} \vect{b} \\
        &= \vect{Q}^{-1} \vect{A}^T (\vect{A} \vect{Q}^{-1} \vect{A}^T)^{-1} \vect{b} \\
        &= \vect{A}_{\vect{Q}^{-1}}^{\dagger} \vect{b}
    \end{align}

Weighted-minimum norm weighted least squares with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This problem resembles the previous one but with an added reference configuration :math:`\bar{\vect{x}}`:

.. math::
    :nowrap:

    \begin{align}
        &\text{Stage 1: }
        &\vect{x}(\hat{\vect{x}})
        &= \arg \min_{\vect{x}} f(\vect{x})
         = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2 \\
        &\text{Stage 2: }
        &\hat{\vect{x}}^*
        &= \arg \min_{\hat{\vect{x}}} g(\hat{\vect{x}})
         = \frac{1}{2} \|\vect{x}(\hat{\vect{x}}) - \bar{\vect{x}}\|_{\vect{Q}}^2
    \end{align}

Here we employ the following transformations:

* :math:`\vect{A}' = \vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}}`
* :math:`\vect{x}' = \vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})`
* :math:`\vect{b}' = \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}})`

These transformations enable the reduction to an (ordinary) least norm problem as follows:

.. math::
    :nowrap:

    \begin{align}
        \|\vect{A}' \vect{x}' - \vect{b}'\|^2
        &= \|
            \vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}}
            \vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})
            - \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}})
            \|^2 \\
        &= \|
            \vect{W}^{\frac{1}{2}} \vect{A} (\vect{x} - \bar{\vect{x}})
            - \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}})
            \|^2 \\
        &= \|
            \vect{W}^{\frac{1}{2}} [\vect{A} (\vect{x} - \bar{\vect{x}})
            - (\vect{b} - \vect{A} \bar{\vect{x}})]
            \|^2 \\
        &= \|
            \vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{A} \bar{\vect{x}}
            - \vect{b} + \vect{A} \bar{\vect{x}})
            \|^2 \\
        &= \|
            \vect{W}^{\frac{1}{2}} (\vect{A} \vect{x} - \vect{b})
            \|^2 \\
        &= \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2
    \end{align}

.. math::
    :nowrap:

    \begin{align}
        \|\vect{x}'\|^2
        &= \|\vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})\|^2 \\
        &= [\vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})]^T
           [\vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})] \\
        &= (\vect{x} - \bar{\vect{x}})^T \vect{Q}^{\frac{1}{2}}
            \vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}}) \\
        &= (\vect{x} - \bar{\vect{x}})^T \vect{Q} (\vect{x} - \bar{\vect{x}}) \\
        &= \|\vect{x} - \bar{\vect{x}}\|_{\vect{Q}}^2
    \end{align}

The resulting solution :math:`\vect{x}` is:

.. math::
    :nowrap:

    \begin{align}
        \vect{x}'
        &= (\vect{A}')^\dagger \vect{b}' \\
        \vect{x}'
        &= \vect{A}_{(W,Q)}^\dagger \vect{b}' \\
        \vect{Q}^{\frac{1}{2}} (\vect{x} - \bar{\vect{x}})
        &= (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger
            \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}}) \\
        \vect{x} - \bar{\vect{x}}
        &= \vect{Q}^{-\frac{1}{2}}
           (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger
           \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}}) \\
        \vect{x}
        &= \vect{Q}^{-\frac{1}{2}}
           (\vect{W}^{\frac{1}{2}} \vect{A} \vect{Q}^{-\frac{1}{2}})^\dagger
           \vect{W}^{\frac{1}{2}} (\vect{b} - \vect{A} \bar{\vect{x}})
            + \bar{\vect{x}} \\
        &= \vect{A}_{(W,Q)}^{\dagger}\vect{b} - \vect{A}_{(W,Q)}^{\dagger} \vect{A} \bar{\vect{x}} + \bar{\vect{x}} \\
        &= \vect{A}_{(W,Q)}^{\dagger} \vect{b} + \vect{N}_{(W,Q)} \bar{\vect{x}}
    \end{align}

where :math:`\vect{N}_{(W,Q)} = \vect{I} - \vect{A}_{(W,Q)}^{\dagger} \vect{A}` is the nullspace projector associated with the :math:`\{\vect{W},\vect{Q}\}`-weighted pseudoinverse.



Regularization
--------------

The objective function has contributions from the "system" (:math:`\vect{A} \vect{x} = \vect{b}`) and the "state" (:math:`\vect{x}`) to handle ill-conditioned problems for overdetermined and underdetermined systems.


Damped least squares or ridge regression
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Introduce a scalar weight :math:`\lambda` on the "state" vector.

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}} f(\vect{x}) = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|^2 + \lambda \|\vect{x}\|^2
    \end{equation}

Solve just as before:

.. math::
    :nowrap:

    \begin{align}
        \vect{A}^T (\vect{A} \vect{x} - \vect{b}) + \lambda \vect{I} \vect{x} &= \vect{0} \\
        \vect{A}^T \vect{W} \vect{A} \vect{x} - \vect{A}^T \vect{b} + \lambda \vect{I} \vect{x} &= \vect{0} \\
        (\vect{A}^T \vect{A} + \lambda \vect{I}) \vect{x} &= \vect{A}^T \vect{b} \\
        \vect{x} &= (\vect{A}^T \vect{A} + \lambda \vect{I})^{-1} \vect{A}^T \vect{b}
    \end{align}


Weighted damped least squares
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here, the *weighted* damped least squares is given without derivation:

.. math::
    :nowrap:

    \begin{equation}
        \vect{x} = (\vect{A}^T \vect{W} \vect{A} + \lambda \vect{I})^{-1} \vect{A}^T \vect{W} \vect{b}
    \end{equation}


Generalized Tikhonov regularization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The "system" (:math:`\vect{A} \vect{x} = \vect{b}`) and the "state" (:math:`\vect{x}`) are weighted by the positive-definited matrices :math:`\vect{W}` and :math:`\vect{Q}`, respectively.

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}} f(\vect{x})
        = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2 + \frac{1}{2} \|\vect{x}\|_{\vect{Q}}^2
    \end{equation}

Solve just as before:

.. math::
    :nowrap:

    \begin{align}
        \vect{A}^T \vect{W} (\vect{A} \vect{x} - \vect{b}) + \vect{Q} \vect{x} &= \vect{0} \\
        \vect{A}^T \vect{W} \vect{A} \vect{x} - \vect{A}^T \vect{W} \vect{b} + \vect{Q} \vect{x} &= \vect{0} \\
        (\vect{A}^T \vect{W} \vect{A} + \vect{Q}) \vect{x} &= \vect{A}^T \vect{W} \vect{b} \\
        \vect{x} &= (\vect{A}^T \vect{W} \vect{A} + \vect{Q})^{-1} \vect{A}^T \vect{W} \vect{b}
    \end{align}


Generalized Tikhonov regularization with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a mixture of :ref:`weighted least squares<sec:app:over:wlsq>` and :ref:`minimum-reference weighted least squares<sec:app:under:min-ref-wls>`.
An explanation is also available `here <https://en.wikipedia.org/wiki/Tikhonov_regularization#Generalized_Tikhonov_regularization>`__.

.. math::
    :nowrap:

    \begin{equation}
        \min_{\vect{x}} f(\vect{x})
        = \frac{1}{2} \|\vect{A} \vect{x} - \vect{b}\|_{\vect{W}}^2 + \frac{1}{2} \|\vect{x} - \bar{\vect{x}}\|_{\vect{Q}}^2
    \end{equation}

Solve just as before:

.. math::
    :nowrap:

    \begin{align}
        \vect{A}^T \vect{W} (\vect{A} \vect{x} - \vect{b}) + \vect{Q} (\vect{x} - \bar{\vect{x}}) &= \vect{0} \\
        \vect{A}^T \vect{W} \vect{A} \vect{x} - \vect{A}^T \vect{W} \vect{b} + \vect{Q} \vect{x} - \vect{Q} \bar{\vect{x}} &= \vect{0} \\
        (\vect{A}^T \vect{W} \vect{A} + \vect{Q}) \vect{x} &= \vect{A}^T \vect{W} \vect{b} + \vect{Q} \bar{\vect{x}} \\
        \vect{x} &= (\vect{A}^T \vect{W} \vect{A} + \vect{Q})^{-1} (\vect{A}^T \vect{W} \vect{b} + \vect{Q} \bar{\vect{x}})
    \end{align}




Numerical implementation via matrix decompositions
==================================================

Refer to [Press1992]_ and [Golub2013]_ for more details on the topic. The most relevant matrix decompositions (in order from highest to lowest numerical robustness, but in lowest to highest computational speed) are:

* the singular value decomposition (SVD) :math:`\vect{A} = \vect{U} \vect{\Sigma} \vect{V}^T` for an arbitrary matrix :math:`\vect{A}`
* the QR decomposition :math:`\vect{A} = \vect{Q} \vect{R}` for an arbitrary matrix :math:`\vect{A}`
* the LU decomposition :math:`\vect{A} = \vect{L} \vect{U}` for a square matrix :math:`\vect{A}`
* the Cholesky decomposition :math:`\vect{A} = \vect{L} \vect{L}^T` for a symmetric positive-definite matrix :math:`\vect{A}`

Only the first two will be discussed in the following.

.. note::
    In a *software implementation* all matrix inverses :math:`(\cdot)^{-1}`, pseudoinverses :math:`(\cdot)^{\dagger}` and weighted pseudoinverses :math:`(\cdot)_{\vect{W}}^{\dagger}` should rely on a matrix decomposition for better numerical robustness.
    One should never `explicitly <https://www.johndcook.com/blog/2010/01/19/dont-invert-that-matrix/>`__ `compute <https://gregorygundersen.com/blog/2020/12/09/matrix-inversion/>`__ an `inverse matrix <https://civilstat.com/2015/07/dont-invert-that-matrix-why-and-how/>`__.



Ordinary least squares
----------------------

SVD
^^^

Given the SVD decomposition of a matrix :math:`\vect{A}`, the solution to an ordinary least-squares problem is given by (see `here <https://stackoverflow.com/questions/9071020/compute-projection-hat-matrix-via-qr-factorization-svd-and-cholesky-factoriz>`__):

.. math::
    :nowrap:

    \begin{align}
        \vect{A} = \vect{U} \vect{\Sigma} \vect{V}^T
        \Rightarrow
        \vect{x}
        &= (\vect{A}^T \vect{A})^{-1} \vect{A}^T \vect{b} \\
        &= [(\vect{V} \vect{\Sigma}^{+} \vect{U}^T)
            (\vect{U} \vect{\Sigma}^{+} \vect{V}^T)]
            (\vect{V} \vect{\Sigma} \vect{U}^T) \vect{b} \\
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \vect{b}
    \end{align}

where :math:`\vect{\Sigma}^{+}` is the `pseudoinverse <https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Scalars>`__ of :math:`\vect{\Sigma}` where *(i)* non-zero entries are (scalar-)inverted; and *(ii)* zero entries are kept zero.

LAPACK provides the routines

* `DGESVD <https://netlib.org/lapack/explore-html-3.6.1/d1/d7e/group__double_g_esing_ga6a6ce95c3fd616a7091df45287c75cfa.html#ga6a6ce95c3fd616a7091df45287c75cfa>`__ for computing the SVD of an arbitrary (shape and rank-deficient) matrix
* `DGELSS <https://netlib.org/lapack/explore-html-3.6.1/d7/d3b/group__double_g_esolve_ga325c648632de4d60f5e59ee8d2a618f0.html#ga325c648632de4d60f5e59ee8d2a618f0>`__ and `DGELSD <https://netlib.org/lapack/explore-html-3.6.1/d7/d3b/group__double_g_esolve_ga479cdaa0d257e4e42f2fb5dcc30111b1.html>`__ for solving ordinary least-squares problems using an SVD of an arbitrary (shape and rank-deficient) matrix

See `here <https://www.netlib.org/lapack/lug/node27.html>`__, `here <https://www.netlib.org/lapack/lug/node32.html>`__ or `here <https://www.netlib.org/lapack/lug/node53.html>`__ for further LAPACK documentation (in increasing level detail).


QR decomposition
^^^^^^^^^^^^^^^^

Given the QR decomposition of a matrix :math:`\vect{A}`, the solution to an ordinary least-squares problem is (see `here <http://galton.uchicago.edu/~lekheng/courses/302/demmel/demmch3.pdf>`__ and `here <https://stackoverflow.com/questions/9071020/compute-projection-hat-matrix-via-qr-factorization-svd-and-cholesky-factoriz>`__)

.. math::
    :nowrap:

    \begin{equation}
        \vect{A} = \vect{Q} \vect{R}
        \Rightarrow
        \vect{x} = (\vect{A}^T \vect{A})^{-1} \vect{A}^T \vect{b}
                 = \vect{R}^{-1} \vect{Q}^T \vect{b}
    \end{equation}

:math:`\vect{R}` is an upper triangular and non-singular.
Hence, this problem is efficiently solvable by back substitution (see [Nocedal2006]_ pp. 432).

LAPACK provides the routines

* `DGEQRF <https://netlib.org/lapack/explore-html-3.6.1/dd/d9a/group__double_g_ecomputational_ga3766ea903391b5cf9008132f7440ec7b.html>`__ for computing the QR decomposition of an arbitrary matrix
* `DGELS <https://netlib.org/lapack/explore-html-3.6.1/d7/d3b/group__double_g_esolve_ga1df516c81d3e902cca1fc79a7220b9cb.html#ga1df516c81d3e902cca1fc79a7220b9cb>`__ for solving ordinary least-squares problems using a QR decomposition of a full-rank matrix.



Weighted least squares
----------------------

LAPACK features two routines (see `here <https://www.netlib.org/lapack/lug/node28.html>`__ for further documentation):

* `DGGGLM <https://netlib.org/lapack/explore-html-3.6.1/d3/df4/dggglm_8f_a43f7ff0e98b9b64fabd887b5ec22580e.html#a43f7ff0e98b9b64fabd887b5ec22580e>`__ supports a weight matrix but lacks support for a reference value :math:`\bar{\vect{x}}`.
* `DGGLSE <https://www.netlib.org/lapack/explore-html-3.6.1/d0/d85/dgglse_8f_a131c0fa85b2fb29d385ce87b199bf9aa.html>`__ supports a reference value :math:`\bar{\vect{x}}` but lacks support for a weight matrix.

Because both routines do not support the full feature set required for the above problems, we review the more fine-granular building blocks.
The following approaches should be ordered from highest to lowest numerical robustness.


Reduction to ordinary least squares
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At first sight, the weighted least-squares problem seems unapplicable for the above decompositions.
However, the following transformation allow us to reduce a weighted least-squares to an ordinary least squares problem (see `here <https://stackoverflow.com/questions/20562177/get-hat-matrix-from-qr-decomposition-for-weighted-least-square-regression>`__) :math:`\|\tilde{\vect{A}} \vect{x} - \tilde{\vect{b}}\|^2` with

.. math::
    :nowrap:

    \begin{align}
        \tilde{\vect{A}} &= \vect{W}^{\frac{1}{2}} \vect{A} \\
        \tilde{\vect{b}} &= \vect{W}^{\frac{1}{2}} \vect{b}
    \end{align}

This can then be solved by the SVD or QR decomposition as discussed above.

The square root of a positive-definite matrix :math:`\vect{W}` can be computed using an eigendecomposition (see [BenIsrael2003]_ p. 222 (Ex. 37) or `here <https://www.stat.uchicago.edu/~lekheng/courses/302/notes8.pdf>`__):

.. math::
    :nowrap:

    \begin{equation}
        \vect{W}
          = \vect{Z} \vect{\Lambda} \vect{Z}^T \Rightarrow \vect{W}^{\frac{1}{2}} = \vect{Z} \vect{\Lambda}^{\frac{1}{2}} \vect{Z}^T
    \end{equation}

where :math:`\vect{\Lambda}^{\frac{1}{2}}` are the component-wise square roots of the eigenvalues (due to positive-definiteness they are always positive).

LAPACK provides the routines `DSYEVR <https://netlib.org/lapack/explore-html-3.6.1/d2/d8a/group__double_s_yeigen_ga2ad9f4a91cddbf67fe41b621bd158f5c.html#ga2ad9f4a91cddbf67fe41b621bd158f5c>`__ or `DSYEV <https://netlib.org/lapack/explore-html-3.6.1/d2/d8a/group__double_s_yeigen_ga442c43fca5493590f8f26cf42fed4044.html>`__ for computing the eigendecomposition of a symmetric matrix.
See `here <https://www.seehuhn.de/pages/matrixfn.html>`__ for an example and `here <https://www.netlib.org/lapack/lug/node30.html>`__ or `here <https://www.netlib.org/lapack/lug/node48.html>`__ for further LAPACK documentation.


SVD of weighted matrix
^^^^^^^^^^^^^^^^^^^^^^

The reference (see `here <https://www.osti.gov/servlets/purl/5047753>`__) suggests the following approach:

.. math::
    :nowrap:

    \begin{equation}
        \vect{W} \vect{A} = \vect{U} \vect{\Sigma} \vect{V}^T \Rightarrow \vect{x} = \vect{V} \vect{\Sigma}^{+} \vect{U}^T \vect{W} \vect{b}
    \end{equation}

While computationally more efficient (due to less operations), it computes a product of two matrices first which may reduce numerical robustness.


QR decomposition of weighted matrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Similar to the SVD decomposition (see `here <https://www.osti.gov/servlets/purl/5047753>`__) (but with constraints on the dimension of :math:`\vect{A}`):

.. math::
    :nowrap:

    \begin{equation}
        \vect{W} \vect{A} = \vect{Q} \vect{R}
        \Rightarrow
        \vect{x} = (\vect{A}^T \vect{W} \vect{A})^{-1} \vect{A}^T \vect{W} \vect{b}
                 = \vect{R}^{-1} \vect{Q}^T \vect{W} \vect{b}
    \end{equation}



Regularized least squares
-------------------------

The regularized least squares with a damping parameter :math:`\lambda` can be solved efficiently given an SVD as explained `here <https://en.wikipedia.org/wiki/Tikhonov_regularization#Relation_to_singular-value_decomposition_and_Wiener_filter>`__:

.. math::
    :nowrap:

    \begin{equation}
        \vect{A} = \vect{U} \vect{\Sigma} \vect{V}^T
        \Rightarrow
        \vect{x} = \vect{V} \vect{D} \vect{U}^T \vect{b}
    \end{equation}

with the diagonal values :math:`D_{i,i}` of matrix :math:`\vect{D}` as

.. math::
    :nowrap:

    \begin{equation}
        D_{i,i} = \frac{\sigma_i}{\sigma_i^2 + \lambda^2}
    \end{equation}


Weighted-minimum norm weighted least squares
--------------------------------------------
Use the reduction to an ordinary least-squares problem provided in the associated section above and solve that reduced problem in turn using an SVD or a QR decomposition.

Alternatively, compute the SVD of either weight matrix and adapt the last approach for the weighted least squares.

[BenIsrael2003]_ (p. 255, Eq. 201):

.. math::
    \vect{x} = \vect{V} \vect{D}^{\dagger}_A \vect{U}^{-1} \vect{b}




Algorithms for HDDC platforms
=============================

Force distribution
------------------

The following routines map forces from the platform frame to the pivot frame.

Singular platform
^^^^^^^^^^^^^^^^^

Solver:

  .. math::
        \vect{F}_d
        = (\vect{G}^T \vect{W}_p \vect{G})^{-1} \vect{G}^T \vect{W}_p \vect{F}_p

1. Weighting:

    .. math::
        \vect{Z}_p, \vect{\Lambda}_p
        &= \operatorname{dsyev}(\vect{W}_p) \\
        \vect{W}_p^{\frac{1}{2}}
        &= \vect{Z}_p \vect{\Lambda}_p^{\frac{1}{2}} \vect{Z}_p^T \\
        \vect{G}'
        &= \vect{W}_p^{\frac{1}{2}} \vect{G} \\
        \vect{F}_p'
        &= \vect{W}_p^{\frac{1}{2}} \vect{F}_p

2. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}') \\
        \vect{F}_d
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \vect{F}_p'


Redundant platform with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Solver:

  .. math::
        \vect{F}_d
        = \bar{\vect{F}}_d
          + \vect{W}_d^{-1} \vect{G}^T
            (\vect{G} \vect{W}_d^{-1} \vect{G}^T)^{-1}
            (\vect{F}_p - \vect{G} \bar{\vect{F}}_d)

1. Reference (initial):

    .. math::
        \vect{F}_p' = \vect{F}_p - \vect{G} \bar{\vect{F}}_d

2. Weighting (initial):

    .. math::
        \vect{Z}_d, \vect{\Lambda}_d
        &= \operatorname{dsyev}(\vect{W}_d) \\
        \vect{W}_d^{-\frac{1}{2}}
        &= \vect{Z}_d \vect{\Lambda}_d^{-\frac{1}{2}}
           \vect{Z}_d^T \\
        \vect{G}'
        &= \vect{G} \vect{W}_d^{-\frac{1}{2}}

3. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}') \\
        \vect{F}_d'
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \vect{F}_p'

4. Weighting (final):

    .. math::
        \vect{F}_d'' = \vect{W}_d^{-\frac{1}{2}} \vect{F}_d'

5. Reference (final):

    .. math::
        \vect{F}_d = \bar{\vect{F}}_d + \vect{F}_d''


Fused singular and redundant platform with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Solver:

    .. math::
        \vect{F}_d
        = \bar{\vect{F}}_d + \vect{W}_d^{-\frac{1}{2}}
          (\vect{W}_p^{\frac{1}{2}} \vect{G} \vect{W}_d^{-\frac{1}{2}})^{-1}
          \vect{W}_p^{\frac{1}{2}} (\vect{F}_p - \vect{G} \bar{\vect{F}}_d)

1. Reference (initial):

    .. math::
        \vect{F}_p' = \vect{F}_p - \vect{G} \bar{\vect{F}}_d

2. Weighting (initial):

    .. math::
        \vect{Z}_p, \vect{\Lambda}_p
        &= \operatorname{dsyev}(\vect{W}_p) \\
        \vect{W}_p^{\frac{1}{2}}
        &= \vect{Z}_p \vect{\Lambda}_p^{\frac{1}{2}}
           \vect{Z}_p^T \\
        \vect{Z}_d, \vect{\Lambda}_d
        &= \operatorname{dsyev}(\vect{W}_d) \\
        \vect{W}_d^{-\frac{1}{2}}
        &= \vect{Z}_d \vect{\Lambda}_d^{-\frac{1}{2}}
           \vect{Z}_d^T \\
        \vect{G}'
        &= \vect{W}_p^{\frac{1}{2}} \vect{G}
           \vect{W}_d^{-\frac{1}{2}} \\
        \vect{F}_p''
        &= \vect{W}_p^{\frac{1}{2}} \vect{F}_p'

3. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}') \\
        \vect{F}_d'
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \vect{F}_p''

4. Weighting (final):

    .. math::
        \vect{F}_d'' = \vect{W}_d^{-\frac{1}{2}} \vect{F}_d'

5. Reference (final):

    .. math::
        \vect{F}_d = \bar{\vect{F}}_d + \vect{F}_d''


Velocity composition
--------------------

The following routines map velocities from the pivot frame to the platform frame.

Singular platform
^^^^^^^^^^^^^^^^^

Solver:

  .. math::
        \dot{\vect{X}}_p
        = (\vect{G} \vect{W}_d \vect{G}^T)^{-1} \vect{G} \vect{W}_d
          \dot{\vect{X}}_d

1. Weighting:

    .. math::
        \vect{Z}_d, \vect{\Lambda}_d
        &= \operatorname{dsyev}(\vect{W}_d) \\
        \vect{W}_d^{\frac{1}{2}}
        &= \vect{Z}_d \vect{\Lambda}_d^{\frac{1}{2}} \vect{Z}_d^T \\
        \vect{G}'
        &= \vect{G} \vect{W}_d^{\frac{1}{2}} \\
        \dot{\vect{X}}_d'
        &= \vect{W}_d^{\frac{1}{2}} \dot{\vect{X}}_d

2. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}'^T) \\
        \dot{\vect{X}}_p
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \dot{\vect{X}}_d'


Redundant platform with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Solver:

  .. math::
        \dot{\vect{X}}_p
        = \bar{\dot{\vect{X}}}_p
          + \vect{W}_p^{-1} \vect{G}
            (\vect{G}^T \vect{W}_p^{-1} \vect{G})^{-1}
            (\dot{\vect{X}}_d - \vect{G}^T \bar{\dot{\vect{X}}}_p)

1. Reference (initial):

    .. math::
        \dot{\vect{X}}_d' = \dot{\vect{X}}_d - \vect{G}^T \bar{\dot{\vect{X}}}_p

2. Weighting (initial):

    .. math::
        \vect{Z}_p, \vect{\Lambda}_p
        &= \operatorname{dsyev}(\vect{W}_p) \\
        \vect{W}_p^{-\frac{1}{2}}
        &= \vect{Z}_p \vect{\Lambda}_p^{-\frac{1}{2}}
           \vect{Z}_p^T \\
        \vect{G}'
        &= \vect{W}_p^{-\frac{1}{2}} \vect{G}

3. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}'^T) \\
        \dot{\vect{X}}_p'
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \dot{\vect{X}}_d'

4. Weighting (final):

    .. math::
        \dot{\vect{X}}_p'' = \vect{W}_p^{-\frac{1}{2}} \dot{\vect{X}}_p'

5. Reference (final):

    .. math::
        \dot{\vect{X}}_p = \bar{\dot{\vect{X}}}_p + \dot{\vect{X}}_p''


Fused singular and redundant platform with reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Solver:

    .. math::
        \dot{\vect{X}}_p
        = \bar{\dot{\vect{X}}}_p + \vect{W}_p^{-\frac{1}{2}}
          (\vect{W}_d^{\frac{1}{2}} \vect{G}^T \vect{W}_p^{-\frac{1}{2}})^{-1}
          \vect{W}_d^{\frac{1}{2}}
          (\dot{\vect{X}}_d - \vect{G}^T \bar{\dot{\vect{X}}}_p)

1. Reference (initial):

    .. math::
        \dot{\vect{X}}_d' = \dot{\vect{X}}_d - \vect{G}^T \bar{\dot{\vect{X}}}_p

2. Weighting (initial):

    .. math::
        \vect{Z}_d, \vect{\Lambda}_d
        &= \operatorname{dsyev}(\vect{W}_d) \\
        \vect{W}_d^{\frac{1}{2}}
        &= \vect{Z}_d \vect{\Lambda}_d^{\frac{1}{2}} \vect{Z}_d^T \\
        \vect{Z}_p, \vect{\Lambda}_p
        &= \operatorname{dsyev}(\vect{W}_p) \\
        \vect{W}_p^{-\frac{1}{2}}
        &= \vect{Z}_p \vect{\Lambda}_p^{-\frac{1}{2}}
           \vect{Z}_p^T \\
        \vect{G}'
        &= \vect{W}_p^{-\frac{1}{2}} \vect{G} \vect{W}_d^{\frac{1}{2}} \\
        \dot{\vect{X}}_d''
        &= \vect{W}_d^{\frac{1}{2}} \dot{\vect{X}}_d'

3. OLS:

    .. math::
        \vect{U}, \vect{\Sigma}, \vect{V}^T
        &= \operatorname{dgesvd}(\vect{G}'^T) \\
        \dot{\vect{X}}_p'
        &= \vect{V} \vect{\Sigma}^{+} \vect{U}^T \dot{\vect{X}}_d''

4. Weighting (final):

    .. math::
        \dot{\vect{X}}_p'' = \vect{W}_p^{-\frac{1}{2}} \dot{\vect{X}}_p'

5. Reference (final):

    .. math::
        \dot{\vect{X}}_p = \bar{\dot{\vect{X}}}_p + \dot{\vect{X}}_p''


Context
-------

The "fused" algorithm version is sometimes used for robotic manipulators where *(i)* :math:`\vect{x}` represents a joint velocity; *(ii)* :math:`\vect{b}` represents a Cartesian-space velocity twist; and *(iii)* :math:`\vect{A}` represents the manipulator's Jacobian matrix.
The `WDLS <https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_wdls.hpp>`__ solver of the Kinematics and Dynamics Library (KDL) is an example implementation.
The "benefit" is that the same implementation can handle redundant (e.g. a 7-DoF *3-1-3* robot solving a 6-DoF Cartesian task) and singular (e.g. a 5-DoF *2-1-2* robot solving a 6-DoF Cartesian task) kinematic chains.
In the former case the solver minimizes over joint-space velocities (while exactly achieving the Cartesian-space velocity twist), whereas in the latter case the solver minimizes over Cartesian-space velocity twists (while using all joints).

This algorithm is *not* a good choice if the kinematic chain structure is known and free from singularities.
In such a situation it is better to use one of the dedicated algorithms for either singular or redundant platforms to avoid superflous computations.


.. _sec_cheating:

"Cheating"
----------

In various robotics applications the concrete values in the weight matrices are deemed irrelevant, only their relative values matter.
Then the following properties are sometimes exploited:

* :math:`\vect{X}` positive-definite :math:`\rightarrow` :math:`\vect{X}^{-1}` positive-definite
* :math:`\vect{X}` positive-definite :math:`\rightarrow` :math:`\vect{X}^{\frac{1}{2}}` positive-definite

As a result, one can get by with somewhat "arbitrarily-designed" positive-definite matrices :math:`\vect{W}' \sim \vect{W}^{\frac{1}{2}}` and :math:`\vect{Q}' \sim \vect{Q}^{-\frac{1}{2}}` so that

.. math::
    \vect{x} = \vect{Q}' (\vect{W}' \vect{A} \vect{Q}')^\dagger \vect{W}' \vect{b}

Note, that neither the :math:`\vect{W}` nor the :math:`\vect{Q}` matrix should provide a "zero weight" for any :math:`\vect{x}` coordinate value as that violates the positive-definiteness assumption.
This problem may be "hidden" behind a damped least squares solution, though.