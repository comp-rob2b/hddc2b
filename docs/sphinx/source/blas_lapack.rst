BLAS & LAPACK
*************

Allocation-free SVD
===================

LAPACKE offers two functions to compute the SVD of a matrix: `LAPACKE_dgesvd` and `LAPACKE_dgesvd_work`. As the SVD requires additional work memory this can be dynamically allocated (as in the former function) or passed explicitly as an argument (in the latter function) in which the caller can decide where and how to allocate the work memory. Still, even the latter function will always dynamically allocate memory for *row-major* matrices. The general rules for computing the size of that working memory is:

- if `JOBU` = 'N' & :math:`M \gg N: \text{LWORK} \ge \max(1, 5 \cdot \min(M, N))`
- elseif `JOBVT` = 'N' & :math:`N \gg M: \text{LWORK} \ge \max(1, 5 \cdot \min(M, N))`
- else: :math:`\text{LWORK} \ge \max(1, 3 \cdot \min(M, N) + \max(M, N), 5 \cdot \min(M,N))`

The matrices in this project are always in column-major layout, small enough to be stack-allocated and fall under the third case of the rules. More specifically, the following matrices are decomposed in this project:

- :math:`M = 3, N \in \{2 x: x \in \mathbb{N}\}:\text{LWORK} \ge 15 + (N - 4)`.
- :math:`M = N = 2: \text{LWORK} \ge 10`
- :math:`M = N = 3: \text{LWORK} \ge 15`