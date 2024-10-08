API documentation
=================

The API does not enforce any specific units of measurements, but all quantities must have consistent units.
For instance, all lengths must be provided in metres.

By default, any array that contains measurements from or commands to a drive are arranged in right-before-left order.
This convention aligns with the `KELO Drive API <https://github.com/kelo-robotics/kelo_tulip/blob/master/include/kelo_tulip/KeloDriveAPI.h>`__ where the right wheel has identifier :math:`1` (index :math:`0`) and the left wheel has identifier :math:`2` (index :math:`1`).

All matrices in ``hddc2b`` must be provided and will be returned in **column-major** storage order.


.. doxygenindex::