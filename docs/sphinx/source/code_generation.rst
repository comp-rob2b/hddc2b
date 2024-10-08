Code generation
***************

Least squares solvers feature many policies as explained `here <least_squares.rst>`_.
Most of these policies are reflected in the solvers' control-flow.
Hence, a library typically has the choice of either trying to cover all possible combinations or ignoring some/most of these combinations.
The former case leads to an "API explosion" where the library provides a vast amount of dedicated functions that closely resemble each other and must encode the policy in its name and arguments.
As an alternative, ``hddc2b`` only provides the building blocks of such solvers while relying on a configuration file and a code generator to realize the *specific* solver required by the concrete application.

Domain-specific language for configuration
==========================================

The configuration is provided as a simple JSON file with the following structure and semantics.
Required data types are typographically indicated by a [**bold font**].


Generator configuration
-----------------------

The top-level JSON object with the following properties.

* ``@context``: must be any valid `JSON-LD context <https://www.w3.org/TR/json-ld/#the-context>`_ that defines the used terms. A convenience option is to use the provided ``hddc2b.context.json`` context.
* ``@type``: must either be a [**string**] with the value or an [**array** of strings] containing ``GeneratorConfiguration``
* ``solvers``: an array of objects, each containing a :ref:`solver configuration<solver-configuration>`

The first two properties, starting with an ``@``, indicate the file's conformance to `JSON-LD <https://json-ld.org/>`__.
They are optional (as long as the SHACL validation is not required), yet recommended and allow us to connect the JSON file to the `Semantic Web <https://en.wikipedia.org/wiki/Semantic_web>`_.


.. _solver-configuration:

Solver configuration
--------------------

Each solver configuration object must be comprised of the following properties. 

* ``name`` [**string**]: a valid C identifier that will be the name of the generated function
* ``quantity`` [**string**]: indicate for which physical quantity this solver is meant to be. The value must either be

  - ``force`` for the *force distribution* solver; or
  - ``velocity`` for the *velocity composition* solver.

* ``preprocess-platform-weight`` [**boolean**]: determine whether to preprocess the platform weight matrix in the solver or if it is passed in as an argument (see also :ref:`sec_cheating`).

  - For the *force distribution* solver this will compute the square root of that matrix.
  - For the *velocity composition* solver this will compute the inverse square root of that matrix.

* ``preprocess-drive-weight`` [**boolean**]: determine whether to preprocess the drive weight matrix in the solver or if it is passed in as an argument (see also :ref:`sec_cheating`).

  - For the *force distribution* solver this will compute the inverse square root of that matrix.
  - For the *velocity composition* solver this will compute the square root of that matrix.

* ``weight-in-platform-space`` [**boolean**]: use this flag when the platform *may* be(come) singular
* ``weight-in-drive-space`` [**boolean**]: use this flag when the platform *may* be(come) redundant
* ``has-secondary-task`` [**boolean**]: use this flag to indicate if the solver should handle a lower-priority task that will be projected into the nullspace of the platform task.

  - For the *force distribution* solver the secondary task is specified in the drive space (e.g. for aligning the drive units with a lower priority than solving the platform-level task).
  - For the *velocity composition* solver the secondary task is specified in the platform space.

* ``inverse`` [**string**]: the type of inverse to compute. The value must either be

  - ``pseudoinverse`` for computing a `(scalar) pseudoinverse <https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Scalars>`_; or
  - ``damped-least-squares`` for computing a damped inverse akin to `ridge regression <https://en.wikipedia.org/wiki/Ridge_regression#Relation_to_singular-value_decomposition_and_Wiener_filter>`_.

* ``description``: an array of strings that will be used as a comment in the generated header file


Validation
----------

The configuration can be validated in two ways, either via `SHACL constraints <../../generator/hddc2b.shacl.ttl>`_ or `JSON-Schema <../../generator/hddc2b.schema.json>`_.

The SHACL constraint check requires the JSON-LD keywords ``@context`` and ``@type`` to be present.
Then execute the following command in the ``generator/`` folder:

.. code-block:: sh

   make shacl

The JSON-Schema check works without the JSON-LD keywords and can be invoked via

.. code-block:: sh

   make schema


Code generation and embedding into a project
============================================

When the configuration is valid, the C code can be generated using the Jinja2 template engine and the associated templates.
The following command only prints the generated code to the terminal.

.. code-block:: sh

   make jinja

Alternatively, the ``hddc2b`` tests provide a CMake example on how to trigger the code generator at build time and compile the generated code as part of a project.

The generated code assumes that the solver's header file ``solver.h`` is available to the C compiler via an include directory.
The ``hddc2b`` example demonstrate this for a CMake project:

.. code-block:: cmake

   target_include_directories(hddc2b_example
     PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}
   )


Licensing
=========

All generator-related tooling (templates, schemata, or the example configuration) are licenced under the `MIT No Attribution <https://github.com/aws/mit-0>`__ Licence.
It is provided as a reference implementation that can be used, modified and even relicensed without restrictions
By design, the MIT No Attribution licence does not even have to be mentioned in any derivative work.
The same also applies to the generated code.