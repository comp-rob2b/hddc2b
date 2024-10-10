# hddc2b

`hddc2b` is a C library that provides building blocks (2b) for solving planar, two-dimensional kinematics and statics problems of mobile robots that feature a hub-drive with differential-castor (HDDC) kinematics configuration such as the [KELO Robotics](https://www.kelo-robotics.com/products/) platforms.
The library is complemented by a code generator that composes these building blocks into a wide range of concrete solvers as specified via a JSON-based configuration language.

By design, `hddc2b` only supports two levels of task hierarchy and deliberately excludes a velocity distribution solver.
The library is directly compatible with the data structure layout of the KELO Robotics interface provided by the [`robif2b`](https://github.com/rosym-project/robif2b) library.


More details of HDDC platforms and their integration into overall robot control architectures are also explained in [this online book](https://robmosys.pages.gitlab.kuleuven.be/).

## Documentation

The documentation comprises

* the [installation instructions](https://comp-rob2b.github.io/hddc2b/installation.html)
* [code generator](https://comp-rob2b.github.io/hddc2b/code_generation.html) instructions
* a [mathematical derivation](https://comp-rob2b.github.io/hddc2b/least_squares.html) of the solvers
* a [frame diagram](https://comp-rob2b.github.io/hddc2b/frames.html) for KELO robots
* an [example](src/example/hddc2b_example.c) that showcases how to use the solver.

## Third-party software

* [`CodeCoverage.cmake`](thirdparty/cmake/codecoverage/CodeCoverage.cmake) is licensed under the BSD-3-Clause license and originates from Lars Bilke's project [Additional CMake Modules](https://github.com/bilke/cmake-modules).
* [`FindSphinx.cmake`](thirdparty/cmake/sphinx/FindSphinx.cmake) is licensed under the BSD-3-Clause license and originates from Jeroen Koekkoek's project [Sphinx integration for CMake](https://github.com/k0ekk0ek/cmake-sphinx).

## Contributors

* [Vamsi Kalagaturu](https://github.com/vamsikalagaturu)

## License

The core ``hddc2b`` library is licensed under the GNU Lesser General Public License v3.0 only (see [`LICENSE.LGPL-3.0`](LICENSE.LGPL-3.0))

The code generator is licensed under the MIT No Attribution (see [`LICENSE.MIT-0`](LICENSE.MIT-0)) License.

Effectively that means you can link against the core library from free or proprietary applications.
Only when you modify the core library, these changes must be released under the conditions of the LGPL-3.0.
Additionally, you can modify or reuse everything related to the code generator without even acknowledging ``hddc2b``.

## Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.
