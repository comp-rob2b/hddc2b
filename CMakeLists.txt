cmake_minimum_required(VERSION 3.19)
project(hddc2b)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cmake/codecoverage)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cmake/sphinx)

include(GNUInstallDirs)
include(CMakeDependentOption)

option(ENABLE_TESTS                         "Build unit tests" Off)
option(ENABLE_DOC                           "Build documentation" Off)
option(ENABLE_PACKAGE_REGISTRY              "Add this package to CMake's package registry" Off)
cmake_dependent_option(ENABLE_TEST_COVERAGE "Generate a test coverage report" OFF "ENABLE_TESTS" OFF)

if(ENABLE_TEST_COVERAGE)
  include(CodeCoverage)
  append_coverage_compiler_flags()
endif()

# The directory to which all CMake files are installed
set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

add_subdirectory(src)
add_subdirectory(generator)

# Build the test suite
if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

# Build the documentation
if(ENABLE_DOC)
  find_package(Doxygen REQUIRED)
  find_package(Sphinx REQUIRED breathe)

  set(DOXYGEN_GENERATE_HTML NO)
  set(DOXYGEN_GENERATE_XML YES)
  set(DOXYGEN_ALIASES
    [[href{2}="\verbatim embed:rst:inline `\1 <\2>`_ \endverbatim"]]
    [[cite{1}="\verbatim embed:rst:inline [\1]_ \endverbatim"]]
    [[func{1}="\verbatim embed:rst:inline :c:func:`\1` \endverbatim"]]
  )
  set(DOXYGEN_VERBATIM_VARS DOXYGEN_ALIASES)
  doxygen_add_docs(${PROJECT_NAME}_doxygen include)

  sphinx_add_docs(
    ${PROJECT_NAME}_sphinx
    BREATHE_PROJECTS ${PROJECT_NAME}_doxygen
    BUILDER html
    SOURCE_DIRECTORY docs/sphinx/source
  )
endif()

# Export this package to CMake's package registry
if(ENABLE_PACKAGE_REGISTRY)
  export(PACKAGE ${PROJECT_NAME})
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DIR}
)

# Generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  VERSION "0.1.0"
  COMPATIBILITY AnyNewerVersion
)

# Install cmake configuration and package version
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION ${CMAKE_INSTALL_DIR}
)

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR}
)

# Install all public header files
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)
