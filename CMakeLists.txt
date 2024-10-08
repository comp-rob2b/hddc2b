cmake_minimum_required(VERSION 3.19)
project(hddc2b)

option(ENABLE_TESTS                         "Build unit tests" Off)

add_subdirectory(src)

# Build the test suite
if(ENABLE_TESTS)
  add_subdirectory(test)
endif()
