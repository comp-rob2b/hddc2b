find_package(check)

add_custom_command(
  OUTPUT solver.h
  COMMAND
    jinja2 --format json
    ${PROJECT_SOURCE_DIR}/generator/hdr.jinja
    ${PROJECT_SOURCE_DIR}/generator/example.hddc2b.json
    > solver.h
  DEPENDS
    ${PROJECT_SOURCE_DIR}/generator/hdr.jinja
    ${PROJECT_SOURCE_DIR}/generator/example.hddc2b.json
)

add_custom_command(
  OUTPUT solver.c
  COMMAND
    jinja2 --format json
    ${PROJECT_SOURCE_DIR}/generator/impl.jinja
    ${PROJECT_SOURCE_DIR}/generator/example.hddc2b.json
    > solver.c
  DEPENDS
    solver.h
    ${PROJECT_SOURCE_DIR}/generator/impl.jinja
    ${PROJECT_SOURCE_DIR}/generator/example.hddc2b.json
)

add_executable(main_test
  main_test.c
  solver_test.c
  platform_test.c
  drive_test.c
  wheel_test.c
  solver.c
)

target_include_directories(main_test
  PRIVATE
    ${CMAKE_BINARY_DIR}/test
)

target_link_libraries(main_test
  PRIVATE
    hddc2b
    Check::checkShared
)

set_target_properties(main_test
  PROPERTIES
    C_STANDARD 11
)

add_test(main_test
  ${CMAKE_CURRENT_BINARY_DIR}/main_test
)

if(ENABLE_TEST_COVERAGE)
  setup_target_for_coverage_lcov(
    NAME coverage
    DEPENDENCIES main_test
    EXECUTABLE main_test
    EXCLUDE src/example/*
  )
endif()
