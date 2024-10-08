find_package(check)

add_executable(main_test
  main_test.c
  platform_test.c
  drive_test.c
  wheel_test.c
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
