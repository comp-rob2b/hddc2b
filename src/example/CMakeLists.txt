add_executable(hddc2b_example
  hddc2b_example.c
  solver.c
)

target_include_directories(hddc2b_example
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(hddc2b_example
  hddc2b
)