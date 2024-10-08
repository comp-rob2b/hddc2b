add_library(hddc2b SHARED
  config.c
  wheel.c
)

target_include_directories(hddc2b
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
