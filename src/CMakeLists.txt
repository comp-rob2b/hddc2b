find_package(lapacke REQUIRED)
find_package(cblas REQUIRED)
find_library(MATH_LIBRARY m)

add_library(hddc2b SHARED
  config.c
  wheel.c
  drive.c
  platform.c
)

target_include_directories(hddc2b
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(hddc2b
  PRIVATE
    cblas
    lapacke
    ${MATH_LIBRARY}
)

install(
  TARGETS hddc2b
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
