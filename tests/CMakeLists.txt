if (PHYSKIT_BUILD_TESTS)
  message(STATUS "PhysKit: Including graphical tests.")
  add_compile_definitions(PHYSKIT_BUILD_TESTS)
  include(${CMAKE_SOURCE_DIR}/cmake/FetchMagnum.cmake)

  find_package(glaze REQUIRED)
  list(APPEND TEST_LIBRARIES "glaze::glaze")
else()
    message(STATUS "PhysKit: Skipping graphical tests.")
    set(TEST_LIBRARIES "")
endif()

add_library(phys_graphics INTERFACE)

target_sources(phys_graphics
  INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
      include/graphics.h
      include/camera.h
      include/convert.h
      include/dep/argparse.hpp
)

target_link_libraries(phys_graphics INTERFACE ${TEST_LIBRARIES} physkit)

add_subdirectory(demo)
add_subdirectory(mesh)
add_subdirectory(graphics_demo)