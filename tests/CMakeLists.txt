if (PHYSKIT_BUILD_TESTS)
  add_compile_definitions(PHYSKIT_BUILD_TESTS)
  find_package(glaze QUIET)

  if (glaze_FOUND)
    message(STATUS "PhysKit: Including graphical tests.")
    include(${CMAKE_SOURCE_DIR}/cmake/FetchMagnum.cmake)
    list(APPEND TEST_LIBRARIES "glaze::glaze")
    set(PHYSKIT_BUILD_GRAPHICS_TESTS ON)
  else()
    message(WARNING "PhysKit: glaze not found, skipping graphical tests.")
    set(PHYSKIT_BUILD_GRAPHICS_TESTS OFF)
  endif()
else()
    message(STATUS "PhysKit: Skipping graphical tests.")
    set(TEST_LIBRARIES "")
    set(PHYSKIT_BUILD_GRAPHICS_TESTS OFF)
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

add_library(phys_testing INTERFACE)
target_sources(phys_testing
  INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
      include/test.h
)

target_link_libraries(phys_graphics INTERFACE ${TEST_LIBRARIES} physkit)
target_link_libraries(phys_testing INTERFACE physkit)


add_subdirectory(mesh)
add_subdirectory(obb)
add_subdirectory(dynamic_bvh)
add_subdirectory(gjk)
add_subdirectory(epa)
