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

add_library(phys_testing INTERFACE)
target_sources(phys_testing
  INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/helper
    FILES
      helper/test.h
)

target_link_libraries(phys_testing INTERFACE physkit)

if (PHYSKIT_BUILD_GRAPHICS_TESTS)
  set(PHYS_GRAPHICS_HEADERS
    helper/dep/argparse.hpp
    helper/graphics/detail/magnum_headers.h
    helper/graphics/detail/macro.h
    helper/graphics/detail/gmf.h
    helper/graphics/convert.h
    helper/graphics/camera.h
    helper/graphics/graphics.h
  )

  if (PHYSKIT_GRAPHICS_MODULES)
    set(CMAKE_CXX_SCAN_FOR_MODULES ON)
    add_library(phys_graphics SHARED)

    target_sources(phys_graphics
      PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/helper
      FILES
        ${PHYS_GRAPHICS_HEADERS}
    )

    target_sources(phys_graphics
      PUBLIC
        FILE_SET modules TYPE CXX_MODULES
      FILES
        helper/src/modules/magnum_m.cpp
        helper/src/modules/graphics_m.cpp
        helper/src/modules/camera_m.cpp
        helper/src/modules/convert_m.cpp
    )

    target_link_libraries(phys_graphics PUBLIC ${TEST_LIBRARIES} physkit)
    target_compile_definitions(phys_graphics PUBLIC PHYSKIT_GRAPHICS_MODULES)
    target_include_directories(phys_graphics PRIVATE graphics dep)
  else()
    set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
    add_library(phys_graphics SHARED)

    set(PHYS_GRAPHICS_SOURCES
      helper/src/graphics.cpp
    )

    target_sources(phys_graphics
      PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/helper
      FILES
        ${PHYS_GRAPHICS_HEADERS}
        PUBLIC
        ${PHYS_GRAPHICS_SOURCES} 
    )

    if (NOT PHYSKIT_MODULES)
      target_precompile_headers(phys_graphics PUBLIC
        [["graphics/detail/magnum_headers.h"]]
        <GLFW/glfw3.h>
        <glaze/glaze.hpp>
        <physkit/physkit.h>
      )
    endif()
    target_link_libraries(phys_graphics PUBLIC ${TEST_LIBRARIES} physkit)
    target_include_directories(phys_graphics PUBLIC graphics dep)
  endif()
endif()

add_subdirectory(mesh)
add_subdirectory(obb)
add_subdirectory(dynamic_bvh)
add_subdirectory(gjk)
add_subdirectory(epa)

if (PHYSKIT_BUILD_GRAPHICS_TESTS)
  add_subdirectory(demo)
  add_subdirectory(graphics_demo)
endif()
