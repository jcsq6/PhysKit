if (PHYSKIT_BUILD_GRAPHICS OR NOT DEFINED PHYSKIT_BUILD_GRAPHICS)
    message(STATUS "PhysKit: Including graphical tests.")
    add_compile_definitions(PHYSKIT_BUILD_GRAPHICS)
    include(${CMAKE_SOURCE_DIR}/cmake/FetchMagnum.cmake)
else()
    message(STATUS "PhysKit: Skipping graphical tests.")
    set(MAGNUM_LIBRARIES "")
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
)

target_link_libraries(phys_graphics INTERFACE ${MAGNUM_LIBRARIES} physkit)

add_subdirectory(demo)
add_subdirectory(graphics_demo)