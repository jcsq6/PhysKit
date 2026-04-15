cmake_minimum_required(VERSION 3.40)

project(PhysKit VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	# find_library doesn't find libstdc++exp.a with gcc
	execute_process(
		COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++exp.a
		OUTPUT_VARIABLE STDCXXEXP_LIBRARY
		OUTPUT_STRIP_TRAILING_WHITESPACE)
	if(STDCXXEXP_LIBRARY AND NOT STDCXXEXP_LIBRARY STREQUAL "libstdc++exp.a")
		link_libraries($<LINK_ONLY:${STDCXXEXP_LIBRARY}>)
	endif()

	# TODO: find a better alternative than just statically linking the runtime libraries on windows
	if(WIN32)
		add_link_options(-static-libstdc++ -static-libgcc)
	endif()
else()
	find_library(STDCXXEXP_LIBRARY NAMES stdc++exp)
	if(STDCXXEXP_LIBRARY)
		link_libraries($<LINK_ONLY:${STDCXXEXP_LIBRARY}>)
	endif()
endif()

include(${CMAKE_SOURCE_DIR}/cmake/EnsureClang.cmake)


# ----- PhysKit Library -----
set(PHYSKIT_SOURCES
	src/world.cpp
	src/mesh.cpp
	src/collision.cpp
	src/bvh.cpp
)

set(PHYSKIT_MODULE_SOURCES
	src/modules/physkit-decl.cpp
	src/modules/physkit.cpp
	src/modules/bvh_m.cpp
	src/modules/collision_m.cpp
	src/modules/mesh_m.cpp
	src/modules/world_m.cpp)

set(PHYSKIT_HEADERS
	physkit.h
	algebra/eigen_format.h
	algebra/lin_alg.h
	algebra/types.h
	collision/bounds.h
	collision/bvh.h
	collision/collision.h
	collision/collision_phases.h
	collision/constraint.h
	collision/mesh.h
	core/accum.h
	core/awaitables.h
	core/co.h
	core/object.h
	core/particle.h
	core/world.h
	detail/arena.h
	detail/integrate.h
)

list(TRANSFORM PHYSKIT_HEADERS PREPEND "include/physkit/")

option(PHYSKIT_MODULES "Build PhysKit as C++ modules" OFF)
option(PHYSKIT_BUILD_STATIC "Build PhysKit as a static library" OFF)

if (PHYSKIT_MODULES)
	set(CMAKE_CXX_SCAN_FOR_MODULES ON)
else()
	set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
endif()

if (PHYSKIT_BUILD_STATIC)
	add_library(physkit STATIC)
else()
	add_library(physkit SHARED)
endif()

target_sources(physkit
  	PUBLIC
    	FILE_SET HEADERS
    	BASE_DIRS include
    FILES
    	${PHYSKIT_HEADERS}
)

if (PHYSKIT_MODULES)
	set(MP_UNITS_BUILD_CXX_MODULES ON)

	target_sources(physkit
		PUBLIC
		FILE_SET modules TYPE CXX_MODULES
		FILES
			${PHYSKIT_MODULE_SOURCES}
	)
	target_compile_definitions(physkit PUBLIC PHYSKIT_MODULES)
	if (PHYSKIT_IMPORT_STD)
		target_compile_definitions(physkit PUBLIC PHYSKIT_IMPORT_STD)
		set(MP_UNITS_API_STD_FORMAT ON)
	endif()
else()
	target_sources(physkit PRIVATE ${PHYSKIT_SOURCES})
endif()

target_include_directories(physkit
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(physkit 
    PUBLIC PHYSKIT_VERSION="${PROJECT_VERSION}"
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	if(MSVC)
		target_compile_options(physkit PRIVATE /O2 /Oi /Ot)
	else()
		target_compile_options(physkit PRIVATE -O3)
	endif()
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	if (PHYSKIT_MODULES)
		target_compile_options(physkit PUBLIC -Winclude-angled-in-module-purview)
	endif()
endif()

find_package(mp-units REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(absl REQUIRED)

target_link_libraries(physkit
    PUBLIC mp-units::mp-units Eigen3::Eigen absl::flat_hash_map absl::flat_hash_set absl::inlined_vector
)

if (NOT PHYSKIT_MODULES)
	target_precompile_headers(physkit PRIVATE
		<Eigen/Dense>
		<Eigen/Geometry>
	)
else()
	set_source_files_properties(${PHYSKIT_SOURCES} ${PHYSKIT_MODULE_SOURCES} PROPERTIES SKIP_PRECOMPILE_HEADERS ON)
endif()

add_library(physkit::physkit ALIAS physkit)

set(PHYSKIT_LIBRARIES physkit)

option(PHYSKIT_BUILD_TESTS "Build tests and demos" OFF)
if (PHYSKIT_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()

include(${CMAKE_SOURCE_DIR}/cmake/FilterCompileCommands.cmake)

# ----- Documentation (Doxygen) -----
option(ENABLE_DOCS "Enable building API documentation" ON)
if(ENABLE_DOCS)
	find_package(Doxygen QUIET)
	if(DOXYGEN_FOUND)
		set(DOXYGEN_IN  ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
		set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

		configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

		add_custom_target(docs
			COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
			COMMENT "Generating API documentation with Doxygen"
			VERBATIM)

		message(STATUS "Docs: Run 'cmake --build . --target docs'")
	else()
		message(STATUS "Doxygen not found. `doc` target disabled.")
	endif()
endif()
