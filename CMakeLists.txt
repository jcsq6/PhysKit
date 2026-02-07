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
	src/physkit.cpp
	src/mesh.cpp
)

add_library(physkit ${PHYSKIT_SOURCES})

target_include_directories(physkit
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(physkit 
    PUBLIC PHYSKIT_VERSION="${PROJECT_VERSION}"
)

set(MP_UNITS_BUILD_CXX_MODULES ON)
set(MP_UNITS_API_STD_FORMAT ON)
find_package(mp-units REQUIRED)

find_package(Eigen3 REQUIRED)

target_link_libraries(physkit
    PUBLIC mp-units::mp-units Eigen3::Eigen
)

add_library(physkit::physkit ALIAS physkit)

set(PHYSKIT_LIBRARIES physkit)

add_subdirectory(tests)

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