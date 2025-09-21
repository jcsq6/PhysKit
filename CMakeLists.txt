cmake_minimum_required(VERSION 3.21)

project(PhysKit VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----- PhysKit Library -----
set(PHYSKIT_SOURCES 
	src/physkit.cpp
)

add_library(physkit ${PHYSKIT_SOURCES})
target_include_directories(physkit
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(physkit::physkit ALIAS physkit)

target_compile_definitions(physkit 
    PUBLIC PHYSKIT_VERSION="${PROJECT_VERSION}"
)

set(PHYSKIT_LIBRARIES physkit)

add_subdirectory(tests)