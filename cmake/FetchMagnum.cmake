cmake_minimum_required(VERSION 3.24)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

include(FetchContent)

FetchContent_Declare(
    corrade
    GIT_REPOSITORY https://github.com/mosra/corrade.git
    GIT_TAG 5f66f3cbc47c14da57bb7014ffae76ab6f255b29 # arbitrary stable commit
    GIT_SHALLOW    FALSE
    GIT_PROGRESS   TRUE)

set(CORRADE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CORRADE_BUILD_STATIC ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(corrade)

FetchContent_Declare(
    magnum
    GIT_REPOSITORY https://github.com/mosra/magnum.git
    GIT_TAG 4d80751e6f31abdafbba2233fe4315fb7cbcf5a7 # arbitrary stable commit
    GIT_SHALLOW    FALSE
    GIT_PROGRESS   TRUE)

set(MAGNUM_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MAGNUM_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MAGNUM_BUILD_TOOLS    OFF CACHE BOOL "" FORCE)

set(MAGNUM_WITH_GL          ON  CACHE BOOL "" FORCE)
set(MAGNUM_WITH_SHADERS     ON  CACHE BOOL "" FORCE)
set(MAGNUM_WITH_PRIMITIVES  ON  CACHE BOOL "" FORCE)

set(MAGNUM_WINDOW_LIBRARY Magnum::GlfwApplication)
set(MAGNUM_WITH_GLFWAPPLICATION ON CACHE BOOL "" FORCE)

set(MAGNUM_BUILD_STATIC ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(magnum)

set(MAGNUM_LIBRARIES
    glfw
    Magnum::GL
    Magnum::Primitives
    Magnum::Shaders
    Magnum::SceneGraph
    OpenGL::GL
    ${MAGNUM_WINDOW_LIBRARY})