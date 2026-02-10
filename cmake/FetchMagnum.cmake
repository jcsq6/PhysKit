cmake_minimum_required(VERSION 3.24)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

include(FetchContent)

FetchContent_Declare(
    corrade
    GIT_REPOSITORY https://github.com/mosra/corrade.git
    GIT_TAG ef37b2a35a149d29854bf30240100e0d924af4a9 # arbitrary stable commit
    GIT_SHALLOW    FALSE
    GIT_PROGRESS   TRUE)

set(CORRADE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CORRADE_BUILD_STATIC ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(corrade)

FetchContent_Declare(
    magnum
    GIT_REPOSITORY https://github.com/mosra/magnum.git
    GIT_TAG f3a4ce7d1d0cd8085d4f05811c378813ada3cfcc # arbitrary stable commit
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