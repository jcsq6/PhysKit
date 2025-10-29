if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    get_filename_component(_clang_bin_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    get_filename_component(_clang_root    "${_clang_bin_dir}/.." REALPATH)
    set(_libcxx_dir "${_clang_root}/lib/c++")

    if(EXISTS "${_libcxx_dir}/libc++.dylib")
        message(STATUS "PhysKit: using libc++ from ${_libcxx_dir}")
        add_link_options(
            "-L${_libcxx_dir}"
            "-Wl,-rpath,${_libcxx_dir}"
        )
    else()
        message(WARNING
            "PhysKit: could not find libc++ next to ${CMAKE_CXX_COMPILER}. "
            "You may hit undefined symbols at link/run time.")
    endif()
endif()