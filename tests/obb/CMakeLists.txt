add_executable(obb_tests
    obb_test.cpp
)

target_link_libraries(obb_tests PRIVATE phys_testing)

add_test(NAME obb_tests COMMAND obb_tests)
