add_executable(gjk_tests
    gjk_test.cpp
)

target_link_libraries(gjk_tests PRIVATE phys_testing)

add_test(NAME gjk_tests COMMAND gjk_tests)
