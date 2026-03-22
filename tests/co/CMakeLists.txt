add_executable(co_tests
    co_tests.cpp
)

target_link_libraries(co_tests PRIVATE phys_testing)

add_test(NAME co_tests COMMAND co_tests)
