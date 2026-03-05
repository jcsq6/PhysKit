add_executable(epa_tests
    epa_test.cpp
)

target_link_libraries(epa_tests PRIVATE phys_testing)

add_test(NAME epa_tests COMMAND epa_tests)
