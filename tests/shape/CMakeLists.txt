add_executable(shape_tests
    main.cpp
)

target_link_libraries(shape_tests PRIVATE phys_testing)

add_test(NAME shape_tests COMMAND shape_tests)
