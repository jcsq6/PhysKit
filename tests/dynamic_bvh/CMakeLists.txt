add_executable(dynamic_bvh_tests
    main.cpp
)

target_link_libraries(dynamic_bvh_tests PRIVATE phys_testing)

add_test(NAME dynamic_bvh_tests COMMAND dynamic_bvh_tests)
