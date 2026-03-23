add_executable(co_tests
    co_tests.cpp
)

target_link_libraries(co_tests PRIVATE phys_testing)

add_test(NAME co_tests COMMAND co_tests)

if (PHYSKIT_BUILD_GRAPHICS_TESTS)
    add_executable(missile_demo
        missile_demo.cpp
    )

    target_link_libraries(missile_demo PRIVATE phys_graphics)

    add_test(NAME missile_demo COMMAND missile_demo --testing)
endif()
