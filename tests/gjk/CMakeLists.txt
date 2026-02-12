add_executable(gjk_tests
    gjk_test.cpp
)

target_link_libraries(gjk_tests PRIVATE ${PHYSKIT_LIBRARIES})