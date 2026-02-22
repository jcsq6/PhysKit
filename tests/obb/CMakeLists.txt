add_executable(obb_tests
    obb_test.cpp
)

target_link_libraries(obb_tests PRIVATE ${PHYSKIT_LIBRARIES})