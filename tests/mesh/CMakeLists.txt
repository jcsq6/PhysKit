add_executable(mesh_tests
    main.cpp
)

target_link_libraries(mesh_tests PRIVATE ${PHYSKIT_LIBRARIES})

add_test(NAME mesh_tests COMMAND mesh_tests)