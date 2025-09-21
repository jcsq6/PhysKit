add_executable(demo
    main.cpp
)

target_link_libraries(demo PRIVATE ${PHYSKIT_LIBRARIES})