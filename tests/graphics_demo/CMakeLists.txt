add_executable(graphics_demo
    main.cpp
)

target_link_libraries(graphics_demo PRIVATE ${PHYSKIT_LIBRARIES} ${MAGNUM_LIBRARIES})