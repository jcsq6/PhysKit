add_executable(pool
    pong_demo.cpp
)
target_link_libraries(pool PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)
