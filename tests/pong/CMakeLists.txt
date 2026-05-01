add_executable(pong
    pong_demo.cpp
)
target_link_libraries(pong PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)
