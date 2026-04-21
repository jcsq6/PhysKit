add_executable(pool
    pool_demo.cpp
)
target_link_libraries(pool PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)
