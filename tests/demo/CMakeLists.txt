add_executable(config
    config_demo.cpp
)
target_link_libraries(config PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)