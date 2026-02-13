add_executable(bounce_demo
    bounce_demo.cpp
)

target_link_libraries(bounce_demo PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)

add_executable(config
    config_demo.cpp
)
target_link_libraries(config PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)