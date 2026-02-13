add_executable(bounce_demo
    bounce_demo.cpp
)

target_link_libraries(bounce_demo PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)

add_executable(config
    config_demo.cpp
)
target_link_libraries(config PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)

add_test(NAME bounce_demo COMMAND bounce_demo --testing)
add_test(NAME config_demo COMMAND config -c ${CMAKE_CURRENT_SOURCE_DIR}/../configs/ball_box_pyr.json --testing)