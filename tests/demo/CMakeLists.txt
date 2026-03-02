add_executable(config
    config_demo.cpp
)
target_link_libraries(config PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)

add_test(NAME config_demo COMMAND config -c ${CMAKE_CURRENT_SOURCE_DIR}/../configs/ball_box_pyr.json --testing)