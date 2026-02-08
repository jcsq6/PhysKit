add_executable(solar_system
    solar_system.cpp
)

target_link_libraries(solar_system PRIVATE phys_graphics)

add_executable(track_demo
    track_demo.cpp
)
target_link_libraries(track_demo PRIVATE phys_graphics)