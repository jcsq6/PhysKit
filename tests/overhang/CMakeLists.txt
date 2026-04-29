add_executable(overhang
    overhang_demo.cpp
)

target_link_libraries(overhang PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)
