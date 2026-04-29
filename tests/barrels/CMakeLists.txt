add_executable(barrels
        barrels.cpp
)
target_link_libraries(barrels PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)