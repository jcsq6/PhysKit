add_executable(sandbox
    sandbox.cpp
)
target_link_libraries(sandbox PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)
