add_executable(config
    config_demo.cpp
)
target_link_libraries(config PRIVATE ${PHYSKIT_LIBRARIES} phys_graphics)

get_directory_property(TESTS_CWD PARENT_DIRECTORY)
add_test(NAME billards WORKING_DIRECTORY ${TESTS_CWD} COMMAND config -c configs/billiards.json --testing)
add_test(NAME collisions WORKING_DIRECTORY ${TESTS_CWD} COMMAND config -c configs/collisions.json --testing)
add_test(NAME dominoes WORKING_DIRECTORY ${TESTS_CWD} COMMAND config -c configs/dominoes.json --testing)
add_test(NAME stacking_blocks WORKING_DIRECTORY ${TESTS_CWD} COMMAND config -c configs/stacking_blocks.json --testing)
add_test(NAME tower_assault WORKING_DIRECTORY ${TESTS_CWD} COMMAND config -c configs/tower_assault.json --testing)