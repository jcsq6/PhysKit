# FilterCompileCommands.cmake
# Filters compile_commands.json to exclude build/_deps/
# Runs automatically on Windows (where clangd crashes on _deps), or when PHYSKIT_FILTER_COMPILE_COMMANDS is explicitly set to ON.

option(PHYSKIT_FILTER_COMPILE_COMMANDS "Filter _deps from compile_commands.json" OFF)

if(CMAKE_EXPORT_COMPILE_COMMANDS AND (WIN32 OR PHYSKIT_FILTER_COMPILE_COMMANDS))
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_FOUND)
        file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/filter_compile_commands.py" CONTENT [[
import json, sys, os

path = sys.argv[1]
if not os.path.exists(path):
    sys.exit(0)

with open(path) as f:
    commands = json.load(f)

filtered = [e for e in commands if "/_deps/" not in e.get("file", "").replace("\\", "/")]

with open(path, "w") as f:
    json.dump(filtered, f, indent=2)

print(f"compile_commands.json: kept {len(filtered)}/{len(commands)} entries (excluded _deps)")
]])
        add_custom_target(filter-compile-commands ALL
            COMMAND ${Python3_EXECUTABLE} "${CMAKE_BINARY_DIR}/filter_compile_commands.py"
                    "${CMAKE_BINARY_DIR}/compile_commands.json"
            COMMENT "Filtering _deps entries from compile_commands.json"
            VERBATIM
        )
    else()
        message(STATUS "Python3 not found - compile_commands.json will not be filtered")
    endif()
endif()
