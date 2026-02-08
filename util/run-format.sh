#!/bin/bash
find . -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' | grep -v build/ | xargs clang-format --style=file -i