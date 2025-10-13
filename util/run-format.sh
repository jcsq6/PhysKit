#!/bin/bash
find . -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' | grep -v build/ | clang-format --style=file -i --files=/dev/stdin