#!/bin/bash

CLANG_FORMAT=$(compgen -c clang-format | grep -E '^clang-format(-[0-9]+)?$' | sort -t- -k3 -n -r | head -1)

if [ -z "$CLANG_FORMAT" ]; then
    echo "Error: no clang-format found" >&2
    exit 1
fi

echo "Using $CLANG_FORMAT"
find . -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' | grep -v build/ | xargs "$CLANG_FORMAT" --style=file -i