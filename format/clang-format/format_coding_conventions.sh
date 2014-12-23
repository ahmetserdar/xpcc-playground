#!/usr/bin/env bash

# go to the clang-format dir because this is where the .clang-format file is
# located
cd "$( dirname "${BASH_SOURCE[0]}" )"

# format files
clang-format -style=file ../input/coding_conventions.hpp > ../output/coding_conventions.hpp
clang-format -style=file ../input/coding_conventions.cpp > ../output/coding_conventions.cpp

#
echo "To check the output use:"
echo "> meld input output"
