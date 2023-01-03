#!/bin/sh

for file in "$@"; do
    # strip trailing whitespace
    sed -i -e 's/[[:space:]]*$//g' "$file"

    # remove spaces before tabs
    sed -i -e 's/^ *\t/\t/g' "$file"

    # strip leading blank lines
    sed -i -e '/./,$!d' "$file"

    # strip trailing blank lines
    sed -i -e ':a' -e '/^\n*$/{$d;N;};/\n$/ba' "$file"

    # add final newline
    sed -i -e '$a\' "$file"
done
