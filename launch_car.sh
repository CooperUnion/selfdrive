#!/bin/bash

set -e

parent_proc=$(ps -ho comm $(ps -ho ppid $$))

echo -e "\n-> Dropping you into a new shell...\n"
if [ "$parent_proc" == "fish" ]; then
    fish --init-command="source .venv/bin/activate.fish"
else
    if [ "$parent_proc" != "bash" ]; then
        echo -e "\nUnrecognized shell '$usershell'"
    fi
    bash --init-file <(echo -e "source ~/.bashrc; source .venv/bin/activate")
fi
