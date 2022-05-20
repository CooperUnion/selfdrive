#!/bin/bash

echo -e "--- Cooper IGVC Setup Tool ---"
cd "$(dirname "${BASH_SOURCE[0]}")"

echo -e "-> Gathering data..."
parent_proc=$(ps -ho comm $(ps -ho ppid $$))

echo -e ""
echo -e "PARENT SHELL: $parent_proc"
echo -e ""

echo -e "-> Preparing system dependencies...\n"
if [ -x "$(command -v apt-get)" ]; then
    sudo apt-get install -y -qq \
        python3.9 \
        python3.9-venv \
        can-utils \
        fish \
        redis
elif [ -x "$(command -v dnf)" ]; then
    sudo dnf install -y -q \
        python3.9 \
        can-utils \
        fish \
        redis
else
    echo -e "---        WARNING: Package manager not found.        ---"
    echo -e "--- You need to manually install system dependencies. ---"
fi

echo -e "\n-> Preparing git submodules...\n"
git submodule update --init --recursive

echo -e "\n-> Removing any old .venv...\n"
rm -rf .venv

echo -e "\n-> Creating new venv...\n"
python3.9 -m venv .venv

echo -e "\n-> Sourcing new venv...\n"
source .venv/bin/activate

echo -e "\n-> Running 'make clean'...\n"
make clean

echo -e "\n-> Running 'make dependencies'...\n"
rm -f .install-dependencies
make dependencies

