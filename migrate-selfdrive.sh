#!/bin/bash

./git-pullall.sh
git lfs migrate import --everything --above=5MB
git remote set-url origin git@github.com:CooperUnion/selfdrive-migration-staging.git
git push --all -f
