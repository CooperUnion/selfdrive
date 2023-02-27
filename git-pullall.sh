#!/bin/bash

for branch in $(git branch --all | grep '^\s*remotes' | egrep --invert-match '(:?HEAD|master)$'); do
    git branch --track ${branch#remotes/origin/} $branch
done

git fetch --all
git pull --all
