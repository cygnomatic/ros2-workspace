#!/bin/bash

# Check if the remote named 'upstream' exists
git remote get-url upstream &>/dev/null

if [ $? -ne 0 ]; then
    # If 'upstream' doesn't exist, add it
    echo "Adding remote 'upstream' ..."
    git remote add upstream git@codeup.aliyun.com:cygnomatic/dev_ws/vscode_ros2_workspace.git
else
    # If it exists, fetch the latest changes from 'upstream'
    echo "Fetching the latest changes from 'upstream' ..."
    git fetch upstream
fi

# Check if you are on a branch (and not in detached HEAD state)
branch_name=$(git branch --show-current)
if [ -z "$branch_name" ]; then
    echo "You are not on any branch, aborting merge."
    exit 1
fi

# Merge upstream/main into your current branch
echo "Merging 'upstream/main' into the current branch: $branch_name ..."
git merge upstream/main

# Handle merge conflicts if they exist
merge_status=$?
if [ $merge_status -ne 0 ]; then
    echo "Merge conflicts detected. Please resolve them before proceeding."
    exit $merge_status
fi

echo "Merge completed successfully."
