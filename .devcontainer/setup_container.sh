#!/bin/bash

echo ""
echo -e "\e[1mSetting up Dev Container ...\e[0m"

if command -v nvidia-smi &> /dev/null; then
    HAS_GPU="enable"
else
    HAS_GPU="disable"
fi

if [[ $(grep microsoft /proc/version) ]]; then
    OS="win"
else
    OS="unix"
fi

echo -e "Active container variant: GPU = \e[1m${HAS_GPU}\e[0m, OS = \e[1m${OS}\e[0m"
cp -f "$(pwd)/.devcontainer/docker-compose.gpu-${HAS_GPU}.yaml" "$(pwd)/.devcontainer/docker-compose.gpu.yaml"
cp -f "$(pwd)/.devcontainer/docker-compose.os-${OS}.yaml" "$(pwd)/.devcontainer/docker-compose.os.yaml"

echo ""
echo -e "\e[1mBuilding Dev Container ...\e[0m"
echo "Click the blue text 'Starting Dev Container (show log)' at the bottom right dialog to open build log."
