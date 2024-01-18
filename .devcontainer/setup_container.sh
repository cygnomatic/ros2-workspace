#!/bin/bash

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

ln -sf "$(pwd)/.devcontainer/docker-compose.gpu-${HAS_GPU}.yaml" "$(pwd)/.devcontainer/docker-compose.gpu.yaml"
ln -sf "$(pwd)/.devcontainer/docker-compose.os-${OS}.yaml" "$(pwd)/.devcontainer/docker-compose.os.yaml"
