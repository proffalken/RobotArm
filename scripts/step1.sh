#!/bin/bash

set -x

echo "Build the container"

cd workspace/src/docker
otel-cli exec --service github_actions --name "Robot Arm - Container Build Script" ./build.sh
