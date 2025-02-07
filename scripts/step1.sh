#!/bin/bash

set -x

echo "Build the container"

cd workspace/src/docker
./build.sh
