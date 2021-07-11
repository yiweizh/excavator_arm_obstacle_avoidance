#!/bin/bash -e
mkdir segResult
cd ./segResult
../build/segment/segment ${1} ${2}
# ../build/visualizer/visualizer ${2} ${1}
