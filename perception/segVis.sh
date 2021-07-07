#!/bin/bash -e
mkdir segResult
cd ./segResult
../build/segment/segment ${1}
../build/visualizer/visualizer ${2} ${1}
