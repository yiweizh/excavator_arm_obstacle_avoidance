#!/bin/bash -e
cd ./result
../build/segment/segment ${1}
../build/visualizer/visualizer ${2} ${1}
