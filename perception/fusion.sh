#!/bin/bash -e
mkdir fusionResult
cd ./fusionResult
../build/pc_fusion/pc_fusion ../dataset/${1} ../dataset/${2}