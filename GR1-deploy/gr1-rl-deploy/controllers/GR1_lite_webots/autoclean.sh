#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build/
#build main
pushd ${build_dir}
sudo rm -r -f ./*

popd