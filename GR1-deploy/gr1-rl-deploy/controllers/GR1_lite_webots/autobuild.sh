#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build/
#build main
pushd ${build_dir}
# sudo rm -r -f ./*
cmake -DCMAKE_BUILD_TYPE=Release \
    -Dabsl_DIR=$root/ThirdParty/grpc/x86_64/lib/cmake/absl \
    -DProtobuf_DIR=$root/ThirdParty/grpc/x86_64/lib/cmake/protobuf \
    -DgRPC_DIR=$root/ThirdParty/grpc/x86_64/lib/cmake/grpc ..
make -j8
popd