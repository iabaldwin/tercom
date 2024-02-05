#!/bin/bash 

set -euo pipefail

pushd thirdparty/
  if [[ ! -d raylib ]]; then
    echo "Downloading raylib..."
    git clone https://github.com/raysan5/raylib.git
  fi
  pushd raylib
    if [[ ! -d build ]]; then
      mkdir build
      pushd build
        ccmake ..
        make -j
      popd
    fi
  popd

  if [[ ! -f bazelisk/bazel ]]; then
    mkdir -p bazelisk
    echo "Downloading bazelisk..."
    wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64 -O bazelisk/bazel
    chmod a+x bazelisk/bazel
  fi
popd
