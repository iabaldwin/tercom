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
popd
