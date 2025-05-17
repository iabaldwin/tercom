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
        cmake .. -DCMAKE_BUILD_TYPE=Release
        make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
      popd
    fi
  popd

  if [[ ! -f bazelisk/bazel ]]; then
    mkdir -p bazelisk
    echo "Downloading bazelisk..."
    platform=$(uname -s)
    arch=$(uname -m)
    case "${platform}" in
      Darwin)
        if [[ "${arch}" == "arm64" ]]; then
          file="bazelisk-darwin-arm64"
        else
          file="bazelisk-darwin-amd64"
        fi
        ;;
      Linux)
        file="bazelisk-linux-amd64"
        ;;
      *)
        echo "Unsupported platform: ${platform}"
        exit 1
        ;;
    esac
    url="https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/${file}"
    if command -v wget >/dev/null 2>&1; then
      wget "${url}" -O bazelisk/bazel
    else
      curl -L "${url}" -o bazelisk/bazel
    fi
    chmod a+x bazelisk/bazel
  fi
popd
