#!/usr/bin/env bash
set -e

echo "Building project..."

echo "Building Container"
docker build -t esp-owb .

echo "Building Image"
docker run --rm -it \
  -v "$(pwd)":/owb \
  -w /owb \
  esp-owb \
  cargo build -q --release

echo "Build complete!"
