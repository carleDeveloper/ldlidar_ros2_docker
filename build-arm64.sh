#!/usr/bin/env bash
# Cross-build the arm64 image for the Arduino UNO Q from an x86_64 host and
# export it as a tarball you can copy to the board and `docker load`.
#
# Prereqs (one-time, privileged): register QEMU emulation for arm64:
#   docker run --privileged --rm tonistiigi/binfmt --install arm64
#
# On the board itself you do NOT need this script; just run:
#   docker compose build
set -euo pipefail

IMAGE="ldlidar_stl_ros2:jazzy-arm64"
OUT="ldlidar_stl_ros2_jazzy_arm64.tar"
BUILDER="ldlidar-arm64"

# Create a container-driver builder (supports multi-arch) if missing.
if ! docker buildx inspect "${BUILDER}" >/dev/null 2>&1; then
  docker buildx create --name "${BUILDER}" --driver docker-container --use
else
  docker buildx use "${BUILDER}"
fi

# Build for arm64 and load the result into the local docker image store.
docker buildx build \
  --platform linux/arm64 \
  --tag "${IMAGE}" \
  --load \
  .

# Export to a tarball for transfer to the UNO Q.
docker save "${IMAGE}" -o "${OUT}"
echo "Wrote ${OUT}"
echo "Copy to the board and load with:  docker load -i ${OUT}"
