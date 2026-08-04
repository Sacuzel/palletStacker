#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_DIR="${HOME}/.gz/gui/plugins"

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --parallel
mkdir -p "${INSTALL_DIR}"
cp "${BUILD_DIR}/libForkliftTeleop.so" "${INSTALL_DIR}/"

echo "Installed ${INSTALL_DIR}/libForkliftTeleop.so"
echo "Gazebo can now load <plugin filename=\"ForkliftTeleop\"> from the world SDF."
