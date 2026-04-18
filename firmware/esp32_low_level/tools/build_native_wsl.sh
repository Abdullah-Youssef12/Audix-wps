#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${HOME}/audix_esp32_build"
PIO_BIN="${HOME}/.local/bin/pio"

if [[ ! -x "${PIO_BIN}" ]]; then
  echo "PlatformIO was not found at ${PIO_BIN}" >&2
  exit 1
fi

git config --global core.autocrlf false

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cp -a "${FIRMWARE_DIR}/." "${BUILD_DIR}/"

cd "${BUILD_DIR}"
"${PIO_BIN}" run
