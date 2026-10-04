#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ARDUINO_FQBN="${ARDUINO_FQBN:-arduino:avr:mega}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyACM0}"
FIRMWARE_DIR="${FIRMWARE_DIR:-firmware/mega}"
CONTROLLER_DIR="${CONTROLLER_DIR:-controller}"
BUILD_PROFILE="${BUILD_PROFILE:-release}"
SERVICE_NAME="${SERVICE_NAME:-}"

resolve_from_root() {
    local path="$1"

    if [[ "${path}" = /* ]]; then
        printf '%s\n' "${path}"
    else
        printf '%s\n' "${REPO_ROOT}/${path}"
    fi
}

CONTROLLER_PATH="$(resolve_from_root "${CONTROLLER_DIR}")"
FIRMWARE_PATH="$(resolve_from_root "${FIRMWARE_DIR}")"

printf '[1/5] Building Rust controller (profile: %s)...\n' "${BUILD_PROFILE}"
cargo build \
    --manifest-path "${CONTROLLER_PATH}/Cargo.toml" \
    --profile "${BUILD_PROFILE}"

printf '[2/5] Compiling Arduino firmware (board: %s)...\n' "${ARDUINO_FQBN}"
arduino-cli compile \
    --fqbn "${ARDUINO_FQBN}" \
    "${FIRMWARE_PATH}"

if [[ -n "${SERVICE_NAME}" ]]; then
    printf '[3/5] Stopping systemd service: %s...\n' "${SERVICE_NAME}"
    systemctl stop "${SERVICE_NAME}"
else
    printf '[3/5] No systemd service configured; skipping stop.\n'
fi

printf '[4/5] Uploading Arduino firmware to %s...\n' "${ARDUINO_PORT}"
arduino-cli upload \
    --port "${ARDUINO_PORT}" \
    --fqbn "${ARDUINO_FQBN}" \
    "${FIRMWARE_PATH}"

if [[ -n "${SERVICE_NAME}" ]]; then
    printf '[5/5] Restarting systemd service: %s...\n' "${SERVICE_NAME}"
    systemctl restart "${SERVICE_NAME}"
else
    printf '[5/5] No systemd service configured; skipping start.\n'
fi

printf 'Deployment completed successfully.\n'
