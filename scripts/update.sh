#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

printf '[1/2] Updating repository (fast-forward only)...\n'
git -C "${REPO_ROOT}" pull --ff-only

printf '[2/2] Deploying controller and firmware...\n'
"${SCRIPT_DIR}/deploy.sh"
