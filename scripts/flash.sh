#!/bin/bash
# SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
# SPDX-License-Identifier: MIT
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
if [ ! -d /workspace ]; then
    cd ${SCRIPT_PATH}/../toolchain/esp-idf/
    . ./export.sh
fi
esptool.py --baud 1152000 write_flash -z 0x0000 /tmp/firmware.bin