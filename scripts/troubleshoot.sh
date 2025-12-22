#!/bin/bash
# SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
# SPDX-License-Identifier: MIT
rm -f log.txt || true
timeout --foreground  5s idf.py -p /dev/ttyUSB0 monitor 1> log.txt
reset
grep "Backtrace: " log.txt
