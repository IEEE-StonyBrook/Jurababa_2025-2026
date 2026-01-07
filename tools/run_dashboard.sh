#!/bin/bash
# Run the MotorLab Dashboard

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use system Python with user packages (more reliable on macOS)
/usr/bin/python3 "$SCRIPT_DIR/motorlab_dashboard.py"
