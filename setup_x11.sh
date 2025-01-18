#!/bin/bash
set -e

# Set up X11 for Docker
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:1.0
fi

echo "Setting up X11 forwarding for Docker..."
if ! xhost +local:docker > /dev/null 2>&1; then
    echo "Error: Failed to set X11 permissions. Is X11 running?"
    exit 1
fi

echo "✓ X11 forwarding has been set up for Docker"
echo "✓ Display set to: $DISPLAY"

# Run GUI tests if test script exists
if [ -f "scripts/test_gui.sh" ]; then
    echo -e "\nRunning GUI tests..."
    ./scripts/test_gui.sh
fi
