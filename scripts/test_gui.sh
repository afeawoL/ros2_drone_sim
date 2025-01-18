#!/bin/bash

# Colors for output
GREEN="[0;32m"
RED="[0;31m"
NC="[0m"

# Check for required packages
packages=("xeyes" "glxgears" "rviz2")
missing=()

for pkg in "${packages[@]}"; do
    if ! command -v $pkg >/dev/null 2>&1; then
        missing+=($pkg)
    fi
done

if [ ${#missing[@]} -ne 0 ]; then
    echo -e "${RED}Error: Missing required packages: ${missing[*]}${NC}"
    echo "Please install the missing packages and try again"
    exit 1
fi

echo -e "${GREEN}Starting X11 and ROS2 GUI tests...${NC}"

# Test 1: Basic X11
echo -n "Testing basic X11 forwarding with xeyes... "
if timeout 5 xeyes >/dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo -e "${RED}Please check:${NC}"
    echo " - X11 is running (run 'xhost' to verify)"
    echo " - DISPLAY variable is set correctly (current: $DISPLAY)"
    echo " - setup_x11.sh has been run successfully"
    fi

# Test 2: OpenGL
echo -n "Testing OpenGL support with glxgears... "
if timeout 5 glxgears >/dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "OpenGL support might not be working correctly"
fi

# Test 3: RViz2
echo -n "Testing RViz2... "
if timeout 5 rviz2 >/dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "RViz2 failed to start. Check ROS2 installation and X11 setup"
fi

echo "Test complete. If any tests failed, please check the troubleshooting section in README.md"
