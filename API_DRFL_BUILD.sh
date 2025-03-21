#!/bin/bash

# Detect the Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
ARCHITECTURE=$(uname -m)

# Navigate to the example directory
cd example/Linux_64 || { echo "Directory 'example/Linux_64' not found. Exiting."; exit 1; }

# Create the out/ directory if it doesn't exist
OUT_DIR="./out"
mkdir -p "$OUT_DIR"

# Check if drfl_test already exists in the out/ directory
if [[ -f "$OUT_DIR/drfl_test" ]]; then
    echo "Executable 'drfl_test' already exists in the out/ directory."
    read -p "Do you want to overwrite it? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Build process skipped."
        exit 0
    fi
else
    echo "'drfl_test' does not exist in the out/ directory."
fi

# Compile main.cpp to main.o
echo "Compiling main.cpp..."
g++ -c main.cpp
if [[ $? -ne 0 ]]; then
    echo "Compilation failed. Exiting."
    exit 1
fi
echo "Compiled successfully to main.o."

# Set the correct library path based on Ubuntu version and architecture
LIBRARY_PATH="../../library/Linux/64bits"
if [[ "$ARCHITECTURE" == "x86_64" ]]; then
    if [[ "$UBUNTU_VERSION" == "18.04" ]]; then
        LIBRARY_PATH+="/amd64/18.04"
    elif [[ "$UBUNTU_VERSION" == "20.04" || "$UBUNTU_VERSION" == "22.04" ]]; then
        LIBRARY_PATH+="/amd64/${UBUNTU_VERSION}"
    else
        echo "Unsupported Ubuntu version for x86_64."
        exit 1
    fi
elif [[ "$ARCHITECTURE" == "aarch64" ]]; then
    if [[ "$UBUNTU_VERSION" == "18.04" ]]; then
        LIBRARY_PATH+="/arn64/18.04"
    elif [[ "$UBUNTU_VERSION" == "20.04" || "$UBUNTU_VERSION" == "22.04" ]]; then
        LIBRARY_PATH+="/arn64/${UBUNTU_VERSION}"
    else
        echo "Unsupported Ubuntu version for arm64."
        exit 1
    fi
else
    echo "Unsupported architecture: $ARCHITECTURE"
    exit 1
fi

# Find the appropriate Poco library versions and directories
FOUNDATION_LIB=$(find "${LIBRARY_PATH}" -name "libPocoFoundation.so.*" | head -n 1)
NET_LIB=$(find "${LIBRARY_PATH}" -name "libPocoNet.so.*" | head -n 1)
FOUNDATION_DIR=$(dirname "$FOUNDATION_LIB")
NET_DIR=$(dirname "$NET_LIB")

if [[ -z "$FOUNDATION_LIB" || -z "$NET_LIB" ]]; then
    echo "Poco libraries not found in the expected path. Exiting."
    exit 1
fi

# Link libraries and generate the executable in the out/ directory
echo "Linking and creating the executable in the out/ directory..."
g++ -o "$OUT_DIR/drfl_test" main.o "${LIBRARY_PATH}/libDRFL.a" "$FOUNDATION_LIB" "$NET_LIB"
if [[ $? -ne 0 ]]; then
    echo "Linking failed. Exiting."
    exit 1
fi
echo "Executable drfl_test created successfully in the out/ directory."

# Set LD_LIBRARY_PATH to include Poco libraries' directories if they are not already included
[[ ":$LD_LIBRARY_PATH:" != *":$FOUNDATION_DIR:"* ]] && export LD_LIBRARY_PATH="$FOUNDATION_DIR:$LD_LIBRARY_PATH"
[[ ":$LD_LIBRARY_PATH:" != *":$NET_DIR:"* ]] && export LD_LIBRARY_PATH="$NET_DIR:$LD_LIBRARY_PATH"

# Ask if the user wants to run the executable
read -p "Do you want to run drfl_test now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    "$OUT_DIR/drfl_test"
else
    echo "Execution skipped."
fi