#!/bin/bash

# Detect the Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
ARCHITECTURE=$(uname -m)

# Navigate to the example directory
cd example/Linux_64 || { echo "Directory 'example/Linux_64' not found. Exiting."; exit 1; }

# Find all .cpp files in the current directory
CPP_FILES=(*.cpp)
if [[ ${#CPP_FILES[@]} -eq 0 || ! -f "${CPP_FILES[0]}" ]]; then
    echo "No .cpp files found in example/Linux_64 directory. Exiting."
    exit 1
fi

# Display available .cpp files
echo "Available .cpp files:"
for i in "${!CPP_FILES[@]}"; do
    echo "$((i+1)). ${CPP_FILES[i]}"
done

# Get user selection
while true; do
    read -p "Select a file to build (1-${#CPP_FILES[@]}): " choice
    if [[ "$choice" =~ ^[0-9]+$ ]] && [[ "$choice" -ge 1 ]] && [[ "$choice" -le ${#CPP_FILES[@]} ]]; then
        SELECTED_FILE="${CPP_FILES[$((choice-1))]}"
        break
    else
        echo "Invalid selection. Please enter a number between 1 and ${#CPP_FILES[@]}."
    fi
done

echo "Selected file: $SELECTED_FILE"

# Create the out/ directory if it doesn't exist
OUT_DIR="./out"
mkdir -p "$OUT_DIR"

# Generate executable name from cpp file name
EXE_NAME="${SELECTED_FILE%.cpp}"
EXE_PATH="$OUT_DIR/$EXE_NAME"

# Check if executable already exists
if [[ -f "$EXE_PATH" ]]; then
    echo "Executable '$EXE_NAME' already exists in the out/ directory."
    read -p "Do you want to overwrite it? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Build process skipped."
        exit 0
    fi
fi

# Compile selected cpp file
echo "Compiling $SELECTED_FILE..."
g++ -c "$SELECTED_FILE"
if [[ $? -ne 0 ]]; then
    echo "Compilation failed. Exiting."
    exit 1
fi
OBJ_FILE="${SELECTED_FILE%.cpp}.o"
echo "Compiled successfully to $OBJ_FILE."

# Set the correct library path based on Ubuntu version and architecture
LIBRARY_PATH="../../library/Linux/64bits"
if [[ "$ARCHITECTURE" == "x86_64" ]]; then
    if [[ "$UBUNTU_VERSION" == "18.04" || "$UBUNTU_VERSION" == "20.04" || "$UBUNTU_VERSION" == "22.04" || "$UBUNTU_VERSION" == "24.04" ]]; then
        LIBRARY_PATH+="/amd64/${UBUNTU_VERSION}"
    else
        echo "Unsupported Ubuntu version for x86_64."
        exit 1
    fi
elif [[ "$ARCHITECTURE" == "aarch64" ]]; then
    if [[ "$UBUNTU_VERSION" == "18.04" || "$UBUNTU_VERSION" == "20.04" || "$UBUNTU_VERSION" == "22.04" || "$UBUNTU_VERSION" == "24.04" ]]; then
        LIBRARY_PATH+="/arm64/${UBUNTU_VERSION}"
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
g++ -o "$EXE_PATH" "$OBJ_FILE" "${LIBRARY_PATH}/libDRFL.a" "$FOUNDATION_LIB" "$NET_LIB"
if [[ $? -ne 0 ]]; then
    echo "Linking failed. Exiting."
    exit 1
fi
echo "Executable $EXE_NAME created successfully in the out/ directory."

# Set LD_LIBRARY_PATH to include Poco libraries' directories if they are not already included
[[ ":$LD_LIBRARY_PATH:" != *":$FOUNDATION_DIR:"* ]] && export LD_LIBRARY_PATH="$FOUNDATION_DIR:$LD_LIBRARY_PATH"
[[ ":$LD_LIBRARY_PATH:" != *":$NET_DIR:"* ]] && export LD_LIBRARY_PATH="$NET_DIR:$LD_LIBRARY_PATH"

# Ask if the user wants to run the executable
read -p "Do you want to run $EXE_NAME now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    "$EXE_PATH"
else
    echo "Execution skipped."
fi