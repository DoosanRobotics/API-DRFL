#!/bin/bash

# Navigate to the example directory
cd example/Linux_64 || { echo "Directory 'example/Linux_64' not found. Exiting."; exit 1; }

# Check if there are any files to clean
OBJ_FILES=(*.o)
OUT_DIR="./out"

FOUND_FILES=false

# Check for .o files
if [[ -f "${OBJ_FILES[0]}" ]]; then
    echo "Found object files:"
    for obj in "${OBJ_FILES[@]}"; do
        echo "  $obj"
    done
    FOUND_FILES=true
fi

# Check for out directory and executables
if [[ -d "$OUT_DIR" ]]; then
    EXECUTABLES=($(find "$OUT_DIR" -type f -executable))
    if [[ ${#EXECUTABLES[@]} -gt 0 ]]; then
        echo "Found executables in out/ directory:"
        for exe in "${EXECUTABLES[@]}"; do
            echo "  $(basename "$exe")"
        done
        FOUND_FILES=true
    fi
fi

if [[ "$FOUND_FILES" == false ]]; then
    echo "No build files found to clean."
    exit 0
fi

# Ask for confirmation
read -p "Do you want to clean all build files? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    # Remove .o files
    if [[ -f "${OBJ_FILES[0]}" ]]; then
        rm -f *.o
        echo "Removed object files."
    fi
    
    # Remove out directory
    if [[ -d "$OUT_DIR" ]]; then
        rm -rf "$OUT_DIR"
        echo "Removed out/ directory."
    fi
    
    echo "Clean completed."
else
    echo "Clean cancelled."
fi