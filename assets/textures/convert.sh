#!/bin/bash

# Texture conversion script - part of the texture module
# Converts PNG files to raw RGBA format for the game engine
# Usage: ./assets/textures/convert.sh [input_dir] [output_dir]

set -e

INPUT_DIR="${1:-assets/textures/png}"
OUTPUT_DIR="${2:-assets/textures}"

echo "Converting textures from $INPUT_DIR to $OUTPUT_DIR"

# Check if ImageMagick is installed
if ! command -v magick &> /dev/null; then
    echo "Error: ImageMagick is not installed."
    echo "Install with: brew install imagemagick (macOS) or apt install imagemagick (Linux)"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Convert all PNG files to RGBA
converted_count=0
for png_file in "$INPUT_DIR"/*.png; do
    if [ -f "$png_file" ]; then
        base_name=$(basename "$png_file" .png)
        output_file="$OUTPUT_DIR/${base_name}.rgba"
        
        echo "Converting: $base_name.png -> $base_name.rgba"
        
        # Convert PNG to raw RGBA format
        magick "$png_file" -depth 8 -format rgba "$output_file"
        
        # Get dimensions for verification
        dimensions=$(magick identify -format "%wx%h" "$png_file")
        echo "  Dimensions: $dimensions"
        
        ((converted_count++))
    fi
done

if [ $converted_count -eq 0 ]; then
    echo "No PNG files found in $INPUT_DIR"
    echo "Create some PNG files there first, then run this script again."
else
    echo "Successfully converted $converted_count texture(s)"
fi

echo "Done!"