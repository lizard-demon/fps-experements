#!/bin/bash

# Audio conversion script - part of the audio module
# Converts any audio format to mono 44.1kHz 16-bit raw PCM
# Usage: ./assets/audio/convert.sh <input_file> [output_file]
#        ./assets/audio/convert.sh --batch <input_dir> [output_dir]

set -e

show_help() {
    echo "Audio conversion script for game engine"
    echo "Converts audio to mono 44.1kHz 16-bit raw PCM"
    echo ""
    echo "Usage:"
    echo "  $0 <input_file> [output_file]     - Convert single file"
    echo "  $0 --batch <input_dir> [output_dir] - Convert all files in directory"
    echo "  $0 -h, --help                    - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 music.wav music.pcm"
    echo "  $0 --batch assets/audio/wav assets/audio"
}

if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

# Check if ffmpeg is available
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found. Install with: brew install ffmpeg (macOS) or apt install ffmpeg (Linux)"
    exit 1
fi

# Batch conversion mode
if [ "$1" = "--batch" ]; then
    if [ $# -lt 2 ]; then
        echo "Error: --batch requires input directory"
        show_help
        exit 1
    fi
    
    INPUT_DIR="$2"
    OUTPUT_DIR="${3:-$INPUT_DIR}"
    
    if [ ! -d "$INPUT_DIR" ]; then
        echo "Error: Input directory '$INPUT_DIR' not found"
        exit 1
    fi
    
    echo "Converting audio files from $INPUT_DIR to $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
    
    converted_count=0
    for audio_file in "$INPUT_DIR"/*.{wav,mp3,ogg,flac,m4a,aac} 2>/dev/null; do
        if [ -f "$audio_file" ]; then
            base_name=$(basename "$audio_file")
            name_without_ext="${base_name%.*}"
            output_file="$OUTPUT_DIR/${name_without_ext}.pcm"
            
            echo "Converting: $base_name -> ${name_without_ext}.pcm"
            
            if ffmpeg -i "$audio_file" -ar 44100 -ac 1 -f s16le -y "$output_file" 2>/dev/null; then
                # Get duration for verification
                duration=$(ffmpeg -i "$audio_file" 2>&1 | grep "Duration" | cut -d ' ' -f 4 | sed s/,//)
                echo "  Duration: $duration"
                ((converted_count++))
            else
                echo "  Failed to convert $base_name"
            fi
        fi
    done
    
    if [ $converted_count -eq 0 ]; then
        echo "No audio files found in $INPUT_DIR"
        echo "Supported formats: wav, mp3, ogg, flac, m4a, aac"
    else
        echo "Successfully converted $converted_count audio file(s)"
    fi
    
    exit 0
fi

# Single file conversion mode
INPUT="$1"
OUTPUT="${2:-${INPUT%.*}.pcm}"

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found"
    exit 1
fi

echo "Converting: $(basename "$INPUT") -> $(basename "$OUTPUT")"

# Create output directory if needed
mkdir -p "$(dirname "$OUTPUT")"

# Convert audio
if ffmpeg -i "$INPUT" -ar 44100 -ac 1 -f s16le -y "$OUTPUT" 2>/dev/null; then
    # Get file info for verification
    duration=$(ffmpeg -i "$INPUT" 2>&1 | grep "Duration" | cut -d ' ' -f 4 | sed s/,//)
    file_size=$(stat -f%z "$OUTPUT" 2>/dev/null || stat -c%s "$OUTPUT" 2>/dev/null || echo "unknown")
    echo "  Duration: $duration"
    echo "  Output size: $file_size bytes"
    echo "Done!"
else
    echo "Error: Failed to convert audio file"
    exit 1
fi