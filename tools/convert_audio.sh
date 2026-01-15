#!/bin/bash

# Audio conversion script for FPS game
# Converts any audio format to mono 44.1kHz 16-bit WAV with high quality

set -e

show_usage() {
    echo "Usage: $0 <input_file> [output_file]"
    echo ""
    echo "Converts audio to mono 44.1kHz 16-bit WAV format for the game engine."
    echo ""
    echo "Arguments:"
    echo "  input_file   Input audio file (any format supported by ffmpeg)"
    echo "  output_file  Output WAV file (optional, defaults to input with .wav extension)"
    echo ""
    echo "Examples:"
    echo "  $0 music.mp3"
    echo "  $0 sound.flac assets/game/music.wav"
    echo "  $0 assets/game/music.wav  # Convert in-place"
}

if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_usage
    exit 0
fi

INPUT="$1"
OUTPUT="${2:-${INPUT%.*}.wav}"

# Check if input file exists
if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found"
    exit 1
fi

# Check if ffmpeg is available
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg is not installed or not in PATH"
    echo "Install with: brew install ffmpeg (macOS) or apt install ffmpeg (Ubuntu)"
    exit 1
fi

echo "Converting: $INPUT -> $OUTPUT"
echo "Target format: Mono 44.1kHz 16-bit WAV"

# Create output directory if it doesn't exist
mkdir -p "$(dirname "$OUTPUT")"

# Convert with high quality settings
ffmpeg -i "$INPUT" \
    -ar 44100 \
    -ac 1 \
    -sample_fmt s16 \
    -acodec pcm_s16le \
    -af "highpass=f=20,lowpass=f=20000" \
    -y \
    "$OUTPUT"

echo "Conversion complete!"

# Show file info
if command -v file &> /dev/null; then
    echo "Output file info:"
    file "$OUTPUT"
fi