# Audio Module - Mixer-Registry

This directory contains a mixer-registry system that automatically loads all audio files and allows triggering sounds by name.

## Directory Structure

- `mod.zig` - Audio mixer-registry module
- `convert.sh` - Audio conversion script
- `*.pcm` - Raw PCM audio files (16-bit mono 44.1kHz)

## Quick Start

### Use the Zig module
```zig
const audio = @import("audio");

// Initialize registry - automatically loads from assets/audio/
var registry = try audio.Registry.init(allocator);
defer registry.deinit();

// Play sounds with options
registry.play("jump", .{});                    // Default: no loop, full volume
registry.play("music", .{ .loop = true });     // Loop with full volume
registry.play("explosion", .{ .volume = 0.5 }); // Half volume, no loop
registry.play("ambient", .{ .loop = true, .volume = 0.3 }); // Background audio
```

### Convert audio with shell script
1. **Place audio files** in any format (wav, mp3, ogg, etc.)
2. **Convert single file**: `./assets/audio/convert.sh input.wav output.pcm`
3. **Convert directory**: `./assets/audio/convert.sh --batch input_dir assets/audio`

## Workflow

1. Place audio files in any supported format
2. Convert using `./assets/audio/convert.sh --batch input_dir assets/audio`
3. Initialize with `audio.Registry.init(allocator)` - automatically loads assets/audio/
4. Play sounds by name: `registry.play("sound_name", .{})`

## Module Features

- **Powerful options**: Single `play()` function with flexible options struct
- **Name-based access**: Play sounds by filename (without extension)
- **Automatic loading**: Loads entire directories of PCM files
- **Simple defaults**: `play("sound", .{})` for basic usage
- **Multi-voice mixing**: Up to 16 simultaneous voices
- **Full control**: Loop, volume, and more options per sound

## Example Usage

```zig
const audio = @import("audio");

// Create registry - automatically loads from assets/audio/
var registry = try audio.Registry.init(allocator);
defer registry.deinit();

// Direct access to data
std.log.info("Loaded {} clips", .{registry.clips.items.len});
std.log.info("Active voices: {}", .{registry.voices.len});

// Play sounds with different options
registry.play("music", .{ .loop = true, .volume = 0.3 });  // Background music
registry.play("jump", .{});                                // Sound effect (defaults)
registry.play("explosion", .{ .volume = 0.8 });           // Loud explosion
registry.play("ambient", .{ .loop = true, .volume = 0.1 }); // Quiet ambient loop

// Stop specific sounds
registry.stop("music");          // Stop background music
registry.stopAll();              // Stop everything

// In your audio callback
const sample = registry.sample(44100.0);
```

## API Reference

```zig
pub const Options = struct {
    loop: bool = false,    // Whether to loop the sound
    volume: f32 = 1.0,     // Volume (0.0 to 1.0+)
};

pub const Clip = struct {
    name: []const u8,      // Clip name (without extension)
    samples: []f32,        // Audio sample data
    sample_rate: u32,      // Sample rate in Hz
};

pub const Voice = struct {
    clip: ?*const Clip,    // Currently playing clip (null if free)
    position: f32,         // Playback position in samples
    looping: bool,         // Whether this voice loops
    volume: f32,           // Voice volume multiplier
};

pub const Registry = struct {
    clips: ArrayListUnmanaged(Clip),     // All loaded clips
    voices: [16]Voice,                   // Voice slots for mixing
    // ... other fields
    
    pub fn init(allocator) !Registry     // Loads from assets/audio/
    pub fn play(sound_name, options) void
    pub fn stop(sound_name) void
    pub fn stopAll() void
    pub fn sample(target_sample_rate) f32
};
```

## Supported Audio Formats

Input formats (via ffmpeg):
- WAV, MP3, OGG, FLAC, M4A, AAC, and more

Output format:
- Raw PCM: 16-bit signed, mono, 44.1kHz

## Current Audio Files

Check the assets directory for existing PCM files:
- Game audio in `assets/game/`
- Player audio in `assets/player/`