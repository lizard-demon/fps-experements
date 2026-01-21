# Maps Module - Registry System

This directory contains a map registry system that automatically loads all map files and provides map data.

## Directory Structure

- `mod.zig` - Map registry module
- `*.json` - Map definition files

## Quick Start

### Use the Zig module
```zig
const maps = @import("maps");

// Initialize registry - automatically loads from assets/maps/
var registry = try maps.Registry.init(allocator);
defer registry.deinit();

// Get map data by name
if (registry.get("test")) |map_data| {
    const spawn_pos = map_data.getSpawnPosition();
    // Use map_data.brushes for world creation
}
```

## Workflow

1. Create JSON map files in `assets/maps/`
2. Initialize with `maps.Registry.init(allocator)` - automatically loads assets/maps/
3. Get map data by name: `registry.get("map_name")`
4. Use the data to create your world/physics representation

## Module Features

- **Simple API**: Single `Registry` struct with `init()` and `get()`
- **Name-based access**: Get maps by filename (without extension)
- **Automatic loading**: Loads entire directories of JSON files
- **Data-only**: Provides parsed map data, not full world objects

## Map Format

```json
{
  "name": "Test Map",
  "spawn_position": [0.0, 3.0, -20.0],
  "brushes": [
    {
      "type": "box",
      "position": [0.0, -2.25, 0.0],
      "size": [100.0, 4.5, 100.0]
    },
    {
      "type": "slope",
      "position": [0.0, 0.0, 20.0],
      "width": 30.0,
      "height": 20.0,
      "angle": 46.0
    }
  ]
}
```

## Example Usage

```zig
const maps = @import("maps");

// Create registry - automatically loads from assets/maps/
var registry = try maps.Registry.init(allocator);
defer registry.deinit();

// Get map data
if (registry.get("test")) |map_data| {
    std.log.info("Map: {s}", .{map_data.name});
    std.log.info("Spawn: {any}", .{map_data.spawn_position});
    std.log.info("Brushes: {}", .{map_data.brushes.len});
    
    // Use the data to create your world
    for (map_data.brushes) |brush_data| {
        switch (brush_data) {
            .box => |box| {
                // Create box brush from box.position and box.size
            },
            .slope => |slope| {
                // Create slope brush from slope data
            },
        }
    }
}
```

## API Reference

```zig
pub const BrushData = union(enum) {
    box: struct {
        position: [3]f32,
        size: [3]f32,
    },
    slope: struct {
        position: [3]f32,
        width: f32,
        height: f32,
        angle: f32,
    },
};

pub const Data = struct {
    name: []const u8,
    spawn_position: [3]f32,
    brushes: []BrushData,
    
    pub fn getSpawnPosition() Vec3
};

pub const Registry = struct {
    maps: StringHashMap(Data),    // Name -> map data mapping
    
    pub fn init(allocator) !Registry     // Loads from assets/maps/
    pub fn get(map_name) ?*const Data
};
```

## Current Maps

- `test.json` - Test map with various brush types