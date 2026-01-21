# Models Module - Registry System

This directory contains a model registry system that automatically loads all OBJ files and provides mesh data.

## Directory Structure

- `mod.zig` - Model registry module
- `*.obj` - Wavefront OBJ model files

## Quick Start

### Use the Zig module
```zig
const models = @import("models");

// Initialize registry - automatically loads from assets/models/
var registry = try models.Registry.init(allocator);
defer registry.deinit();

// Get mesh data by name
if (registry.get("cube")) |mesh| {
    // Use mesh.vertices and mesh.indices for rendering
    std.log.info("Loaded {} vertices, {} triangles", .{ mesh.vertices.len, mesh.indices.len / 3 });
}
```

## Workflow

1. Place OBJ model files in `assets/models/`
2. Initialize with `models.Registry.init(allocator)` - automatically loads assets/models/
3. Get mesh data by name: `registry.get("model_name")`
4. Use the vertices and indices for rendering

## Module Features

- **Simple API**: Single `Registry` struct with `init()` and `get()`
- **Name-based access**: Get models by filename (without extension)
- **Automatic loading**: Loads entire directories of OBJ files
- **Triangle meshes**: Supports triangulated OBJ files with vertices and faces

## OBJ Format Support

Supports basic Wavefront OBJ format:
- `v x y z` - Vertex positions
- `f v1 v2 v3` - Triangle faces (1-indexed)
- `f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3` - Faces with texture/normal indices (ignores texture/normals)

## Example Usage

```zig
const models = @import("models");

// Create registry - automatically loads from assets/models/
var registry = try models.Registry.init(allocator);
defer registry.deinit();

// Get mesh data
if (registry.get("cube")) |mesh| {
    std.log.info("Model: {s}", .{mesh.name});
    std.log.info("Vertices: {}", .{mesh.vertices.len});
    std.log.info("Triangles: {}", .{mesh.indices.len / 3});
    
    // Use for rendering
    for (mesh.vertices) |vertex| {
        // Process vertex positions
    }
    
    // Process triangles (indices in groups of 3)
    var i: usize = 0;
    while (i < mesh.indices.len) : (i += 3) {
        const v1 = mesh.vertices[mesh.indices[i]];
        const v2 = mesh.vertices[mesh.indices[i + 1]];
        const v3 = mesh.vertices[mesh.indices[i + 2]];
        // Render triangle
    }
}
```

## API Reference

```zig
pub const Mesh = struct {
    name: []const u8,        // Model name (without extension)
    vertices: []Vec3,        // Vertex positions
    indices: []u32,          // Triangle indices (groups of 3)
    allocator: Allocator,    // For cleanup
    
    pub fn deinit() void
};

pub const Registry = struct {
    meshes: StringHashMap(Mesh),    // Name -> mesh mapping
    
    pub fn init(allocator) !Registry     // Loads from assets/models/
    pub fn get(model_name) ?*const Mesh
};
```

## Current Models

- `cube.obj` - Simple cube model