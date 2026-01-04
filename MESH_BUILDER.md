# Enhanced Mesh Builder for Brush-Based Geometry

This implementation provides a comprehensive mesh builder that can generate triangle meshes from convex brush definitions. It works with any brush defined by a set of planes, making it suitable for CSG (Constructive Solid Geometry) operations and complex level geometry.

## Features

- **Universal Brush Support**: Works with any convex brush defined by planes
- **Convex Hull Generation**: Automatically generates proper triangle meshes from brush definitions
- **Multiple Color Modes**: Support for single-color brushes and per-face coloring
- **Backward Compatibility**: Still supports simple box generation
- **Performance Optimized**: Uses efficient algorithms for vertex generation and face detection
- **Memory Efficient**: Uses unmanaged arrays for optimal memory usage

## Core Components

### MeshBuilder

The main class that handles mesh generation:

```zig
pub const MeshBuilder = struct {
    vertices: std.ArrayListUnmanaged(Vertex) = .{},
    indices: std.ArrayListUnmanaged(u16) = .{},
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) MeshBuilder
    pub fn deinit(self: *MeshBuilder) void
    pub fn clear(self: *MeshBuilder) void
}
```

### Key Methods

#### `addBrush(brush: Brush, color: [4]f32)`
Generates a triangle mesh from any convex brush definition. The algorithm:
1. Finds all vertices by intersecting plane triplets
2. Validates vertices are inside the brush
3. Generates convex hull faces
4. Applies proper winding order for correct normals

#### `addBrushes(brushes: []const Brush, color: [4]f32)`
Convenience method to add multiple brushes with the same color.

#### `addBrushWithFaceColors(brush: Brush, colors: []const [4]f32)`
Generates mesh with different colors per face, useful for debugging or visual variety.

#### `addBox(center: Vec3, size: Vec3, color: [4]f32)`
Backward-compatible method for simple box generation.

#### `getStats()`
Returns mesh statistics (vertex count, triangle count, etc.).

## Algorithm Details

### Vertex Generation
The mesh builder finds vertices by:
1. Intersecting every combination of three planes
2. Checking if the intersection point satisfies all brush plane constraints
3. Removing duplicate vertices within epsilon tolerance

### Face Generation
Uses a convex hull approach:
1. For each triplet of vertices, calculates face normal
2. Checks if all other vertices are on one side of the face
3. Applies correct winding order based on normal direction
4. Filters out degenerate triangles

### Plane Intersection Mathematics
Three planes intersect at a point using Cramer's rule:
```
Point = (n1×n2×d3 + n2×n3×d1 + n3×n1×d2) / det(n1, n2, n3)
```
Where n1, n2, n3 are plane normals and d1, d2, d3 are plane distances.

## Usage Examples

### Basic Brush Mesh
```zig
var mesh_builder = MeshBuilder.init(allocator);
defer mesh_builder.deinit();

const planes = [_]Plane{
    .{ .normal = Vec3.new(1, 0, 0), .distance = -1 },
    .{ .normal = Vec3.new(-1, 0, 0), .distance = -1 },
    // ... more planes
};

const brush = Brush{
    .planes = &planes,
    .bounds = AABB.new(Vec3.new(-1, -1, -1), Vec3.new(1, 1, 1))
};

try mesh_builder.addBrush(brush, .{ 1.0, 0.0, 0.0, 1.0 });
```

### Multiple Brushes
```zig
const brushes = [_]Brush{ brush1, brush2, brush3 };
try mesh_builder.addBrushes(&brushes, .{ 0.5, 0.5, 0.5, 1.0 });
```

### Per-Face Colors
```zig
const colors = [_][4]f32{
    .{ 1.0, 0.0, 0.0, 1.0 }, // Red
    .{ 0.0, 1.0, 0.0, 1.0 }, // Green
    .{ 0.0, 0.0, 1.0, 1.0 }, // Blue
};
try mesh_builder.addBrushWithFaceColors(brush, &colors);
```

## Integration

The mesh builder integrates seamlessly with the existing rendering pipeline:

1. Generate meshes from brushes during initialization
2. Upload vertex/index data to GPU buffers
3. Render using standard triangle rendering

## Performance Considerations

- **Complexity**: O(n³) for n planes per brush (due to triplet intersection)
- **Memory**: Efficient unmanaged arrays minimize allocations
- **Optimization**: Duplicate vertex removal and degenerate triangle filtering
- **Scalability**: Suitable for hundreds of brushes in real-time applications

## Limitations

- Only works with convex brushes (concave shapes need decomposition)
- Vertex precision limited by floating-point epsilon
- Face generation may create extra triangles for complex geometry
- No automatic UV coordinate generation (vertices only have position and color)

## Future Enhancements

- UV coordinate generation for texture mapping
- Normal vector calculation for lighting
- LOD (Level of Detail) generation
- Mesh optimization (vertex welding, triangle reduction)
- Support for non-convex brushes via decomposition