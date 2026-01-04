// Example usage of the enhanced MeshBuilder
const std = @import("std");
const world = @import("src/lib/world.zig");
const math = @import("src/lib/math.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;
const Plane = world.Plane;
const Brush = world.Brush;
const MeshBuilder = world.MeshBuilder;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    
    // Create a mesh builder
    var mesh_builder = MeshBuilder.init(allocator);
    defer mesh_builder.deinit();
    
    // Example 1: Simple box brush
    const box_planes = [_]Plane{
        .{ .normal = Vec3.new( 1,  0,  0), .distance = -1 },  // Right
        .{ .normal = Vec3.new(-1,  0,  0), .distance = -1 },  // Left  
        .{ .normal = Vec3.new( 0,  1,  0), .distance = -1 },  // Top
        .{ .normal = Vec3.new( 0, -1,  0), .distance = -1 },  // Bottom
        .{ .normal = Vec3.new( 0,  0,  1), .distance = -1 },  // Front
        .{ .normal = Vec3.new( 0,  0, -1), .distance = -1 },  // Back
    };
    const box_brush = Brush{
        .planes = &box_planes,
        .bounds = AABB.new(Vec3.new(-1, -1, -1), Vec3.new(1, 1, 1))
    };
    
    // Add the box brush to the mesh
    try mesh_builder.addBrush(box_brush, .{ 1.0, 0.0, 0.0, 1.0 }); // Red color
    
    // Example 2: Triangular prism brush
    const prism_planes = [_]Plane{
        .{ .normal = Vec3.new( 0, -1,  0), .distance = 0 },     // Bottom
        .{ .normal = Vec3.new( 0,  1,  0), .distance = -2 },    // Top
        .{ .normal = Vec3.new( 1,  0,  0), .distance = -1 },    // Right
        .{ .normal = Vec3.new(-0.5, 0, 0.866), .distance = -1 }, // Left angled face
        .{ .normal = Vec3.new(-0.5, 0, -0.866), .distance = -1 }, // Right angled face
    };
    const prism_brush = Brush{
        .planes = &prism_planes,
        .bounds = AABB.new(Vec3.new(-2, 0, -2), Vec3.new(1, 2, 2))
    };
    
    // Add the prism brush to the mesh
    try mesh_builder.addBrush(prism_brush, .{ 0.0, 1.0, 0.0, 1.0 }); // Green color
    
    // Example 3: Using the convenience method for multiple brushes
    const brushes = [_]Brush{ box_brush, prism_brush };
    mesh_builder.clear(); // Clear previous meshes
    try mesh_builder.addBrushes(&brushes, .{ 0.0, 0.0, 1.0, 1.0 }); // Blue color
    
    // Example 4: Using per-face colors
    const face_colors = [_][4]f32{
        .{ 1.0, 0.0, 0.0, 1.0 }, // Red
        .{ 0.0, 1.0, 0.0, 1.0 }, // Green  
        .{ 0.0, 0.0, 1.0, 1.0 }, // Blue
        .{ 1.0, 1.0, 0.0, 1.0 }, // Yellow
        .{ 1.0, 0.0, 1.0, 1.0 }, // Magenta
        .{ 0.0, 1.0, 1.0, 1.0 }, // Cyan
    };
    mesh_builder.clear();
    try mesh_builder.addBrushWithFaceColors(box_brush, &face_colors);
    
    // Get and print statistics
    const stats = mesh_builder.getStats();
    std.log.info("Final mesh statistics:", .{});
    std.log.info("  Vertices: {}", .{stats.vertices});
    std.log.info("  Indices: {}", .{stats.indices});
    std.log.info("  Triangles: {}", .{stats.triangles});
    
    std.log.info("MeshBuilder example completed successfully!", .{});
}