const std = @import("std");
const math = @import("../lib/math.zig");
const collision = @import("collision.zig");
const Vec3 = math.Vec3;
const Brush = collision.Brush;

// Rendering
pub const Vertex = extern struct { pos: [3]f32, col: [4]f32 };

pub const MeshBuilder = struct {
    vertices: std.ArrayListUnmanaged(Vertex) = .{},
    indices: std.ArrayListUnmanaged(u16) = .{},
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) MeshBuilder { 
        return .{ .allocator = allocator }; 
    }
    
    pub fn deinit(self: *MeshBuilder) void { 
        self.vertices.deinit(self.allocator); 
        self.indices.deinit(self.allocator); 
    }
    
    pub fn addBrush(self: *MeshBuilder, brush: Brush, color: [4]f32) !void {
        var hull_vertices = std.ArrayListUnmanaged(Vec3){};
        defer hull_vertices.deinit(self.allocator);
        
        for (0..brush.planes.len) |i| {
            for (i + 1..brush.planes.len) |j| {
                for (j + 1..brush.planes.len) |k| {
                    // intersect three planes
                    const intersection = blk: {
                        const p1 = brush.planes[i];
                        const p2 = brush.planes[j];
                        const p3 = brush.planes[k];
                        const n1 = p1.normal;
                        const n2 = p2.normal;
                        const n3 = p3.normal;
                        
                        const det = Vec3.dot(n1, Vec3.cross(n2, n3));
                        if (@abs(det) < math.EPSILON) break :blk null;
                        
                        const c1 = Vec3.cross(n2, n3);
                        const c2 = Vec3.cross(n3, n1);
                        const c3 = Vec3.cross(n1, n2);
                        
                        break :blk Vec3.scale(Vec3.add(Vec3.add(Vec3.scale(c1, -p1.distance), Vec3.scale(c2, -p2.distance)), Vec3.scale(c3, -p3.distance)), 1.0 / det);
                    };
                    
                    if (intersection) |vertex| {
                        // check if vertex is inside brush
                        const inside_brush = blk: {
                            for (brush.planes) |plane| if (plane.distanceToPoint(vertex) > math.EPSILON) break :blk false;
                            break :blk true;
                        };
                        
                        // check if vertex is duplicate
                        const is_duplicate = blk: {
                            for (hull_vertices.items) |existing| if (Vec3.dist(vertex, existing) < math.EPSILON * 10) break :blk true;
                            break :blk false;
                        };
                        
                        if (inside_brush and !is_duplicate) {
                            try hull_vertices.append(self.allocator, vertex);
                        }
                    }
                }
            }
        }
        
        if (hull_vertices.items.len < 4) return;
        
        // Generate convex hull using gift wrapping (simpler than previous O(n⁴) approach)
        const hull_faces = try self.generateConvexHullFaces(hull_vertices.items);
        defer self.allocator.free(hull_faces);
        
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        for (hull_vertices.items) |v| {
            try self.vertices.append(self.allocator, .{ .pos = .{ v.data[0], v.data[1], v.data[2] }, .col = color });
        }
        
        for (hull_faces) |face| {
            try self.indices.append(self.allocator, base + face[0]);
            try self.indices.append(self.allocator, base + face[1]);
            try self.indices.append(self.allocator, base + face[2]);
        }
    }
    
    fn generateConvexHullFaces(self: *MeshBuilder, vertices: []Vec3) ![][3]u16 {
        if (vertices.len < 4) return &[_][3]u16{};
        
        var faces = std.ArrayListUnmanaged([3]u16){};
        
        // Simple approach: test all triangles, keep those that are hull faces
        for (0..vertices.len) |i| {
            for (i + 1..vertices.len) |j| {
                for (j + 1..vertices.len) |k| {
                    const v0 = vertices[i];
                    const v1 = vertices[j]; 
                    const v2 = vertices[k];
                    
                    const edge1 = Vec3.sub(v1, v0);
                    const edge2 = Vec3.sub(v2, v0);
                    const normal = Vec3.cross(edge1, edge2);
                    
                    if (Vec3.length(normal) < math.EPSILON) continue;
                    
                    // Check if all other vertices are on one side of this triangle
                    var is_hull_face = true;
                    var side_sign: ?f32 = null;
                    
                    for (vertices, 0..) |test_vertex, idx| {
                        if (idx == i or idx == j or idx == k) continue;
                        
                        const dot = Vec3.dot(normal, Vec3.sub(test_vertex, v0));
                        if (@abs(dot) > math.EPSILON) {
                            if (side_sign == null) {
                                side_sign = dot;
                            } else if (side_sign.? * dot < 0) {
                                is_hull_face = false;
                                break;
                            }
                        }
                    }
                    
                    if (is_hull_face) {
                        const face: [3]u16 = if ((side_sign orelse 1) > 0) 
                            .{ @intCast(i), @intCast(k), @intCast(j) }
                        else 
                            .{ @intCast(i), @intCast(j), @intCast(k) };
                        try faces.append(self.allocator, face);
                    }
                }
            }
        }
        
        return try faces.toOwnedSlice(self.allocator);
    }
};