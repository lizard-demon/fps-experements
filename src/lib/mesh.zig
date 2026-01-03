const std = @import("std");
const math = @import("math.zig");

const Vec3 = math.Vec3;

pub const Vertex = extern struct { pos: [3]f32, col: [4]f32 };

pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
};

pub const Brush = struct {
    planes: []const Plane,
    
    pub fn new(planes: []const Plane) Brush {
        return .{ .planes = planes };
    }
};

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
    
    pub fn addBox(self: *MeshBuilder, center: Vec3, size: Vec3, color: [4]f32) !void {
        const half = Vec3.scale(size, 0.5);
        const min = Vec3.sub(center, half);
        const max = Vec3.add(center, half);
        
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        // 8 box vertices
        const verts = [_]Vec3{
            Vec3.new(min.data[0], min.data[1], min.data[2]), // 0
            Vec3.new(max.data[0], min.data[1], min.data[2]), // 1
            Vec3.new(max.data[0], max.data[1], min.data[2]), // 2
            Vec3.new(min.data[0], max.data[1], min.data[2]), // 3
            Vec3.new(min.data[0], min.data[1], max.data[2]), // 4
            Vec3.new(max.data[0], min.data[1], max.data[2]), // 5
            Vec3.new(max.data[0], max.data[1], max.data[2]), // 6
            Vec3.new(min.data[0], max.data[1], max.data[2]), // 7
        };
        
        for (verts) |v| {
            try self.vertices.append(self.allocator, .{ 
                .pos = .{ v.data[0], v.data[1], v.data[2] }, 
                .col = color 
            });
        }
        
        // 12 triangles (6 faces * 2 triangles each)
        const faces = [_][3]u16{
            .{ 0, 1, 2 }, .{ 0, 2, 3 }, // Front
            .{ 5, 4, 7 }, .{ 5, 7, 6 }, // Back
            .{ 4, 0, 3 }, .{ 4, 3, 7 }, // Left
            .{ 1, 5, 6 }, .{ 1, 6, 2 }, // Right
            .{ 4, 5, 1 }, .{ 4, 1, 0 }, // Bottom
            .{ 3, 2, 6 }, .{ 3, 6, 7 }, // Top
        };
        
        for (faces) |face| {
            try self.indices.append(self.allocator, base + face[0]);
            try self.indices.append(self.allocator, base + face[1]);
            try self.indices.append(self.allocator, base + face[2]);
        }
    }
    
    pub fn addBrush(self: *MeshBuilder, brush: Brush, color: [4]f32) !void {
        // Generate mesh for each plane face
        for (brush.planes) |plane| {
            try self.addBrushFace(brush, plane, color);
        }
    }
    
    fn addBrushFace(self: *MeshBuilder, brush: Brush, target_plane: Plane, color: [4]f32) !void {
        // Start with a large quad on the plane, then clip it against all other planes
        var face_verts = std.ArrayListUnmanaged(Vec3){};
        defer face_verts.deinit(self.allocator);
        
        // Create initial large quad on the target plane
        const size: f32 = 100; // Large enough for any reasonable brush
        
        // Find two perpendicular vectors to the plane normal
        var u = Vec3.new(1, 0, 0);
        if (@abs(Vec3.dot(target_plane.normal, u)) > 0.9) {
            u = Vec3.new(0, 1, 0);
        }
        u = Vec3.normalize(Vec3.sub(u, Vec3.scale(target_plane.normal, Vec3.dot(u, target_plane.normal))));
        const v = Vec3.cross(target_plane.normal, u);
        
        // Point on the plane
        const plane_point = Vec3.scale(target_plane.normal, -target_plane.distance);
        
        // Create initial quad
        try face_verts.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(u, -size)), Vec3.scale(v, -size)));
        try face_verts.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(u, size)), Vec3.scale(v, -size)));
        try face_verts.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(u, size)), Vec3.scale(v, size)));
        try face_verts.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(u, -size)), Vec3.scale(v, size)));
        
        // Clip against all other planes
        for (brush.planes) |clip_plane| {
            // Skip the target plane itself
            if (Vec3.dot(clip_plane.normal, target_plane.normal) > 0.999 and 
                @abs(clip_plane.distance - target_plane.distance) < 0.001) continue;
            
            try self.clipPolygon(&face_verts, clip_plane);
            if (face_verts.items.len == 0) break;
        }
        
        if (face_verts.items.len < 3) return;
        
        // Add vertices and triangulate
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        for (face_verts.items) |vert| {
            try self.vertices.append(self.allocator, .{
                .pos = .{ vert.data[0], vert.data[1], vert.data[2] },
                .col = color,
            });
        }
        
        // Simple fan triangulation
        for (1..face_verts.items.len - 1) |i| {
            try self.indices.append(self.allocator, base);
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i)));
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i + 1)));
        }
    }
    
    fn clipPolygon(self: *MeshBuilder, polygon: *std.ArrayListUnmanaged(Vec3), plane: Plane) !void {
        if (polygon.items.len == 0) return;
        
        var output = std.ArrayListUnmanaged(Vec3){};
        defer {
            polygon.deinit(self.allocator);
            polygon.* = output;
        }
        
        const epsilon: f32 = 0.001;
        var prev_vert = polygon.items[polygon.items.len - 1];
        var prev_inside = plane.distanceToPoint(prev_vert) <= epsilon;
        
        for (polygon.items) |curr_vert| {
            const curr_inside = plane.distanceToPoint(curr_vert) <= epsilon;
            
            if (curr_inside) {
                if (!prev_inside) {
                    // Entering: add intersection
                    if (self.intersectLine(prev_vert, curr_vert, plane)) |intersection| {
                        try output.append(self.allocator, intersection);
                    }
                }
                try output.append(self.allocator, curr_vert);
            } else if (prev_inside) {
                // Exiting: add intersection
                if (self.intersectLine(prev_vert, curr_vert, plane)) |intersection| {
                    try output.append(self.allocator, intersection);
                }
            }
            
            prev_vert = curr_vert;
            prev_inside = curr_inside;
        }
        
        // Filter out degenerate polygons (less than 3 vertices)
        if (output.items.len < 3) {
            output.clearAndFree(self.allocator);
        }
    }
    
    fn intersectLine(self: *MeshBuilder, start: Vec3, end: Vec3, plane: Plane) ?Vec3 {
        _ = self;
        const dir = Vec3.sub(end, start);
        const denom = Vec3.dot(plane.normal, dir);
        
        const epsilon: f32 = 0.001;
        if (@abs(denom) < epsilon) return null; // Line is parallel to plane
        
        const t = -(plane.distanceToPoint(start)) / denom;
        
        // Ensure intersection is within line segment (with small tolerance)
        if (t < -epsilon or t > 1.0 + epsilon) return null;
        
        return Vec3.add(start, Vec3.scale(dir, t));
    }
};