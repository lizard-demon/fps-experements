const std = @import("std");
const math = @import("math.zig");
const collision = @import("collision.zig");

const Vec3 = math.Vec3;
const AABB = collision.AABB;
const Brush = collision.Brush;

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
    
    pub fn addBox(self: *MeshBuilder, center: Vec3, size: Vec3, color: [4]f32) !void {
        const half = Vec3.scale(size, 0.5);
        const min = Vec3.sub(center, half);
        const max = Vec3.add(center, half);
        
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        // Box vertices
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
        
        // Box faces (12 triangles)
        const faces = [_][3]u16{
            // Front
            .{ 0, 1, 2 }, .{ 0, 2, 3 },
            // Back
            .{ 5, 4, 7 }, .{ 5, 7, 6 },
            // Left
            .{ 4, 0, 3 }, .{ 4, 3, 7 },
            // Right
            .{ 1, 5, 6 }, .{ 1, 6, 2 },
            // Bottom
            .{ 4, 5, 1 }, .{ 4, 1, 0 },
            // Top
            .{ 3, 2, 6 }, .{ 3, 6, 7 },
        };
        
        for (faces) |face| {
            try self.indices.append(self.allocator, base + face[0]);
            try self.indices.append(self.allocator, base + face[1]);
            try self.indices.append(self.allocator, base + face[2]);
        }
    }
    
    pub fn addBrush(self: *MeshBuilder, brush: Brush, color: [4]f32) !void {
        // For each plane, generate a face
        for (brush.planes) |plane| {
            try self.addBrushPlaneFace(brush, plane, color);
        }
    }
    
    fn addBrushPlaneFace(self: *MeshBuilder, brush: Brush, plane: collision.Plane, color: [4]f32) !void {
        var face_vertices = std.ArrayListUnmanaged(Vec3){};
        defer face_vertices.deinit(self.allocator);
        
        try self.generateClippedFace(brush, plane, &face_vertices);
        
        if (face_vertices.items.len < 3) return;
        
        self.ensureCorrectWinding(face_vertices.items, plane.normal);
        
        const base = @as(u16, @intCast(self.vertices.items.len));
        const center = self.calculateCenter(face_vertices.items);
        
        for (face_vertices.items) |vertex| {
            var c = color;
            if (vertex.data[1] <= center.data[1]) { 
                c[0] *= 0.7; c[1] *= 0.7; c[2] *= 0.7; 
            } else { 
                c[0] *= 1.1; c[1] *= 1.1; c[2] *= 1.1; 
            }
            try self.vertices.append(self.allocator, .{ 
                .pos = .{ vertex.data[0], vertex.data[1], vertex.data[2] }, 
                .col = c 
            });
        }
        
        // Triangulate
        for (1..face_vertices.items.len - 1) |i| {
            try self.indices.append(self.allocator, base);
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i)));
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i + 1)));
        }
    }
    
    fn generateClippedFace(self: *MeshBuilder, brush: Brush, target_plane: collision.Plane, face_vertices: *std.ArrayListUnmanaged(Vec3)) !void {
        var polygon = std.ArrayListUnmanaged(Vec3){};
        defer polygon.deinit(self.allocator);
        
        const bounds_size = Vec3.sub(brush.bounds.max, brush.bounds.min);
        const max_size = @max(@max(bounds_size.data[0], bounds_size.data[1]), bounds_size.data[2]) * 2;
        
        var tangent = Vec3.new(1, 0, 0);
        if (@abs(Vec3.dot(target_plane.normal, tangent)) > 0.9) {
            tangent = Vec3.new(0, 1, 0);
        }
        tangent = Vec3.normalize(Vec3.sub(tangent, Vec3.scale(target_plane.normal, Vec3.dot(tangent, target_plane.normal))));
        const bitangent = Vec3.cross(target_plane.normal, tangent);
        
        const brush_center = Vec3.scale(Vec3.add(brush.bounds.min, brush.bounds.max), 0.5);
        const plane_point = Vec3.sub(brush_center, Vec3.scale(target_plane.normal, target_plane.distanceToPoint(brush_center)));
        
        const half_size = max_size * 0.5;
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, -half_size)), Vec3.scale(bitangent, -half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, half_size)), Vec3.scale(bitangent, -half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, half_size)), Vec3.scale(bitangent, half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, -half_size)), Vec3.scale(bitangent, half_size)));
        
        for (brush.planes) |clip_plane| {
            if (Vec3.dot(clip_plane.normal, target_plane.normal) > 0.999 and 
                @abs(clip_plane.distance - target_plane.distance) < 0.001) continue;
            
            try self.clipPolygonByPlane(&polygon, clip_plane);
            if (polygon.items.len == 0) break;
        }
        
        for (polygon.items) |vertex| {
            try face_vertices.append(self.allocator, vertex);
        }
    }
    
    fn clipPolygonByPlane(self: *MeshBuilder, polygon: *std.ArrayListUnmanaged(Vec3), plane: collision.Plane) !void {
        if (polygon.items.len == 0) return;
        
        var output = std.ArrayListUnmanaged(Vec3){};
        defer {
            polygon.deinit(self.allocator);
            polygon.* = output;
        }
        
        var prev_vertex = polygon.items[polygon.items.len - 1];
        var prev_inside = plane.distanceToPoint(prev_vertex) <= collision.COLLISION_EPSILON;
        
        for (polygon.items) |curr_vertex| {
            const curr_inside = plane.distanceToPoint(curr_vertex) <= collision.COLLISION_EPSILON;
            
            if (curr_inside) {
                if (!prev_inside) {
                    if (self.intersectLineWithPlane(prev_vertex, curr_vertex, plane)) |intersection| {
                        try output.append(self.allocator, intersection);
                    }
                }
                try output.append(self.allocator, curr_vertex);
            } else if (prev_inside) {
                if (self.intersectLineWithPlane(prev_vertex, curr_vertex, plane)) |intersection| {
                    try output.append(self.allocator, intersection);
                }
            }
            
            prev_vertex = curr_vertex;
            prev_inside = curr_inside;
        }
    }
    
    fn intersectLineWithPlane(self: *MeshBuilder, start: Vec3, end: Vec3, plane: collision.Plane) ?Vec3 {
        _ = self;
        const dir = Vec3.sub(end, start);
        const denom = Vec3.dot(plane.normal, dir);
        
        if (@abs(denom) < collision.COLLISION_EPSILON) return null;
        
        const t = -(plane.distanceToPoint(start)) / denom;
        return Vec3.add(start, Vec3.scale(dir, t));
    }
    
    fn ensureCorrectWinding(self: *MeshBuilder, vertices: []Vec3, normal: Vec3) void {
        if (vertices.len < 3) return;
        
        const center = self.calculateCenter(vertices);
        self.sortVerticesCounterClockwise(vertices, normal, center);
    }
    
    fn calculateCenter(self: *MeshBuilder, vertices: []Vec3) Vec3 {
        _ = self;
        var center = Vec3.zero();
        for (vertices) |vertex| {
            center = Vec3.add(center, vertex);
        }
        return Vec3.scale(center, 1.0 / @as(f32, @floatFromInt(vertices.len)));
    }
    
    fn sortVerticesCounterClockwise(self: *MeshBuilder, vertices: []Vec3, normal: Vec3, center: Vec3) void {
        _ = self;
        var tangent = Vec3.new(1, 0, 0);
        if (@abs(Vec3.dot(normal, tangent)) > 0.9) {
            tangent = Vec3.new(0, 1, 0);
        }
        tangent = Vec3.normalize(Vec3.sub(tangent, Vec3.scale(normal, Vec3.dot(tangent, normal))));
        const bitangent = Vec3.cross(normal, tangent);
        
        const Context = struct {
            center: Vec3,
            tangent: Vec3,
            bitangent: Vec3,
            
            fn lessThan(ctx: @This(), a: Vec3, b: Vec3) bool {
                const va = Vec3.sub(a, ctx.center);
                const vb = Vec3.sub(b, ctx.center);
                
                const angle_a = std.math.atan2(Vec3.dot(va, ctx.bitangent), Vec3.dot(va, ctx.tangent));
                const angle_b = std.math.atan2(Vec3.dot(vb, ctx.bitangent), Vec3.dot(vb, ctx.tangent));
                
                return angle_a < angle_b;
            }
        };
        
        const context = Context{ .center = center, .tangent = tangent, .bitangent = bitangent };
        std.mem.sort(Vec3, vertices, context, Context.lessThan);
    }
};