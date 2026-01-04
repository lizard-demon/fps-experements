// Minimal world system - just BVH traversal and brush collision
const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;
const EPSILON = math.EPSILON;

// BVH node for spatial acceleration
pub const BVHNode = struct {
    bounds: AABB,
    brush_start: u32,
    brush_count: u32,
    left_child: u32,  // 0 = no child (leaf)
    
    fn isLeaf(self: BVHNode) bool {
        return self.left_child == 0;
    }
};

// Core geometric primitive
pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
};

// Convex brush defined by planes
pub const Brush = struct {
    planes: []const Plane,
    bounds: AABB,
    
    pub fn intersects(self: Brush, aabb: AABB) bool {
        if (!self.bounds.intersects(aabb)) return false;
        
        for (self.planes) |plane| {
            const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
            const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            
            if (plane.distanceToPoint(center) > radius + EPSILON) return false;
        }
        return true;
    }
};

// Rendering vertex
pub const Vertex = extern struct { 
    pos: [3]f32, 
    col: [4]f32 
};

// Simple mesh builder for rendering
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
            Vec3.new(min.data[0], min.data[1], min.data[2]), Vec3.new(max.data[0], min.data[1], min.data[2]),
            Vec3.new(max.data[0], max.data[1], min.data[2]), Vec3.new(min.data[0], max.data[1], min.data[2]),
            Vec3.new(min.data[0], min.data[1], max.data[2]), Vec3.new(max.data[0], min.data[1], max.data[2]),
            Vec3.new(max.data[0], max.data[1], max.data[2]), Vec3.new(min.data[0], max.data[1], max.data[2]),
        };
        
        for (verts) |v| {
            try self.vertices.append(self.allocator, .{ 
                .pos = .{ v.data[0], v.data[1], v.data[2] }, .col = color 
            });
        }
        
        // 12 triangles
        const faces = [_][3]u16{
            .{ 0, 1, 2 }, .{ 0, 2, 3 }, .{ 5, 4, 7 }, .{ 5, 7, 6 }, .{ 4, 0, 3 }, .{ 4, 3, 7 },
            .{ 1, 5, 6 }, .{ 1, 6, 2 }, .{ 4, 5, 1 }, .{ 4, 1, 0 }, .{ 3, 2, 6 }, .{ 3, 6, 7 },
        };
        
        for (faces) |face| {
            try self.indices.append(self.allocator, base + face[0]);
            try self.indices.append(self.allocator, base + face[1]);
            try self.indices.append(self.allocator, base + face[2]);
        }
    }
};

// World with BVH for fast collision queries
pub const World = struct {
    brushes: []const Brush,
    bvh_nodes: []BVHNode,
    brush_indices: []u32,
    allocator: std.mem.Allocator,
    
    pub fn init(brushes: []const Brush, allocator: std.mem.Allocator) !World {
        if (brushes.len == 0) {
            return World{ 
                .brushes = brushes, 
                .bvh_nodes = &[_]BVHNode{}, 
                .brush_indices = &[_]u32{}, 
                .allocator = allocator 
            };
        }
        
        const max_nodes = brushes.len * 2;
        const bvh_nodes = try allocator.alloc(BVHNode, max_nodes);
        const brush_indices = try allocator.alloc(u32, brushes.len);
        
        for (brush_indices, 0..) |*idx, i| {
            idx.* = @intCast(i);
        }
        
        var world = World{
            .brushes = brushes,
            .bvh_nodes = bvh_nodes,
            .brush_indices = brush_indices,
            .allocator = allocator,
        };
        
        const node_count = world.buildBVH(0, @intCast(brushes.len), 0);
        world.bvh_nodes = world.bvh_nodes[0..node_count];
        
        return world;
    }
    
    pub fn deinit(self: *World) void {
        self.allocator.free(self.bvh_nodes);
        self.allocator.free(self.brush_indices);
    }
    
    // SAH-based BVH construction
    fn buildBVH(self: *World, start: u32, count: u32, node_idx: u32) u32 {
        if (node_idx >= self.bvh_nodes.len) return node_idx;
        
        var bounds = self.brushes[self.brush_indices[start]].bounds;
        for (start + 1..start + count) |i| {
            bounds = bounds.union_with(self.brushes[self.brush_indices[i]].bounds);
        }
        
        self.bvh_nodes[node_idx] = BVHNode{
            .bounds = bounds,
            .brush_start = start,
            .brush_count = count,
            .left_child = 0,
        };
        
        if (count <= 4) return node_idx + 1;
        
        var best_cost = std.math.floatMax(f32);
        var best_axis: u32 = 0;
        var best_split: u32 = start + count / 2;
        
        const parent_area = bounds.surface_area();
        
        for (0..3) |axis| {
            self.sortBrushesByAxis(start, count, axis);
            
            var split = start + 1;
            while (split < start + count) : (split += 1) {
                var left_bounds = self.brushes[self.brush_indices[start]].bounds;
                for (start + 1..split) |i| {
                    left_bounds = left_bounds.union_with(self.brushes[self.brush_indices[i]].bounds);
                }
                
                var right_bounds = self.brushes[self.brush_indices[split]].bounds;
                for (split + 1..start + count) |i| {
                    right_bounds = right_bounds.union_with(self.brushes[self.brush_indices[i]].bounds);
                }
                
                const left_area = left_bounds.surface_area();
                const right_area = right_bounds.surface_area();
                const left_count = split - start;
                const right_count = (start + count) - split;
                
                const cost = 1.0 + 
                    (left_area / parent_area) * @as(f32, @floatFromInt(left_count)) +
                    (right_area / parent_area) * @as(f32, @floatFromInt(right_count));
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = @intCast(axis);
                    best_split = split;
                }
            }
        }
        
        self.sortBrushesByAxis(start, count, best_axis);
        
        var split_pos = start + count / 2;
        for (start..start + count) |i| {
            if (i >= best_split) {
                split_pos = @intCast(i);
                break;
            }
        }
        
        const left_child_idx = node_idx + 1;
        self.bvh_nodes[node_idx].left_child = left_child_idx;
        
        const left_count = split_pos - start;
        const right_count = (start + count) - split_pos;
        
        var next_idx = self.buildBVH(start, left_count, left_child_idx);
        next_idx = self.buildBVH(split_pos, right_count, next_idx);
        
        return next_idx;
    }
    
    fn sortBrushesByAxis(self: *World, start: u32, count: u32, axis: usize) void {
        for (start + 1..start + count) |i| {
            const key = self.brush_indices[i];
            const key_bounds = self.brushes[key].bounds;
            const key_center = (key_bounds.min.data[axis] + key_bounds.max.data[axis]) * 0.5;
            
            var j = i;
            while (j > start) {
                const prev_bounds = self.brushes[self.brush_indices[j - 1]].bounds;
                const prev_center = (prev_bounds.min.data[axis] + prev_bounds.max.data[axis]) * 0.5;
                if (prev_center <= key_center) break;
                
                self.brush_indices[j] = self.brush_indices[j - 1];
                j -= 1;
            }
            self.brush_indices[j] = key;
        }
    }
    
    // Core collision query - just returns true/false
    pub fn testCollision(self: *const World, aabb: AABB) bool {
        if (self.bvh_nodes.len == 0) {
            for (self.brushes) |brush| {
                if (brush.intersects(aabb)) return true;
            }
            return false;
        }
        
        return self.testCollisionBVH(aabb, 0);
    }
    
    fn testCollisionBVH(self: *const World, aabb: AABB, node_idx: u32) bool {
        if (node_idx >= self.bvh_nodes.len) return false;
        
        const node = self.bvh_nodes[node_idx];
        if (!node.bounds.intersects(aabb)) return false;
        
        if (node.isLeaf()) {
            for (node.brush_start..node.brush_start + node.brush_count) |i| {
                const brush_idx = self.brush_indices[i];
                if (self.brushes[brush_idx].intersects(aabb)) return true;
            }
            return false;
        }
        
        return self.testCollisionBVH(aabb, node.left_child) or
               self.testCollisionBVH(aabb, node.left_child + 1);
    }
};