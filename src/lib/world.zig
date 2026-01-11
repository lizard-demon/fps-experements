const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const HullType = enum { point, standing, crouching };
pub const Collision = struct { normal: Vec3 };

const hull_radii = [_]f32{ 0.0, 0.49, 0.49 };

pub const Plane = struct {
    normal: Vec3, distance: f32,
    fn distanceToPoint(self: Plane, point: Vec3) f32 { return Vec3.dot(self.normal, point) + self.distance; }
};

pub const Brush = struct {
    planes: []const Plane, bounds: AABB,
    
    fn expand(self: Brush, radius: f32, allocator: std.mem.Allocator) !Brush {
        if (radius <= 0.0) return .{ .planes = self.planes, .bounds = self.bounds };
        
        const expanded_planes = try allocator.alloc(Plane, self.planes.len);
        for (self.planes, expanded_planes) |p, *ep| ep.* = .{ .normal = p.normal, .distance = p.distance - radius };
        
        const r = Vec3.new(radius, radius, radius);
        return .{ .planes = expanded_planes, .bounds = .{ .min = Vec3.sub(self.bounds.min, r), .max = Vec3.add(self.bounds.max, r) } };
    }
    
    fn check(self: Brush, point: Vec3) ?Collision {
        var closest_normal: ?Vec3 = null;
        var closest_distance: f32 = std.math.floatMax(f32);
        
        for (self.planes) |plane| {
            const distance = plane.distanceToPoint(point);
            if (distance > math.EPSILON) return null;
            if (distance > -closest_distance) {
                closest_distance = -distance;
                closest_normal = plane.normal;
            }
        }
        
        return if (closest_normal) |normal| Collision{ .normal = normal } else null;
    }
};

const Node = packed struct {
    min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32,
    first: u32, count: u30, axis: u2,
    
    fn bounds(self: Node) AABB { return AABB{ .min = Vec3.new(self.min_x, self.min_y, self.min_z), .max = Vec3.new(self.max_x, self.max_y, self.max_z) }; }
    fn isLeaf(self: Node) bool { return self.axis == 3; }
    fn left(self: Node) u32 { return self.first; }
};

const CollisionWorld = struct {
    brushes: []const Brush, nodes: []Node, indices: []u32, allocator: std.mem.Allocator,
    
    fn init(brushes: []const Brush, allocator: std.mem.Allocator) !CollisionWorld {
        if (brushes.len == 0) return .{ .brushes = brushes, .nodes = &[_]Node{}, .indices = &[_]u32{}, .allocator = allocator };
        
        const indices = try allocator.alloc(u32, brushes.len);
        for (indices, 0..) |*idx, i| idx.* = @intCast(i);
        
        var world = CollisionWorld{ .brushes = brushes, .nodes = undefined, .indices = indices, .allocator = allocator };
        world.nodes = try world.buildBVH();
        return world;
    }
    
    fn deinit(self: *CollisionWorld) void {
        self.allocator.free(self.nodes);
        self.allocator.free(self.indices);
    }
    
    fn check(self: *const CollisionWorld, point: Vec3) ?Collision {
        if (self.nodes.len == 0) {
            for (self.brushes) |brush| if (brush.check(point)) |collision| return collision;
            return null;
        }
        
        var node_idx: u32 = 0;
        while (node_idx < self.nodes.len) {
            const node = self.nodes[node_idx];
            if (!node.bounds().containsPoint(point)) { node_idx += 1; continue; }
            
            if (node.isLeaf()) {
                for (node.first..node.first + node.count) |i| 
                    if (self.brushes[self.indices[i]].check(point)) |collision| return collision;
                node_idx += 1;
            } else {
                node_idx = node.left();
            }
        }
        return null;
    }
    
    fn buildBVH(self: *CollisionWorld) ![]Node {
        var nodes = std.ArrayListUnmanaged(Node){};
        defer nodes.deinit(self.allocator);
        _ = try self.buildRecursive(0, @intCast(self.brushes.len), &nodes);
        return try self.layoutBreadthFirst(nodes.items);
    }
    
    fn buildRecursive(self: *CollisionWorld, start: u32, count: u32, nodes: *std.ArrayListUnmanaged(Node)) !u32 {
        const node_idx = @as(u32, @intCast(nodes.items.len));
        var bounds = self.brushes[self.indices[start]].bounds;
        for (start + 1..start + count) |i| bounds = bounds.union_with(self.brushes[self.indices[i]].bounds);
        
        try nodes.append(self.allocator, Node{
            .min_x = bounds.min.data[0], .min_y = bounds.min.data[1], .min_z = bounds.min.data[2],
            .max_x = bounds.max.data[0], .max_y = bounds.max.data[1], .max_z = bounds.max.data[2],
            .first = start, .count = @intCast(count), .axis = 3,
        });
        
        if (count <= 4) return node_idx;
        
        const split = self.findBestSplit(start, count, bounds);
        if (split.cost >= @as(f32, @floatFromInt(count))) return node_idx;
        
        const split_idx = self.partition(start, count, split.axis, split.pos);
        const left_count = split_idx - start;
        const right_count = (start + count) - split_idx;
        if (left_count == 0 or right_count == 0) return node_idx;
        
        const left_child = try self.buildRecursive(start, left_count, nodes);
        _ = try self.buildRecursive(split_idx, right_count, nodes);
        
        nodes.items[node_idx].first = left_child;
        nodes.items[node_idx].count = 0;
        nodes.items[node_idx].axis = @intCast(split.axis);
        return node_idx;
    }
    
    const SplitResult = struct { axis: u32, pos: f32, cost: f32 };
    
    fn findBestSplit(self: *CollisionWorld, start: u32, count: u32, bounds: AABB) SplitResult {
        var best = SplitResult{ .axis = 0, .pos = 0, .cost = std.math.floatMax(f32) };
        const parent_area = bounds.surface_area();
        if (parent_area <= 0) return best;
        
        for (0..3) |axis| {
            for (start..start + count) |i| {
                const brush_bounds = self.brushes[self.indices[i]].bounds;
                const split_pos = (brush_bounds.min.data[axis] + brush_bounds.max.data[axis]) * 0.5;
                
                var left_bounds: ?AABB = null;
                var right_bounds: ?AABB = null;
                var left_count: u32 = 0;
                var right_count: u32 = 0;
                
                for (start..start + count) |j| {
                    const prim_bounds = self.brushes[self.indices[j]].bounds;
                    const centroid = (prim_bounds.min.data[axis] + prim_bounds.max.data[axis]) * 0.5;
                    
                    if (centroid < split_pos) {
                        left_count += 1;
                        left_bounds = if (left_bounds) |lb| lb.union_with(prim_bounds) else prim_bounds;
                    } else {
                        right_count += 1;
                        right_bounds = if (right_bounds) |rb| rb.union_with(prim_bounds) else prim_bounds;
                    }
                }
                
                if (left_count == 0 or right_count == 0) continue;
                
                const left_area = if (left_bounds) |lb| lb.surface_area() else 0;
                const right_area = if (right_bounds) |rb| rb.surface_area() else 0;
                const cost = 0.3 + (left_area / parent_area) * @as(f32, @floatFromInt(left_count)) + (right_area / parent_area) * @as(f32, @floatFromInt(right_count));
                
                if (cost < best.cost) {
                    best = .{ .axis = @intCast(axis), .pos = split_pos, .cost = cost };
                }
            }
        }
        return best;
    }
    
    fn partition(self: *CollisionWorld, start: u32, count: u32, axis: u32, split_pos: f32) u32 {
        var left = start;
        var right = start + count - 1;
        while (left <= right) {
            const centroid = (self.brushes[self.indices[left]].bounds.min.data[axis] + self.brushes[self.indices[left]].bounds.max.data[axis]) * 0.5;
            if (centroid < split_pos) {
                left += 1;
            } else {
                const temp = self.indices[left];
                self.indices[left] = self.indices[right];
                self.indices[right] = temp;
                if (right == 0) break;
                right -= 1;
            }
        }
        return left;
    }
    
    fn layoutBreadthFirst(self: *CollisionWorld, tree_nodes: []Node) ![]Node {
        if (tree_nodes.len == 0) return try self.allocator.alloc(Node, 0);
        const nodes = try self.allocator.alloc(Node, tree_nodes.len);
        var queue: [256]u32 = undefined;
        var queue_head: u32 = 0;
        var queue_tail: u32 = 1;
        queue[0] = 0;
        var write_idx: u32 = 0;
        
        while (queue_head < queue_tail and write_idx < nodes.len) {
            const tree_idx = queue[queue_head];
            queue_head += 1;
            const tree_node = tree_nodes[tree_idx];
            nodes[write_idx] = tree_node;
            
            if (!tree_node.isLeaf() and queue_tail + 1 < queue.len) {
                const left_tree_idx = tree_node.first;
                const right_tree_idx = left_tree_idx + 1;
                const left_bf_idx = write_idx + (queue_tail - queue_head) + 1;
                nodes[write_idx].first = left_bf_idx;
                
                if (left_tree_idx < tree_nodes.len) { queue[queue_tail] = left_tree_idx; queue_tail += 1; }
                if (right_tree_idx < tree_nodes.len) { queue[queue_tail] = right_tree_idx; queue_tail += 1; }
            }
            write_idx += 1;
        }
        return nodes[0..write_idx];
    }
};

pub const World = struct {
    original_brushes: []const Brush,
    hull_worlds: [3]CollisionWorld,
    hull_brushes: [3][]Brush,
    allocator: std.mem.Allocator,
    
    pub fn init(brushes: []const Brush, allocator: std.mem.Allocator) !World {
        var world = World{
            .original_brushes = brushes,
            .hull_worlds = undefined,
            .hull_brushes = undefined,
            .allocator = allocator,
        };
        
        for (hull_radii, 0..) |radius, hull_idx| {
            world.hull_brushes[hull_idx] = try allocator.alloc(Brush, brushes.len);
            for (brushes, world.hull_brushes[hull_idx]) |original_brush, *expanded_brush| {
                expanded_brush.* = try original_brush.expand(radius, allocator);
            }
            world.hull_worlds[hull_idx] = try CollisionWorld.init(world.hull_brushes[hull_idx], allocator);
        }
        
        return world;
    }
    
    pub fn deinit(self: *World) void {
        for (self.hull_brushes, 0..) |hull_brushes, hull_idx| {
            for (hull_brushes, 0..) |brush, brush_idx| {
                if (hull_idx > 0 and brush.planes.ptr != self.original_brushes[brush_idx].planes.ptr) {
                    self.allocator.free(brush.planes);
                }
            }
            self.allocator.free(hull_brushes);
        }
        for (&self.hull_worlds) |*hull_world| hull_world.deinit();
    }
    
    pub fn check(self: *const World, point: Vec3, hull_type: HullType) ?Collision {
        return self.hull_worlds[@intFromEnum(hull_type)].check(point);
    }
};

// Rendering
pub const Vertex = extern struct { pos: [3]f32, col: [4]f32 };

pub const MeshBuilder = struct {
    vertices: std.ArrayListUnmanaged(Vertex) = .{},
    indices: std.ArrayListUnmanaged(u16) = .{},
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) MeshBuilder { return .{ .allocator = allocator }; }
    pub fn deinit(self: *MeshBuilder) void { self.vertices.deinit(self.allocator); self.indices.deinit(self.allocator); }
    
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
        // generate convex hull
        {
            const vertices = hull_vertices.items;
            const base = @as(u16, @intCast(self.vertices.items.len));
            
            for (vertices) |v| {
                try self.vertices.append(self.allocator, .{ .pos = .{ v.data[0], v.data[1], v.data[2] }, .col = color });
            }
            
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
                            if ((side_sign orelse 1) > 0) {
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(i)));
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(k)));
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(j)));
                            } else {
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(i)));
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(j)));
                                try self.indices.append(self.allocator, base + @as(u16, @intCast(k)));
                            }
                        }
                    }
                }
            }
        }
    }
};
    
