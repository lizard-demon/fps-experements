const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const CollisionResult = struct { hit: bool = false, normal: Vec3 = Vec3.zero(), distance: f32 = 0.0 };

pub const Node = packed struct {
    min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32,
    first: u32, count: u30, axis: u2,
    
    fn bounds(self: Node) AABB { return AABB{ .min = Vec3.new(self.min_x, self.min_y, self.min_z), .max = Vec3.new(self.max_x, self.max_y, self.max_z) }; }
    fn setBounds(self: *Node, aabb: AABB) void { self.min_x = aabb.min.data[0]; self.min_y = aabb.min.data[1]; self.min_z = aabb.min.data[2]; self.max_x = aabb.max.data[0]; self.max_y = aabb.max.data[1]; self.max_z = aabb.max.data[2]; }
    fn isLeaf(self: Node) bool { return self.axis == 3; }
    fn left(self: Node) u32 { return self.first; }
};
comptime { if (@sizeOf(Node) != 32) @compileError("Node must be 32 bytes"); }

pub const Plane = struct {
    normal: Vec3, distance: f32,
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 { return Vec3.dot(self.normal, point) + self.distance; }
};

pub const Brush = struct {
    planes: []const Plane, bounds: AABB,
    
    pub fn intersects(self: Brush, aabb: AABB) bool {
        // Quick bounds check first
        if (!self.bounds.intersects(aabb)) return false;
        
        // For a convex brush, we need to check if the AABB intersects the brush volume
        // This means checking if any part of the AABB is inside ALL planes
        
        // Test all 8 corners of the AABB
        const corners = [_]Vec3{
            Vec3.new(aabb.min.data[0], aabb.min.data[1], aabb.min.data[2]),
            Vec3.new(aabb.max.data[0], aabb.min.data[1], aabb.min.data[2]),
            Vec3.new(aabb.min.data[0], aabb.max.data[1], aabb.min.data[2]),
            Vec3.new(aabb.max.data[0], aabb.max.data[1], aabb.min.data[2]),
            Vec3.new(aabb.min.data[0], aabb.min.data[1], aabb.max.data[2]),
            Vec3.new(aabb.max.data[0], aabb.min.data[1], aabb.max.data[2]),
            Vec3.new(aabb.min.data[0], aabb.max.data[1], aabb.max.data[2]),
            Vec3.new(aabb.max.data[0], aabb.max.data[1], aabb.max.data[2]),
        };
        
        // If any corner is inside the brush, we have intersection
        for (corners) |corner| {
            var inside = true;
            for (self.planes) |plane| {
                if (plane.distanceToPoint(corner) > math.EPSILON) {
                    inside = false;
                    break;
                }
            }
            if (inside) return true;
        }
        
        // Also check if the brush intersects the AABB using separating axis theorem
        for (self.planes) |plane| {
            const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
            const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            
            const distance = plane.distanceToPoint(center);
            if (distance > radius + math.EPSILON) return false;
        }
        return true;
    }
    
    // Get collision information including surface normal
    pub fn getCollisionInfo(self: Brush, aabb: AABB) CollisionResult {
        if (!self.bounds.intersects(aabb)) return .{};
        
        const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
        const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
        
        var closest_distance: f32 = std.math.floatMax(f32);
        var collision_normal = Vec3.zero();
        var has_collision = false;
        
        // Find the closest penetrating plane
        for (self.planes) |plane| {
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            
            const distance = plane.distanceToPoint(center);
            const penetration = radius - distance;
            
            // If penetrating this plane and it's the closest
            if (penetration > math.EPSILON and distance < closest_distance) {
                closest_distance = distance;
                collision_normal = plane.normal;
                has_collision = true;
            }
        }
        
        return .{ .hit = has_collision, .normal = collision_normal, .distance = closest_distance };
    }
};

// Rendering vertex
pub const Vertex = extern struct { 
    pos: [3]f32, 
    col: [4]f32 
};

// Optimal mesh builder for arbitrary convex brushes
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
    
    // Core algorithm: Generate mesh from arbitrary convex brush
    pub fn addBrush(self: *MeshBuilder, brush: Brush, color: [4]f32) !void {
        // Step 1: Find all vertices by intersecting plane triplets
        var hull_vertices = std.ArrayListUnmanaged(Vec3){};
        defer hull_vertices.deinit(self.allocator);
        
        for (0..brush.planes.len) |i| {
            for (i + 1..brush.planes.len) |j| {
                for (j + 1..brush.planes.len) |k| {
                    if (intersectThreePlanes(brush.planes[i], brush.planes[j], brush.planes[k])) |vertex| {
                        if (isVertexInsideBrush(vertex, brush) and !isDuplicateVertex(hull_vertices.items, vertex)) {
                            try hull_vertices.append(self.allocator, vertex);
                        }
                    }
                }
            }
        }
        
        if (hull_vertices.items.len < 4) return;
        
        // Step 2: Generate convex hull using gift wrapping algorithm
        try self.generateConvexHull(hull_vertices.items, color);
    }
    
    // Gift wrapping algorithm for convex hull face generation
    fn generateConvexHull(self: *MeshBuilder, vertices: []Vec3, color: [4]f32) !void {
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        // Add vertices to mesh
        for (vertices) |v| {
            try self.vertices.append(self.allocator, .{
                .pos = .{ v.data[0], v.data[1], v.data[2] },
                .col = color
            });
        }
        
        // For each potential face, check if it's on the convex hull
        for (0..vertices.len) |i| {
            for (i + 1..vertices.len) |j| {
                for (j + 1..vertices.len) |k| {
                    const v0 = vertices[i];
                    const v1 = vertices[j]; 
                    const v2 = vertices[k];
                    
                    // Calculate face normal
                    const edge1 = Vec3.sub(v1, v0);
                    const edge2 = Vec3.sub(v2, v0);
                    const normal = Vec3.cross(edge1, edge2);
                    
                    if (Vec3.length(normal) < math.EPSILON) continue;
                    
                    // Check if all other vertices are on one side (convex hull property)
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
                        // Add triangle with correct winding (outward normal)
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
};

// Helper function to intersect three planes and find their common point
fn intersectThreePlanes(p1: Plane, p2: Plane, p3: Plane) ?Vec3 {
    const n1 = p1.normal;
    const n2 = p2.normal;
    const n3 = p3.normal;
    
    // Calculate the determinant of the normal matrix
    const det = Vec3.dot(n1, Vec3.cross(n2, n3));
    
    if (@abs(det) < math.EPSILON) return null; // Planes are parallel or coplanar
    
    // Calculate the intersection point using Cramer's rule
    const c1 = Vec3.cross(n2, n3);
    const c2 = Vec3.cross(n3, n1);
    const c3 = Vec3.cross(n1, n2);
    
    const point = Vec3.scale(
        Vec3.add(
            Vec3.add(
                Vec3.scale(c1, -p1.distance),
                Vec3.scale(c2, -p2.distance)
            ),
            Vec3.scale(c3, -p3.distance)
        ),
        1.0 / det
    );
    
    return point;
}

// Check if a vertex is inside a brush (satisfies all plane constraints)
fn isVertexInsideBrush(vertex: Vec3, brush: Brush) bool {
    for (brush.planes) |plane| {
        if (plane.distanceToPoint(vertex) > math.EPSILON) {
            return false;
        }
    }
    return true;
}

// Check for duplicate vertices within epsilon tolerance
fn isDuplicateVertex(vertices: []Vec3, vertex: Vec3) bool {
    for (vertices) |existing| {
        if (Vec3.dist(vertex, existing) < math.EPSILON * 10) {
            return true;
        }
    }
    return false;
}

const BuildNode = struct { bounds: AABB, start: u32, count: u32, left: ?*BuildNode = null, right: ?*BuildNode = null };

pub const World = struct {
    brushes: []const Brush,
    nodes: []Node,
    indices: []u32,
    allocator: std.mem.Allocator,
    
    pub fn init(brushes: []const Brush, allocator: std.mem.Allocator) !World {
        if (brushes.len == 0) {
            return World{ .brushes = brushes, .nodes = &[_]Node{}, .indices = &[_]u32{}, .allocator = allocator };
        }
        
        const indices = try allocator.alloc(u32, brushes.len);
        for (indices, 0..) |*idx, i| idx.* = @intCast(i);
        
        var world = World{ .brushes = brushes, .nodes = undefined, .indices = indices, .allocator = allocator };
        world.nodes = try world.build();
        
        std.debug.print("BVH built: {} nodes, {} leaves\n", .{ world.nodes.len, world.countLeaves() });
        return world;
    }
    
    pub fn deinit(self: *World) void {
        self.allocator.free(self.nodes);
        self.allocator.free(self.indices);
    }
    
    fn build(self: *World) ![]Node {
        if (self.brushes.len == 0) return try self.allocator.alloc(Node, 0);
        var nodes = std.ArrayListUnmanaged(Node){};
        defer nodes.deinit(self.allocator);
        _ = try self.buildRecursive(0, @intCast(self.brushes.len), &nodes);
        return try self.layoutBreadthFirst(nodes.items);
    }
    
    fn buildRecursive(self: *World, start: u32, count: u32, nodes: *std.ArrayListUnmanaged(Node)) !u32 {
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
    
    fn findBestSplit(self: *World, start: u32, count: u32, bounds: AABB) SplitResult {
        var best_cost = std.math.floatMax(f32);
        var best_axis: u32 = 0;
        var best_pos: f32 = 0;
        const parent_area = bounds.surface_area();
        if (parent_area <= 0) return SplitResult{ .axis = 0, .pos = 0, .cost = best_cost };
        
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
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = @intCast(axis);
                    best_pos = split_pos;
                }
            }
        }
        return SplitResult{ .axis = best_axis, .pos = best_pos, .cost = best_cost };
    }
    
    fn partition(self: *World, start: u32, count: u32, axis: u32, split_pos: f32) u32 {
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
    
    fn layoutBreadthFirst(self: *World, tree_nodes: []Node) ![]Node {
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
    
    // Stackless traversal
    pub fn testCollision(self: *const World, aabb: AABB) bool {
        if (self.nodes.len == 0) {
            for (self.brushes) |brush| if (brush.intersects(aabb)) return true;
            return false;
        }
        
        var node_idx: u32 = 0;
        while (node_idx < self.nodes.len) {
            const node = self.nodes[node_idx];
            if (!node.bounds().intersects(aabb)) { node_idx += 1; continue; }
            
            if (node.isLeaf()) {
                for (node.first..node.first + node.count) |i| {
                    if (self.brushes[self.indices[i]].intersects(aabb)) return true;
                }
                node_idx += 1;
            } else {
                node_idx = node.left();
            }
        }
        return false;
    }
    
    pub fn getCollisionInfo(self: *const World, aabb: AABB) CollisionResult {
        if (self.nodes.len == 0) {
            for (self.brushes) |brush| {
                const result = brush.getCollisionInfo(aabb);
                if (result.hit) return result;
            }
            return .{};
        }
        
        var node_idx: u32 = 0;
        while (node_idx < self.nodes.len) {
            const node = self.nodes[node_idx];
            if (!node.bounds().intersects(aabb)) { node_idx += 1; continue; }
            
            if (node.isLeaf()) {
                for (node.first..node.first + node.count) |i| {
                    const result = self.brushes[self.indices[i]].getCollisionInfo(aabb);
                    if (result.hit) return result;
                }
                node_idx += 1;
            } else {
                node_idx = node.left();
            }
        }
        return .{};
    }
    
    fn countLeaves(self: *const World) u32 {
        var count: u32 = 0;
        for (self.nodes) |node| { if (node.isLeaf()) count += 1; }
        return count;
    }
};