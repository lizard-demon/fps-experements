const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

// BVH configuration constants
const BVH_CONFIG = struct {
    const traversal_cost: f32 = 0.3;
    const max_leaf_size: u32 = 4;
    const epsilon: f32 = 1e-6;
    const split_candidates: u32 = 8;
    const max_stack_depth: u32 = 64;
};

// Collision types
pub const HullType = enum { point, standing };
pub const CollisionResult = struct { normal: Vec3, distance: f32 };

pub const Capsule = struct {
    start: Vec3, end: Vec3, radius: f32,
    
    pub fn fromHull(pos: Vec3, hull_type: HullType) Capsule {
        return switch (hull_type) {
            .point => .{ .start = pos, .end = pos, .radius = 0.0 },
            .standing => .{ 
                .start = Vec3.add(pos, Vec3.new(0, -0.7, 0)), 
                .end = Vec3.add(pos, Vec3.new(0, 0.7, 0)), 
                .radius = 0.3 
            },
        };
    }
    
    pub inline fn center(self: Capsule) Vec3 {
        return Vec3.scale(Vec3.add(self.start, self.end), 0.5);
    }
    
    pub inline fn bounds(self: Capsule) AABB {
        const radius_vec = Vec3.new(self.radius, self.radius, self.radius);
        return AABB{
            .min = Vec3.sub(Vec3.min(self.start, self.end), radius_vec),
            .max = Vec3.add(Vec3.max(self.start, self.end), radius_vec),
        };
    }
};

pub const Plane = struct {
    normal: Vec3, distance: f32,
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 { 
        return Vec3.dot(self.normal, point) + self.distance; 
    }
};

pub const Brush = struct {
    planes: []const Plane, 
    bounds: AABB,
    
    pub fn checkCapsule(self: Brush, capsule: Capsule) ?CollisionResult {
        var closest_normal: ?Vec3 = null;
        var closest_distance: f32 = std.math.floatMax(f32);
        
        for (self.planes) |plane| {
            const distance = plane.distanceToPoint(capsule.center()) - capsule.radius;
            if (distance > math.EPSILON) return null;
            if (distance > -closest_distance) {
                closest_distance = -distance;
                closest_normal = plane.normal;
            }
        }
        
        return if (closest_normal) |normal| 
            CollisionResult{ .normal = normal, .distance = closest_distance } 
        else null;
    }
    
    pub fn getBounds(self: Brush) AABB {
        return self.bounds;
    }
};

pub const CollisionWorld = struct {
    bvh_tree: BVH(Brush),
    original_brushes: []const Brush,
    allocator: std.mem.Allocator,
    
    pub fn init(brushes: []const Brush, allocator: std.mem.Allocator) !CollisionWorld {
        return CollisionWorld{
            .original_brushes = brushes,
            .bvh_tree = try BVH(Brush).init(brushes, allocator, Brush.getBounds),
            .allocator = allocator,
        };
    }
    
    pub fn deinit(self: *CollisionWorld) void {
        self.bvh_tree.deinit();
    }
    
    pub fn check(self: *const CollisionWorld, point: Vec3) ?CollisionResult {
        const capsule = Capsule.fromHull(point, .standing);
        return self.checkCapsule(capsule);
    }
    
    pub fn checkCapsule(self: *const CollisionWorld, capsule: Capsule) ?CollisionResult {
        if (self.bvh_tree.items.len == 0) {
            for (self.original_brushes) |brush| if (brush.checkCapsule(capsule)) |collision| return collision;
            return null;
        }
        
        const capsule_bounds = capsule.bounds();
        var best_collision: ?CollisionResult = null;
        var best_distance: f32 = -std.math.floatMax(f32);
        
        var stack_ptr: u32 = 0;
        var stack: [BVH_CONFIG.max_stack_depth]u32 = undefined;
        stack[0] = 0;
        stack_ptr = 1;
        
        while (stack_ptr > 0) {
            stack_ptr -= 1;
            const node_idx = stack[stack_ptr];
            
            if (node_idx >= self.bvh_tree.nodes.items.len) continue;
            
            const node_data = @as(*const Node, @ptrCast(&self.bvh_tree.nodes.items[node_idx]));
            
            const node_bounds = AABB{ 
                .min = Vec3.new(node_data.min_x, node_data.min_y, node_data.min_z), 
                .max = Vec3.new(node_data.max_x, node_data.max_y, node_data.max_z) 
            };
            
            if (!node_bounds.intersects(capsule_bounds)) continue;
            
            const is_leaf = (node_data.count_and_axis & 0x3) == 3;
            if (is_leaf) {
                const count = node_data.count_and_axis >> 2;
                const end_idx = @min(node_data.first + count, self.bvh_tree.indices.items.len);
                for (node_data.first..end_idx) |i| {
                    if (self.bvh_tree.items[self.bvh_tree.indices.items[i]].checkCapsule(capsule)) |collision| {
                        if (collision.distance > best_distance) {
                            best_distance = collision.distance;
                            best_collision = collision;
                            
                            if (collision.distance > 0) return collision;
                        }
                    }
                }
            } else {
                const left_child = node_data.first;
                const right_child = left_child + 1;
                
                if (right_child < self.bvh_tree.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = right_child;
                    stack_ptr += 1;
                }
                if (left_child < self.bvh_tree.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = left_child;
                    stack_ptr += 1;
                }
            }
        }
        
        return best_collision;
    }
};

const Node = packed struct {
    // Reorder fields for better packing and cache efficiency
    first: u32, 
    count_and_axis: u32, // Pack count (30 bits) and axis (2 bits) together
    min_x: f32, min_y: f32, min_z: f32, 
    max_x: f32, max_y: f32, max_z: f32,
    
    fn bounds(self: Node) AABB { 
        return AABB{ .min = Vec3.new(self.min_x, self.min_y, self.min_z), .max = Vec3.new(self.max_x, self.max_y, self.max_z) }; 
    }
    fn isLeaf(self: Node) bool { return (self.count_and_axis & 0x3) == 3; }
    fn left(self: Node) u32 { return self.first; }
    fn count(self: Node) u32 { return self.count_and_axis >> 2; }
    fn axis(self: Node) u32 { return self.count_and_axis & 0x3; }
    
    fn setCountAndAxis(count_val: u32, axis_val: u32) u32 {
        return (count_val << 2) | (axis_val & 0x3);
    }
};

pub fn BVH(comptime T: type) type {
    return struct {
        const Self = @This();
        
        items: []const T,
        nodes: std.ArrayListUnmanaged(Node),
        indices: std.ArrayListUnmanaged(u32), 
        allocator: std.mem.Allocator,
        
        // Pre-allocated traversal stack to avoid allocations during queries
        traversal_stack: [BVH_CONFIG.max_stack_depth]u32 = undefined,
        
        pub fn init(items: []const T, allocator: std.mem.Allocator, bounds_fn: fn(T) AABB) !Self {
            if (items.len == 0) return .{ 
                .items = items, 
                .nodes = .{}, 
                .indices = .{}, 
                .allocator = allocator 
            };
            
            var indices = std.ArrayListUnmanaged(u32){};
            try indices.ensureTotalCapacity(allocator, items.len);
            for (0..items.len) |i| {
                indices.appendAssumeCapacity(@intCast(i));
            }
            
            var bvh = Self{ 
                .items = items, 
                .nodes = .{}, 
                .indices = indices, 
                .allocator = allocator 
            };
            _ = try bvh.build(bounds_fn);
            return bvh;
        }
        
        pub fn deinit(self: *Self) void {
            self.nodes.deinit(self.allocator);
            self.indices.deinit(self.allocator);
        }
        
        pub fn query(self: *const Self, query_bounds: AABB, test_fn: fn(T) bool) bool {
            if (self.nodes.items.len == 0) {
                for (self.items) |item| if (test_fn(item)) return true;
                return false;
            }
            
            // Use pre-allocated stack for traversal - breadth-first layout benefits
            var stack_ptr: u32 = 0;
            var stack: [BVH_CONFIG.max_stack_depth]u32 = undefined;
            stack[0] = 0;
            stack_ptr = 1;
            
            while (stack_ptr > 0) {
                stack_ptr -= 1;
                const node_idx = stack[stack_ptr];
                
                if (node_idx >= self.nodes.items.len) continue;
                const node = self.nodes.items[node_idx];
                
                if (!node.bounds().intersects(query_bounds)) continue;
                
                if (node.isLeaf()) {
                    const end_idx = @min(node.first + node.count(), self.indices.items.len);
                    for (node.first..end_idx) |i| {
                        if (test_fn(self.items[self.indices.items[i]])) return true;
                    }
                } else {
                    // In breadth-first layout, children are stored sequentially
                    // This provides better cache locality when accessing both children
                    const left_child = node.left();
                    const right_child = left_child + 1;
                    
                    // Add children to stack - breadth-first layout ensures they're cache-friendly
                    if (right_child < self.nodes.items.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = right_child;
                        stack_ptr += 1;
                    }
                    if (left_child < self.nodes.items.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = left_child;
                        stack_ptr += 1;
                    }
                }
            }
            return false;
        }
        
        fn build(self: *Self, bounds_fn: fn(T) AABB) ![]Node {
            var nodes = std.ArrayListUnmanaged(Node){};
            defer nodes.deinit(self.allocator);
            _ = try self.buildRecursive(0, @intCast(self.items.len), &nodes, bounds_fn, .{});
            
            // Use breadth-first layout for better cache locality during traversal
            return try self.layoutBreadthFirst(nodes.items);
        }
        
        fn buildRecursive(self: *Self, start: u32, count: u32, nodes: *std.ArrayListUnmanaged(Node), bounds_fn: fn(T) AABB, comptime bvh_config: struct {
            max_leaf_size: u32 = BVH_CONFIG.max_leaf_size,
        }) !u32 {
            const node_idx = @as(u32, @intCast(nodes.items.len));
            var bounds = bounds_fn(self.items[self.indices.items[start]]);
            for (start + 1..start + count) |i| bounds = bounds.union_with(bounds_fn(self.items[self.indices.items[i]]));
            
            try nodes.append(self.allocator, Node{
                .first = start, 
                .count_and_axis = Node.setCountAndAxis(count, 3),
                .min_x = bounds.min.data[0], .min_y = bounds.min.data[1], .min_z = bounds.min.data[2],
                .max_x = bounds.max.data[0], .max_y = bounds.max.data[1], .max_z = bounds.max.data[2],
            });
            
            if (count <= bvh_config.max_leaf_size) return node_idx;
            
            const split = self.findBestSplit(start, count, bounds, bounds_fn, .{});
            if (split.cost >= @as(f32, @floatFromInt(count))) return node_idx;
            
            const split_idx = self.partition(start, count, split.axis, split.pos, bounds_fn);
            const left_count = split_idx - start;
            const right_count = (start + count) - split_idx;
            if (left_count == 0 or right_count == 0) return node_idx;
            
            const left_child = try self.buildRecursive(start, left_count, nodes, bounds_fn, bvh_config);
            _ = try self.buildRecursive(split_idx, right_count, nodes, bounds_fn, bvh_config);
            
            nodes.items[node_idx].first = left_child;
            nodes.items[node_idx].count_and_axis = Node.setCountAndAxis(0, split.axis);
            return node_idx;
        }
        
        const Split = struct { axis: u32, pos: f32, cost: f32 };
        
        fn findBestSplit(self: *Self, start: u32, count: u32, bounds: AABB, bounds_fn: fn(T) AABB, comptime bvh_config: struct {
            traversal_cost: f32 = BVH_CONFIG.traversal_cost,
            epsilon: f32 = BVH_CONFIG.epsilon,
            split_candidates: u32 = BVH_CONFIG.split_candidates,
        }) Split {
            var best = Split{ .axis = 0, .pos = 0, .cost = std.math.floatMax(f32) };
            const parent_area = bounds.surface_area();
            if (parent_area <= 0) return best;
            
            // Use binned SAH for O(n) split finding instead of O(n²)
            for (0..3) |axis| {
                const axis_min = bounds.min.data[axis];
                const axis_max = bounds.max.data[axis];
                const axis_range = axis_max - axis_min;
                if (axis_range <= bvh_config.epsilon) continue;
                
                // Test fewer, evenly distributed split candidates
                const step = axis_range / @as(f32, @floatFromInt(bvh_config.split_candidates));
                var candidate: u32 = 0;
                while (candidate < bvh_config.split_candidates) : (candidate += 1) {
                    const split_pos = axis_min + step * (@as(f32, @floatFromInt(candidate)) + 0.5);
                    
                    var left_bounds: ?AABB = null;
                    var right_bounds: ?AABB = null;
                    var left_count: u32 = 0;
                    var right_count: u32 = 0;
                    
                    for (start..start + count) |j| {
                        const prim_bounds = bounds_fn(self.items[self.indices.items[j]]);
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
                    const cost = bvh_config.traversal_cost + (left_area / parent_area) * @as(f32, @floatFromInt(left_count)) + (right_area / parent_area) * @as(f32, @floatFromInt(right_count));
                    
                    if (cost < best.cost) {
                        best = .{ .axis = @intCast(axis), .pos = split_pos, .cost = cost };
                    }
                }
            }
            return best;
        }
        
        fn partition(self: *Self, start: u32, count: u32, axis: u32, split_pos: f32, bounds_fn: fn(T) AABB) u32 {
            var left = start;
            var right = start + count - 1;
            while (left <= right) {
                const centroid = (bounds_fn(self.items[self.indices.items[left]]).min.data[axis] + bounds_fn(self.items[self.indices.items[left]]).max.data[axis]) * 0.5;
                if (centroid < split_pos) {
                    left += 1;
                } else {
                    const temp = self.indices.items[left];
                    self.indices.items[left] = self.indices.items[right];
                    self.indices.items[right] = temp;
                    if (right == 0) break;
                    right -= 1;
                }
            }
            return left;
        }
        
        fn layoutBreadthFirst(self: *Self, tree_nodes: []Node) ![]Node {
            if (tree_nodes.len == 0) return try self.allocator.alloc(Node, 0);
            
            const nodes = try self.allocator.alloc(Node, tree_nodes.len);
            
            // Use a more efficient queue with pre-allocated size
            var queue: [512]u32 = undefined; // Increased size for larger trees
            var queue_head: u32 = 0;
            var queue_tail: u32 = 1;
            queue[0] = 0;
            var write_idx: u32 = 0;
            
            while (queue_head < queue_tail and write_idx < nodes.len) {
                const tree_idx = queue[queue_head];
                queue_head += 1;
                
                if (tree_idx >= tree_nodes.len) continue;
                const tree_node = tree_nodes[tree_idx];
                nodes[write_idx] = tree_node;
                
                if (!tree_node.isLeaf() and queue_tail + 1 < queue.len) {
                    const left_tree_idx = tree_node.first;
                    const right_tree_idx = left_tree_idx + 1;
                    
                    // Update the breadth-first index for the left child
                    const left_bf_idx = write_idx + (queue_tail - queue_head) + 1;
                    nodes[write_idx].first = left_bf_idx;
                    
                    // Add children to queue
                    if (left_tree_idx < tree_nodes.len) { 
                        queue[queue_tail] = left_tree_idx; 
                        queue_tail += 1; 
                    }
                    if (right_tree_idx < tree_nodes.len and queue_tail < queue.len) { 
                        queue[queue_tail] = right_tree_idx; 
                        queue_tail += 1; 
                    }
                }
                write_idx += 1;
            }
            
            self.nodes = std.ArrayListUnmanaged(Node).fromOwnedSlice(nodes[0..write_idx]);
            return self.nodes.items;
        }
    };
}