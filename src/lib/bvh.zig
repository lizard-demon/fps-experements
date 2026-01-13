const std = @import("std");
const math = @import("math");
const Vec3 = math.Vec3;
const AABB = math.AABB;

// Configuration for BVH construction
const Config = struct {
    traversal_cost: f32 = 0.3,
    max_leaf_size: u32 = 4,
    epsilon: f32 = 1e-6,
    split_candidates: u32 = 8,
    max_stack_depth: u32 = 64,
};

// BVH Node - packed for cache efficiency
const Node = packed struct {
    first: u32, 
    count_and_axis: u32, // count (30 bits) + axis (2 bits)
    min_x: f32, min_y: f32, min_z: f32, 
    max_x: f32, max_y: f32, max_z: f32,
    
    pub fn bounds(self: Node) AABB { 
        return AABB{ 
            .min = Vec3.new(self.min_x, self.min_y, self.min_z), 
            .max = Vec3.new(self.max_x, self.max_y, self.max_z) 
        }; 
    }
    pub fn isLeaf(self: Node) bool { return (self.count_and_axis & 0x3) == 3; }
    pub fn left(self: Node) u32 { return self.first; }
    pub fn count(self: Node) u32 { return self.count_and_axis >> 2; }
    pub fn axis(self: Node) u32 { return self.count_and_axis & 0x3; }
    fn setCountAndAxis(count_val: u32, axis_val: u32) u32 {
        return (count_val << 2) | (axis_val & 0x3);
    }
};

// Generic BVH implementation
pub fn BVH(comptime T: type) type {
    return struct {
        const Self = @This();
        const config = Config{};
        
        items: []const T,
        nodes: std.ArrayListUnmanaged(Node) = .{},
        indices: std.ArrayListUnmanaged(u32) = .{}, 
        allocator: std.mem.Allocator,
        
        pub fn init(items: []const T, allocator: std.mem.Allocator, bounds_fn: fn(T) AABB) !Self {
            if (items.len == 0) return .{ .items = items, .allocator = allocator };
            
            var indices = std.ArrayListUnmanaged(u32){};
            try indices.ensureTotalCapacity(allocator, items.len);
            for (0..items.len) |i| indices.appendAssumeCapacity(@intCast(i));
            
            var bvh = Self{ .items = items, .indices = indices, .allocator = allocator };
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
            
            var stack: [config.max_stack_depth]u32 = undefined;
            var stack_ptr: u32 = 1;
            stack[0] = 0;
            
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
                    const left_child = node.left();
                    const right_child = left_child + 1;
                    
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
        
        fn build(self: *Self, bounds_fn: fn(T) AABB) !void {
            var nodes = std.ArrayListUnmanaged(Node){};
            defer nodes.deinit(self.allocator);
            _ = try self.buildRecursive(0, @intCast(self.items.len), &nodes, bounds_fn);
            try self.layoutBreadthFirst(nodes.items);
        }
        
        fn buildRecursive(self: *Self, start: u32, count: u32, nodes: *std.ArrayListUnmanaged(Node), bounds_fn: fn(T) AABB) !u32 {
            const node_idx = @as(u32, @intCast(nodes.items.len));
            var bounds = bounds_fn(self.items[self.indices.items[start]]);
            for (start + 1..start + count) |i| {
                bounds = bounds.union_with(bounds_fn(self.items[self.indices.items[i]]));
            }
            
            try nodes.append(self.allocator, Node{
                .first = start, 
                .count_and_axis = Node.setCountAndAxis(count, 3),
                .min_x = bounds.min.data[0], .min_y = bounds.min.data[1], .min_z = bounds.min.data[2],
                .max_x = bounds.max.data[0], .max_y = bounds.max.data[1], .max_z = bounds.max.data[2],
            });
            
            if (count <= config.max_leaf_size) return node_idx;
            
            const split = self.findBestSplit(start, count, bounds, bounds_fn);
            if (split.cost >= @as(f32, @floatFromInt(count))) return node_idx;
            
            const split_idx = self.partition(start, count, split.axis, split.pos, bounds_fn);
            const left_count = split_idx - start;
            const right_count = (start + count) - split_idx;
            if (left_count == 0 or right_count == 0) return node_idx;
            
            const left_child = try self.buildRecursive(start, left_count, nodes, bounds_fn);
            _ = try self.buildRecursive(split_idx, right_count, nodes, bounds_fn);
            
            nodes.items[node_idx].first = left_child;
            nodes.items[node_idx].count_and_axis = Node.setCountAndAxis(0, split.axis);
            return node_idx;
        }
        
        const Split = struct { axis: u32, pos: f32, cost: f32 };
        
        fn findBestSplit(self: *Self, start: u32, count: u32, bounds: AABB, bounds_fn: fn(T) AABB) Split {
            var best = Split{ .axis = 0, .pos = 0, .cost = std.math.floatMax(f32) };
            const parent_area = bounds.surface_area();
            if (parent_area <= 0) return best;
            
            for (0..3) |axis| {
                const axis_min = bounds.min.data[axis];
                const axis_max = bounds.max.data[axis];
                const axis_range = axis_max - axis_min;
                if (axis_range <= config.epsilon) continue;
                
                const step = axis_range / @as(f32, @floatFromInt(config.split_candidates));
                for (0..config.split_candidates) |candidate| {
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
                    const cost = config.traversal_cost + 
                        (left_area / parent_area) * @as(f32, @floatFromInt(left_count)) + 
                        (right_area / parent_area) * @as(f32, @floatFromInt(right_count));
                    
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
                const prim_bounds = bounds_fn(self.items[self.indices.items[left]]);
                const centroid = (prim_bounds.min.data[axis] + prim_bounds.max.data[axis]) * 0.5;
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
        
        fn layoutBreadthFirst(self: *Self, tree_nodes: []Node) !void {
            if (tree_nodes.len == 0) return;
            
            const nodes = try self.allocator.alloc(Node, tree_nodes.len);
            var queue: [512]u32 = undefined;
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
                    
                    const left_bf_idx = write_idx + (queue_tail - queue_head) + 1;
                    nodes[write_idx].first = left_bf_idx;
                    
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
        }
    };
}
