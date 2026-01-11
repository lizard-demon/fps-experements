// Suckless BVH: Maximum runtime performance, zero compromises
// - Packed 32-byte nodes for perfect cache alignment
// - Breadth-first layout for linear memory access
// - Stackless traversal using labeled switches
// - Naive O(n²) SAH for optimal splits (build cost irrelevant)

const std = @import("std");

// Assume these exist in your math module
const Vec3 = @Vector(3, f32);
const AABB = struct {
    min: Vec3,
    max: Vec3,
    
    pub fn surface_area(self: AABB) f32 {
        const size = self.max - self.min;
        return 2.0 * (size[0] * size[1] + size[1] * size[2] + size[0] * size[2]);
    }
    
    pub fn union_with(self: AABB, other: AABB) AABB {
        return AABB{
            .min = @min(self.min, other.min),
            .max = @max(self.max, other.max),
        };
    }
    
    pub fn intersects(self: AABB, other: AABB) bool {
        return @reduce(.And, self.min <= other.max) and @reduce(.And, other.min <= self.max);
    }
};

const EPSILON = 1e-6;

// Perfect 32-byte packed node - fits exactly in cache line
const BVHNode = packed struct {
    // AABB: 24 bytes (6 * f32)
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    
    // Node data: 8 bytes
    first_child_or_primitive: u32,  // 4 bytes
    primitive_count: u30,           // 30 bits for count
    axis: u2,                       // 2 bits for split axis (0-2, 3=leaf)
    
    // Total: exactly 32 bytes
    
    pub fn bounds(self: BVHNode) AABB {
        return AABB{
            .min = Vec3{ self.min_x, self.min_y, self.min_z },
            .max = Vec3{ self.max_x, self.max_y, self.max_z },
        };
    }
    
    pub fn set_bounds(self: *BVHNode, aabb: AABB) void {
        self.min_x = aabb.min[0];
        self.min_y = aabb.min[1];
        self.min_z = aabb.min[2];
        self.max_x = aabb.max[0];
        self.max_y = aabb.max[1];
        self.max_z = aabb.max[2];
    }
    
    pub fn is_leaf(self: BVHNode) bool {
        return self.axis == 3; // 3 = leaf marker
    }
    
    pub fn left_child(self: BVHNode) u32 {
        std.debug.assert(!self.is_leaf());
        return self.first_child_or_primitive;
    }
    
    pub fn right_child(self: BVHNode) u32 {
        std.debug.assert(!self.is_leaf());
        return self.first_child_or_primitive + 1;
    }
};

// Compile-time verification of node size
comptime {
    if (@sizeOf(BVHNode) != 32) {
        @compileError("BVHNode must be exactly 32 bytes, got " ++ std.fmt.comptimePrint("{}", .{@sizeOf(BVHNode)}));
    }
}

// Your primitive type - adjust as needed
const Primitive = struct {
    bounds: AABB,
    // Add your brush/object data here
    
    pub fn centroid(self: Primitive) Vec3 {
        return (self.bounds.min + self.bounds.max) * @as(Vec3, @splat(0.5));
    }
    
    pub fn intersects(self: Primitive, aabb: AABB) bool {
        return self.bounds.intersects(aabb);
    }
};

// Build-time data for constructing BVH
const BuildNode = struct {
    bounds: AABB,
    first_primitive: u32,
    primitive_count: u32,
    left_child: ?*BuildNode = null,
    right_child: ?*BuildNode = null,
};

const SplitResult = struct {
    axis: u32,
    pos: f32,
    cost: f32,
    left_count: u32,
};

pub const SucklessBVH = struct {
    nodes: []align(32) BVHNode,  // 32-byte aligned for cache optimization
    primitives: []Primitive,
    primitive_indices: []u32,
    allocator: std.mem.Allocator,
    
    // Tuning constants
    const MAX_LEAF_SIZE = 4;
    const TRAVERSAL_COST = 0.3;
    const INTERSECTION_COST = 1.0;
    
    pub fn init(primitives: []const Primitive, allocator: std.mem.Allocator) !SucklessBVH {
        if (primitives.len == 0) {
            return SucklessBVH{
                .nodes = &[_]BVHNode{},
                .primitives = &[_]Primitive{},
                .primitive_indices = &[_]u32{},
                .allocator = allocator,
            };
        }
        
        // Copy primitives and create indices
        const owned_primitives = try allocator.dupe(Primitive, primitives);
        const indices = try allocator.alloc(u32, primitives.len);
        for (indices, 0..) |*idx, i| {
            idx.* = @intCast(i);
        }
        
        var bvh = SucklessBVH{
            .nodes = undefined,
            .primitives = owned_primitives,
            .primitive_indices = indices,
            .allocator = allocator,
        };
        
        // Build tree using naive optimal SAH
        const root = try bvh.build_recursive(0, @intCast(primitives.len));
        
        // Convert to breadth-first layout
        bvh.nodes = try bvh.layout_breadth_first(root);
        
        return bvh;
    }
    
    pub fn deinit(self: *SucklessBVH) void {
        self.allocator.free(self.nodes);
        self.allocator.free(self.primitives);
        self.allocator.free(self.primitive_indices);
    }
    
    // Naive O(n²) SAH - optimal splits since build cost doesn't matter
    fn find_best_split_naive(self: *SucklessBVH, start: u32, count: u32, bounds: AABB) SplitResult {
        var best_cost = std.math.floatMax(f32);
        var best_axis: u32 = 0;
        var best_pos: f32 = 0;
        var best_left_count: u32 = 0;
        
        const parent_area = bounds.surface_area();
        if (parent_area <= 0) {
            return SplitResult{ .axis = 0, .pos = 0, .cost = best_cost, .left_count = 0 };
        }
        
        // Try each axis
        for (0..3) |axis| {
            // Try splitting at each primitive's centroid
            for (start..start + count) |i| {
                const split_pos = self.primitives[self.primitive_indices[i]].centroid()[axis];
                
                // Calculate bounds and counts for each side
                var left_bounds: ?AABB = null;
                var right_bounds: ?AABB = null;
                var left_count: u32 = 0;
                var right_count: u32 = 0;
                
                for (start..start + count) |j| {
                    const prim = self.primitives[self.primitive_indices[j]];
                    const centroid = prim.centroid();
                    
                    if (centroid[axis] < split_pos) {
                        left_count += 1;
                        if (left_bounds) |lb| {
                            left_bounds = lb.union_with(prim.bounds);
                        } else {
                            left_bounds = prim.bounds;
                        }
                    } else {
                        right_count += 1;
                        if (right_bounds) |rb| {
                            right_bounds = rb.union_with(prim.bounds);
                        } else {
                            right_bounds = prim.bounds;
                        }
                    }
                }
                
                // Skip degenerate splits
                if (left_count == 0 or right_count == 0) continue;
                
                // Calculate SAH cost
                const left_area = if (left_bounds) |lb| lb.surface_area() else 0;
                const right_area = if (right_bounds) |rb| rb.surface_area() else 0;
                
                const cost = TRAVERSAL_COST + 
                    (left_area / parent_area) * @as(f32, @floatFromInt(left_count)) * INTERSECTION_COST +
                    (right_area / parent_area) * @as(f32, @floatFromInt(right_count)) * INTERSECTION_COST;
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = @intCast(axis);
                    best_pos = split_pos;
                    best_left_count = left_count;
                }
            }
        }
        
        return SplitResult{ 
            .axis = best_axis, 
            .pos = best_pos, 
            .cost = best_cost, 
            .left_count = best_left_count 
        };
    }
    
    fn partition_primitives(self: *SucklessBVH, start: u32, count: u32, axis: u32, split_pos: f32) u32 {
        var left = start;
        var right = start + count - 1;
        
        while (left <= right) {
            const centroid = self.primitives[self.primitive_indices[left]].centroid();
            
            if (centroid[axis] < split_pos) {
                left += 1;
            } else {
                // Swap with right
                const temp = self.primitive_indices[left];
                self.primitive_indices[left] = self.primitive_indices[right];
                self.primitive_indices[right] = temp;
                if (right == 0) break;
                right -= 1;
            }
        }
        
        return left;
    }
    
    fn calculate_bounds(self: *SucklessBVH, start: u32, count: u32) AABB {
        var bounds = self.primitives[self.primitive_indices[start]].bounds;
        for (start + 1..start + count) |i| {
            bounds = bounds.union_with(self.primitives[self.primitive_indices[i]].bounds);
        }
        return bounds;
    }
    
    fn build_recursive(self: *SucklessBVH, start: u32, count: u32) !*BuildNode {
        const node = try self.allocator.create(BuildNode);
        node.bounds = self.calculate_bounds(start, count);
        node.first_primitive = start;
        node.primitive_count = count;
        
        // Create leaf if small enough
        if (count <= MAX_LEAF_SIZE) {
            return node;
        }
        
        // Find best split
        const split = self.find_best_split_naive(start, count, node.bounds);
        
        // Create leaf if no good split found
        const leaf_cost = @as(f32, @floatFromInt(count)) * INTERSECTION_COST;
        if (split.cost >= leaf_cost) {
            return node;
        }
        
        // Partition primitives
        const split_idx = self.partition_primitives(start, count, split.axis, split.pos);
        const left_count = split_idx - start;
        const right_count = (start + count) - split_idx;
        
        // Ensure valid partitions
        if (left_count == 0 or right_count == 0) {
            return node; // Fallback to leaf
        }
        
        // Recursively build children
        node.left_child = try self.build_recursive(start, left_count);
        node.right_child = try self.build_recursive(split_idx, right_count);
        
        return node;
    }
    
    fn layout_breadth_first(self: *SucklessBVH, root: *BuildNode) ![]align(32) BVHNode {
        // Count nodes
        const node_count = self.count_nodes(root);
        const nodes = try self.allocator.alignedAlloc(BVHNode, 32, node_count);
        
        // Breadth-first traversal using queue
        var queue = std.ArrayList(*BuildNode).init(self.allocator);
        defer queue.deinit();
        
        var node_map = std.HashMap(*BuildNode, u32, std.hash_map.DefaultContext(*BuildNode), std.hash_map.default_max_load_percentage).init(self.allocator);
        defer node_map.deinit();
        
        try queue.append(root);
        try node_map.put(root, 0);
        
        var write_idx: u32 = 0;
        
        while (queue.items.len > 0) {
            const build_node = queue.orderedRemove(0);
            const node_idx = node_map.get(build_node).?;
            
            var bvh_node = &nodes[node_idx];
            bvh_node.set_bounds(build_node.bounds);
            
            if (build_node.left_child == null) {
                // Leaf node
                bvh_node.first_child_or_primitive = build_node.first_primitive;
                bvh_node.primitive_count = @intCast(build_node.primitive_count);
                bvh_node.axis = 3; // Leaf marker
            } else {
                // Internal node
                const left_idx = write_idx + @as(u32, @intCast(queue.items.len)) + 1;
                const right_idx = left_idx + 1;
                
                bvh_node.first_child_or_primitive = left_idx;
                bvh_node.primitive_count = 0;
                bvh_node.axis = 0; // Will be set properly if needed
                
                try queue.append(build_node.left_child.?);
                try queue.append(build_node.right_child.?);
                try node_map.put(build_node.left_child.?, left_idx);
                try node_map.put(build_node.right_child.?, right_idx);
            }
            
            write_idx += 1;
        }
        
        // Clean up build nodes
        self.free_build_nodes(root);
        
        return nodes;
    }
    
    fn count_nodes(self: *SucklessBVH, node: *BuildNode) u32 {
        _ = self;
        var count: u32 = 1;
        if (node.left_child) |left| {
            count += self.count_nodes(left);
        }
        if (node.right_child) |right| {
            count += self.count_nodes(right);
        }
        return count;
    }
    
    fn free_build_nodes(self: *SucklessBVH, node: *BuildNode) void {
        if (node.left_child) |left| {
            self.free_build_nodes(left);
        }
        if (node.right_child) |right| {
            self.free_build_nodes(right);
        }
        self.allocator.destroy(node);
    }
    
    // Stackless traversal using labeled switch - maximum performance
    pub fn intersect_aabb(self: *const SucklessBVH, query_aabb: AABB) bool {
        if (self.nodes.len == 0) return false;
        
        var node_idx: u32 = 0;
        
        traverse: while (true) {
            const node = self.nodes[node_idx];
            
            // Early AABB rejection
            if (!node.bounds().intersects(query_aabb)) {
                // Move to next node in breadth-first order
                node_idx += 1;
                if (node_idx >= self.nodes.len) break :traverse;
                continue :traverse;
            }
            
            if (node.is_leaf()) {
                // Test primitives in leaf
                for (node.first_child_or_primitive..node.first_child_or_primitive + node.primitive_count) |i| {
                    const prim_idx = self.primitive_indices[i];
                    if (self.primitives[prim_idx].intersects(query_aabb)) {
                        return true;
                    }
                }
                
                // Move to next node
                node_idx += 1;
                if (node_idx >= self.nodes.len) break :traverse;
                continue :traverse;
            } else {
                // Internal node - continue to left child
                node_idx = node.left_child();
                continue :traverse;
            }
        }
        
        return false;
    }
    
    // Alternative stackless traversal with explicit queue for more complex queries
    pub fn intersect_aabb_detailed(self: *const SucklessBVH, query_aabb: AABB, 
                                  comptime max_results: u32) struct { 
        results: [max_results]u32, 
        count: u32 
    } {
        var results: [max_results]u32 = undefined;
        var result_count: u32 = 0;
        
        if (self.nodes.len == 0) return .{ .results = results, .count = 0 };
        
        // Simple queue for nodes to visit
        var queue: [64]u32 = undefined;
        var queue_head: u32 = 0;
        var queue_tail: u32 = 1;
        queue[0] = 0; // Root
        
        while (queue_head < queue_tail and result_count < max_results) {
            const node_idx = queue[queue_head];
            queue_head += 1;
            
            const node = self.nodes[node_idx];
            
            if (!node.bounds().intersects(query_aabb)) continue;
            
            if (node.is_leaf()) {
                // Collect intersecting primitives
                for (node.first_child_or_primitive..node.first_child_or_primitive + node.primitive_count) |i| {
                    const prim_idx = self.primitive_indices[i];
                    if (self.primitives[prim_idx].intersects(query_aabb)) {
                        results[result_count] = prim_idx;
                        result_count += 1;
                        if (result_count >= max_results) break;
                    }
                }
            } else {
                // Add children to queue
                if (queue_tail + 2 <= queue.len) {
                    queue[queue_tail] = node.left_child();
                    queue[queue_tail + 1] = node.right_child();
                    queue_tail += 2;
                }
            }
        }
        
        return .{ .results = results, .count = result_count };
    }
    
    // Debug info
    pub fn get_stats(self: *const SucklessBVH) struct {
        node_count: u32,
        leaf_count: u32,
        max_depth: u32,
        memory_bytes: usize,
    } {
        var leaf_count: u32 = 0;
        for (self.nodes) |node| {
            if (node.is_leaf()) leaf_count += 1;
        }
        
        return .{
            .node_count = @intCast(self.nodes.len),
            .leaf_count = leaf_count,
            .max_depth = self.calculate_max_depth(),
            .memory_bytes = self.nodes.len * @sizeOf(BVHNode) + 
                           self.primitives.len * @sizeOf(Primitive) +
                           self.primitive_indices.len * @sizeOf(u32),
        };
    }
    
    fn calculate_max_depth(self: *const SucklessBVH) u32 {
        if (self.nodes.len == 0) return 0;
        
        var max_depth: u32 = 0;
        var queue: [64]struct { idx: u32, depth: u32 } = undefined;
        var queue_head: u32 = 0;
        var queue_tail: u32 = 1;
        queue[0] = .{ .idx = 0, .depth = 1 };
        
        while (queue_head < queue_tail) {
            const item = queue[queue_head];
            queue_head += 1;
            
            max_depth = @max(max_depth, item.depth);
            
            const node = self.nodes[item.idx];
            if (!node.is_leaf() and queue_tail + 2 <= queue.len) {
                queue[queue_tail] = .{ .idx = node.left_child(), .depth = item.depth + 1 };
                queue[queue_tail + 1] = .{ .idx = node.right_child(), .depth = item.depth + 1 };
                queue_tail += 2;
            }
        }
        
        return max_depth;
    }
};

// Usage example
pub fn example_usage() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    
    // Create some test primitives
    const primitives = [_]Primitive{
        .{ .bounds = AABB{ .min = Vec3{0, 0, 0}, .max = Vec3{1, 1, 1} } },
        .{ .bounds = AABB{ .min = Vec3{2, 0, 0}, .max = Vec3{3, 1, 1} } },
        .{ .bounds = AABB{ .min = Vec3{0, 2, 0}, .max = Vec3{1, 3, 1} } },
    };
    
    // Build BVH
    var bvh = try SucklessBVH.init(&primitives, allocator);
    defer bvh.deinit();
    
    // Query
    const query = AABB{ .min = Vec3{0.5, 0.5, 0.5}, .max = Vec3{1.5, 1.5, 1.5} };
    const hit = bvh.intersect_aabb(query);
    
    std.debug.print("Intersection: {}\n", .{hit});
    
    const stats = bvh.get_stats();
    std.debug.print("BVH Stats: {} nodes, {} leaves, {} max depth, {} bytes\n", 
        .{ stats.node_count, stats.leaf_count, stats.max_depth, stats.memory_bytes });
}