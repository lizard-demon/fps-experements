// High-performance cache-friendly BVH specifically for brush collision
// Optimized for AABB vs Brush queries with minimal memory overhead
const std = @import("std");
const math = @import("math.zig");
const collision = @import("collision.zig");

const Vec3 = math.Vec3;
const AABB = collision.AABB;
const Brush = collision.Brush;

// Compact node structure - 32 bytes, cache line friendly
const BVHNode = struct {
    // AABB bounds (24 bytes)
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    
    // Node data (8 bytes)
    // For internal nodes: child indices (both non-zero)
    // For leaf nodes: first_brush index and count (left_child = 0)
    left_child: u16,    // 0 for leaf nodes, child index for internal nodes
    right_child: u16,   // brush_count for leaf nodes, child index for internal nodes
    first_brush: u16,   // first brush index for leaf nodes, unused for internal
    _padding: u16,      // Keep alignment
    
    pub fn isLeaf(self: *const BVHNode) bool {
        return self.left_child == 0;
    }
    
    pub fn getBrushCount(self: *const BVHNode) u16 {
        std.debug.assert(self.isLeaf());
        return self.right_child; // right_child stores brush_count for leaves
    }
    
    pub fn getFirstBrush(self: *const BVHNode) u16 {
        std.debug.assert(self.isLeaf());
        return self.first_brush;
    }
    
    pub fn getLeftChild(self: *const BVHNode) u16 {
        std.debug.assert(!self.isLeaf());
        return self.left_child;
    }
    
    pub fn getRightChild(self: *const BVHNode) u16 {
        std.debug.assert(!self.isLeaf());
        return self.right_child;
    }
    
    pub fn setAsLeaf(self: *BVHNode, first_brush: u16, brush_count: u16) void {
        self.left_child = 0;
        self.right_child = brush_count;
        self.first_brush = first_brush;
        self._padding = 0;
    }
    
    pub fn setAsInternal(self: *BVHNode, left_child: u16, right_child: u16) void {
        self.left_child = left_child;
        self.right_child = right_child;
        self.first_brush = 0;
        self._padding = 0;
    }
    
    pub fn getAABB(self: *const BVHNode) AABB {
        return AABB{
            .min = Vec3.new(self.min_x, self.min_y, self.min_z),
            .max = Vec3.new(self.max_x, self.max_y, self.max_z),
        };
    }
    
    pub fn setAABB(self: *BVHNode, aabb: AABB) void {
        self.min_x = aabb.min.data[0];
        self.min_y = aabb.min.data[1];
        self.min_z = aabb.min.data[2];
        self.max_x = aabb.max.data[0];
        self.max_y = aabb.max.data[1];
        self.max_z = aabb.max.data[2];
    }
    
    pub fn intersects(self: *const BVHNode, aabb: AABB) bool {
        return (self.min_x <= aabb.max.data[0] and self.max_x >= aabb.min.data[0] and
                self.min_y <= aabb.max.data[1] and self.max_y >= aabb.min.data[1] and
                self.min_z <= aabb.max.data[2] and self.max_z >= aabb.min.data[2]);
    }
};

// Ultra-compact BVH for brush collision
pub const BVH = struct {
    // Linear array of nodes for cache efficiency
    nodes: []BVHNode,
    // Brush indices for leaves
    brush_indices: []u16,
    // Reference to original brush array
    brushes: []const Brush,
    
    allocator: std.mem.Allocator,
    
    const MAX_LEAF_SIZE = 4; // Optimal for most cases
    const MAX_DEPTH = 32;    // Prevent stack overflow
    
    pub fn init(allocator: std.mem.Allocator, brushes: []const Brush) !BVH {
        if (brushes.len == 0) {
            return BVH{
                .nodes = &[_]BVHNode{},
                .brush_indices = &[_]u16{},
                .brushes = brushes,
                .allocator = allocator,
            };
        }
        
        // Estimate node count (worst case: 2*n-1 nodes)
        const max_nodes = @max(1, brushes.len * 2);
        var nodes = try allocator.alloc(BVHNode, max_nodes);
        var brush_indices = try allocator.alloc(u16, brushes.len);
        
        // Initialize brush indices
        for (0..brushes.len) |i| {
            brush_indices[i] = @intCast(i);
        }
        
        var builder = BVHBuilder{
            .nodes = nodes,
            .brush_indices = brush_indices,
            .brushes = brushes,
            .node_count = 0,
        };
        
        _ = try builder.buildRecursive(0, @intCast(brushes.len), 0);
        
        // Shrink to actual size
        nodes = try allocator.realloc(nodes, builder.node_count);
        
        return BVH{
            .nodes = nodes,
            .brush_indices = brush_indices,
            .brushes = brushes,
            .allocator = allocator,
        };
    }
    
    pub fn deinit(self: *BVH) void {
        if (self.nodes.len > 0) {
            self.allocator.free(self.nodes);
        }
        if (self.brush_indices.len > 0) {
            self.allocator.free(self.brush_indices);
        }
    }
    
    // Fast collision test - returns true if ANY brush intersects
    pub fn testCollision(self: *const BVH, query_aabb: AABB) bool {
        if (self.nodes.len == 0) return false;
        
        // Use a small stack for traversal - most trees are shallow
        var stack: [MAX_DEPTH]u16 = undefined;
        var stack_size: u8 = 1;
        stack[0] = 0; // Root node
        
        while (stack_size > 0) {
            stack_size -= 1;
            const node_idx = stack[stack_size];
            const node = &self.nodes[node_idx];
            
            // Early rejection test
            if (!node.intersects(query_aabb)) continue;
            
            if (node.isLeaf()) {
                // Test brushes in this leaf
                const first = node.getFirstBrush();
                const count = node.getBrushCount();
                
                for (0..count) |i| {
                    const brush_idx = self.brush_indices[first + i];
                    const brush = &self.brushes[brush_idx];
                    
                    // Quick AABB test first
                    if (query_aabb.intersects(brush.bounds)) {
                        // Detailed brush test
                        if (collision.testAABBBrush(query_aabb, brush.*)) {
                            return true;
                        }
                    }
                }
            } else {
                // Add children to stack (if there's room)
                if (stack_size + 2 <= MAX_DEPTH) {
                    stack[stack_size] = node.getLeftChild();
                    stack[stack_size + 1] = node.getRightChild();
                    stack_size += 2;
                }
            }
        }
        
        return false;
    }
    
    // Collect all intersecting brush indices
    pub fn queryBrushes(self: *const BVH, query_aabb: AABB, results: *std.ArrayList(u16)) !void {
        results.clearRetainingCapacity();
        if (self.nodes.len == 0) return;
        
        var stack: [MAX_DEPTH]u16 = undefined;
        var stack_size: u8 = 1;
        stack[0] = 0;
        
        while (stack_size > 0) {
            stack_size -= 1;
            const node_idx = stack[stack_size];
            const node = &self.nodes[node_idx];
            
            if (!node.intersects(query_aabb)) continue;
            
            if (node.isLeaf()) {
                const first = node.getFirstBrush();
                const count = node.getBrushCount();
                
                for (0..count) |i| {
                    const brush_idx = self.brush_indices[first + i];
                    const brush = &self.brushes[brush_idx];
                    
                    if (query_aabb.intersects(brush.bounds)) {
                        if (collision.testAABBBrush(query_aabb, brush.*)) {
                            try results.append(brush_idx);
                        }
                    }
                }
            } else {
                if (stack_size + 2 <= MAX_DEPTH) {
                    stack[stack_size] = node.getLeftChild();
                    stack[stack_size + 1] = node.getRightChild();
                    stack_size += 2;
                }
            }
        }
    }
    
    // Get memory usage statistics
    pub fn getMemoryUsage(self: *const BVH) struct { nodes_bytes: usize, indices_bytes: usize, total_bytes: usize } {
        const nodes_bytes = self.nodes.len * @sizeOf(BVHNode);
        const indices_bytes = self.brush_indices.len * @sizeOf(u16);
        return .{
            .nodes_bytes = nodes_bytes,
            .indices_bytes = indices_bytes,
            .total_bytes = nodes_bytes + indices_bytes,
        };
    }
};

// Builder helper struct
const BVHBuilder = struct {
    nodes: []BVHNode,
    brush_indices: []u16,
    brushes: []const Brush,
    node_count: usize,
    
    const SplitInfo = struct { axis: u8, pos: f32 };
    
    fn buildRecursive(self: *BVHBuilder, first: u16, count: u16, depth: u8) !u16 {
        const node_idx = @as(u16, @intCast(self.node_count));
        self.node_count += 1;
        
        // Calculate bounding box for this range
        var bbox = self.brushes[self.brush_indices[first]].bounds;
        for (1..count) |i| {
            const brush_idx = self.brush_indices[first + i];
            bbox = bbox.union_with(self.brushes[brush_idx].bounds);
        }
        
        var node = &self.nodes[node_idx];
        node.setAABB(bbox);
        
        // Create leaf if small enough or too deep
        if (count <= BVH.MAX_LEAF_SIZE or depth >= BVH.MAX_DEPTH) {
            node.setAsLeaf(first, count);
            return node_idx;
        }
        
        // Find best split
        const split_info = self.findBestSplit(first, count, bbox);
        
        // Partition brushes around split
        const left_count = self.partition(first, count, split_info);
        
        // Ensure we don't create empty partitions
        const actual_left_count = if (left_count == 0) 1 else if (left_count == count) count - 1 else left_count;
        
        // Build children
        const left_child = try self.buildRecursive(first, actual_left_count, depth + 1);
        const right_child = try self.buildRecursive(first + actual_left_count, count - actual_left_count, depth + 1);
        
        // Set as internal node
        node.setAsInternal(left_child, right_child);
        
        return node_idx;
    }
    
    fn findBestSplit(self: *BVHBuilder, first: u16, count: u16, bbox: AABB) SplitInfo {
        _ = self;
        _ = first;
        _ = count;
        
        const size = Vec3.sub(bbox.max, bbox.min);
        
        // Choose longest axis for splitting
        const axis: u8 = if (size.data[0] > size.data[1] and size.data[0] > size.data[2]) 0
                        else if (size.data[1] > size.data[2]) 1 else 2;
        
        // Simple midpoint split for now (could be improved with SAH)
        const pos = bbox.min.data[axis] + size.data[axis] * 0.5;
        
        return SplitInfo{ .axis = axis, .pos = pos };
    }
    
    fn partition(self: *BVHBuilder, first: u16, count: u16, split: SplitInfo) u16 {
        var left: u16 = 0;
        var right: u16 = count;
        
        while (left < right) {
            const brush_idx = self.brush_indices[first + left];
            const brush = &self.brushes[brush_idx];
            
            // Use centroid for partitioning
            const centroid = Vec3.scale(Vec3.add(brush.bounds.min, brush.bounds.max), 0.5);
            
            if (centroid.data[split.axis] < split.pos) {
                left += 1;
            } else {
                right -= 1;
                std.mem.swap(u16, &self.brush_indices[first + left], &self.brush_indices[first + right]);
            }
        }
        
        return left;
    }
};