const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;

pub const AABB = struct { 
    min: Vec3, max: Vec3,
    
    pub fn hit(a: AABB, b: AABB) bool { 
        return (a.min.data[0] <= b.max.data[0] and a.max.data[0] >= b.min.data[0] and
                a.min.data[1] <= b.max.data[1] and a.max.data[1] >= b.min.data[1] and
                a.min.data[2] <= b.max.data[2] and a.max.data[2] >= b.min.data[2]); 
    }
    
    pub fn union_with(a: AABB, b: AABB) AABB {
        return .{ .min = Vec3.min(a.min, b.min), .max = Vec3.max(a.max, b.max) };
    }
    
    pub fn surface_area(self: AABB) f32 {
        const d = Vec3.sub(self.max, self.min);
        return 2.0 * (d.data[0] * d.data[1] + d.data[1] * d.data[2] + d.data[2] * d.data[0]);
    }
};

const Node = struct { 
    box: AABB, 
    left: u32 = 0, 
    right: u32 = 0, 
    first: u32 = 0, 
    count: u32 = 0 
};

pub fn BVH(comptime T: type, comptime get_aabb: fn(T) AABB) type {
    return struct {
        const Self = @This();
        
        items: []T,
        nodes: std.ArrayListUnmanaged(Node) = .{},
        indices: std.ArrayListUnmanaged(u32) = .{},
        mem: std.mem.Allocator,
        
        pub fn init(mem: std.mem.Allocator, items: []T) !Self {
            var self = Self{ .items = items, .mem = mem };
            try self.build();
            return self;
        }
        
        pub fn deinit(self: *Self) void {
            self.nodes.deinit(self.mem);
            self.indices.deinit(self.mem);
        }
        
        fn build(self: *Self) !void {
            self.nodes.clearRetainingCapacity();
            self.indices.clearRetainingCapacity();
            
            for (0..self.items.len) |i| try self.indices.append(self.mem, @intCast(i));
            if (self.items.len > 0) try self.split(0, @intCast(self.items.len));
        }
        
        fn split(self: *Self, first: u32, count: u32) !void {
            var box = get_aabb(self.items[self.indices.items[first]]);
            for (1..count) |i| {
                box = box.union_with(get_aabb(self.items[self.indices.items[first + i]]));
            }
            
            if (count <= 2) {
                return self.nodes.append(self.mem, .{ .box = box, .first = first, .count = count });
            }
            
            // Simple longest axis splitting
            const size = Vec3.sub(box.max, box.min);
            const axis: usize = if (size.data[0] > size.data[1] and size.data[0] > size.data[2]) 0 
                               else if (size.data[1] > size.data[2]) 1 else 2;
            const mid = box.min.data[axis] + size.data[axis] * 0.5;
            
            // Partition around midpoint
            var i: u32 = 0; 
            var j: u32 = count;
            while (i < j) {
                const idx = self.indices.items[first + i];
                const centroid = Vec3.scale(Vec3.add(get_aabb(self.items[idx]).min, get_aabb(self.items[idx]).max), 0.5);
                if (centroid.data[axis] < mid) {
                    i += 1;
                } else {
                    j -= 1;
                    std.mem.swap(u32, &self.indices.items[first + i], &self.indices.items[first + j]);
                }
            }
            
            const left_count = if (i == 0 or i == count) count / 2 else i;
            const node_idx = @as(u32, @intCast(self.nodes.items.len));
            try self.nodes.append(self.mem, .{ .box = box });
            
            self.nodes.items[node_idx].left = @intCast(self.nodes.items.len);
            try self.split(first, left_count);
            
            self.nodes.items[node_idx].right = @intCast(self.nodes.items.len);
            try self.split(first + left_count, count - left_count);
        }
        
        pub fn intersect(self: *const Self, query: AABB) bool {
            if (self.nodes.items.len == 0) return false;
            
            // Use a reasonable stack size and handle overflow gracefully
            var stack: [128]u32 = undefined;
            var sp: usize = 1;
            stack[0] = 0;
            
            while (sp > 0) {
                sp -= 1;
                const node = self.nodes.items[stack[sp]];
                if (!query.hit(node.box)) continue;
                
                if (node.left == 0) {
                    // Leaf node - check primitives
                    for (0..node.count) |i| {
                        if (query.hit(get_aabb(self.items[self.indices.items[node.first + i]]))) {
                            return true;
                        }
                    }
                } else {
                    // Internal node - add children to stack if there's room
                    if (sp + 2 <= stack.len) {
                        stack[sp] = node.left;
                        stack[sp + 1] = node.right;
                        sp += 2;
                    } else {
                        // Stack overflow - fall back to direct checking of remaining nodes
                        // This is rare but ensures we don't miss collisions
                        if (self.intersectDirect(query, node.left)) return true;
                        if (self.intersectDirect(query, node.right)) return true;
                    }
                }
            }
            return false;
        }
        
        // Fallback direct intersection check for stack overflow cases
        fn intersectDirect(self: *const Self, query: AABB, node_idx: u32) bool {
            if (node_idx >= self.nodes.items.len) return false;
            
            const node = self.nodes.items[node_idx];
            if (!query.hit(node.box)) return false;
            
            if (node.left == 0) {
                // Leaf node
                for (0..node.count) |i| {
                    if (query.hit(get_aabb(self.items[self.indices.items[node.first + i]]))) {
                        return true;
                    }
                }
                return false;
            } else {
                // Internal node
                return self.intersectDirect(query, node.left) or self.intersectDirect(query, node.right);
            }
        }
    };
}