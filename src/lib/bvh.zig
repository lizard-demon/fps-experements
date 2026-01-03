const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;

pub const AABB = math.AABB;

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
            
            if (count <= 4) { // Larger leaves for better performance
                return self.nodes.append(self.mem, .{ .box = box, .first = first, .count = count });
            }
            
            // Full SAH: try all axes and all split positions
            var best_cost: f32 = std.math.floatMax(f32);
            var best_axis: usize = 0;
            var best_split: u32 = count / 2;
            
            for (0..3) |axis| {
                // Sort by centroid on this axis
                self.sortByAxis(first, count, axis);
                
                // Try every split position
                for (1..count) |split_idx| {
                    const cost = self.sahCost(first, count, @intCast(split_idx));
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_axis = axis;
                        best_split = @intCast(split_idx);
                    }
                }
            }
            
            // Apply best split
            self.sortByAxis(first, count, best_axis);
            
            const node_idx = @as(u32, @intCast(self.nodes.items.len));
            try self.nodes.append(self.mem, .{ .box = box });
            
            self.nodes.items[node_idx].left = @intCast(self.nodes.items.len);
            try self.split(first, best_split);
            
            self.nodes.items[node_idx].right = @intCast(self.nodes.items.len);
            try self.split(first + best_split, count - best_split);
        }
        
        fn sortByAxis(self: *Self, first: u32, count: u32, axis: usize) void {
            const slice = self.indices.items[first..first + count];
            const Context = struct {
                items: []T,
                axis: usize,
                
                fn lessThan(ctx: @This(), a: u32, b: u32) bool {
                    const a_box = get_aabb(ctx.items[a]);
                    const b_box = get_aabb(ctx.items[b]);
                    const a_center = Vec3.scale(Vec3.add(a_box.min, a_box.max), 0.5);
                    const b_center = Vec3.scale(Vec3.add(b_box.min, b_box.max), 0.5);
                    return a_center.data[ctx.axis] < b_center.data[ctx.axis];
                }
            };
            
            const context = Context{ .items = self.items, .axis = axis };
            std.mem.sort(u32, slice, context, Context.lessThan);
        }
        
        fn sahCost(self: *Self, first: u32, count: u32, split_pos: u32) f32 {
            // Calculate bounding boxes for left and right sides
            var left_box = get_aabb(self.items[self.indices.items[first]]);
            for (1..split_pos) |i| {
                left_box = left_box.union_with(get_aabb(self.items[self.indices.items[first + i]]));
            }
            
            var right_box = get_aabb(self.items[self.indices.items[first + split_pos]]);
            for (split_pos + 1..count) |i| {
                right_box = right_box.union_with(get_aabb(self.items[self.indices.items[first + i]]));
            }
            
            // SAH cost = surface_area_left * count_left + surface_area_right * count_right
            const left_area = left_box.surface_area();
            const right_area = right_box.surface_area();
            const left_count = @as(f32, @floatFromInt(split_pos));
            const right_count = @as(f32, @floatFromInt(count - split_pos));
            
            return left_area * left_count + right_area * right_count;
        }
        
        pub fn query(self: *const Self, query_aabb: AABB) bool {
            return self.intersect(query_aabb);
        }
        
        pub fn intersect(self: *const Self, query_aabb: AABB) bool {
            if (self.nodes.items.len == 0) return false;
            
            var stack: [64]u32 = undefined;
            var sp: usize = 1;
            stack[0] = 0;
            
            while (sp > 0) {
                sp -= 1;
                const node = self.nodes.items[stack[sp]];
                if (!query_aabb.hit(node.box)) continue;
                
                if (node.left == 0) {
                    for (0..node.count) |i| {
                        if (query_aabb.hit(get_aabb(self.items[self.indices.items[node.first + i]]))) {
                            return true;
                        }
                    }
                } else if (sp + 2 <= stack.len) {
                    stack[sp] = node.left;
                    stack[sp + 1] = node.right;
                    sp += 2;
                }
            }
            return false;
        }
    };
}