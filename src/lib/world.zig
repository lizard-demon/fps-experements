// Brush-based world collision system
const std = @import("std");
const math = @import("math.zig");
const brush_mod = @import("brush.zig");
const bvh = @import("bvh.zig");

const Vec3 = math.Vec3;
const Brush = brush_mod.Brush;
const AABB = brush_mod.AABB;
const BvhAABB = bvh.AABB;
const TraceResult = brush_mod.TraceResult;
const Plane = brush_mod.Plane;

pub const BrushWorld = struct {
    brushes: std.ArrayListUnmanaged(Brush) = .{},
    bvh_tree: ?bvh.BVH(Brush, getBrushAABB) = null,
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) BrushWorld {
        return .{ .allocator = allocator };
    }
    
    pub fn deinit(self: *BrushWorld) void {
        for (self.brushes.items) |*b| {
            b.deinit(self.allocator);
        }
        self.brushes.deinit(self.allocator);
        if (self.bvh_tree) |*tree| tree.deinit();
    }
    
    pub fn addBrush(self: *BrushWorld, brush: Brush) !void {
        try self.brushes.append(self.allocator, brush);
    }
    
    pub fn addBox(self: *BrushWorld, center: Vec3, size: Vec3) !void {
        const half = Vec3.scale(size, 0.5);
        const min = Vec3.sub(center, half);
        const max = Vec3.add(center, half);
        const brush = try Brush.createBox(self.allocator, min, max);
        try self.addBrush(brush);
    }
    
    pub fn build(self: *BrushWorld) !void {
        if (self.bvh_tree) |*tree| tree.deinit();
        if (self.brushes.items.len > 0) {
            self.bvh_tree = try bvh.BVH(Brush, getBrushAABB).init(self.allocator, self.brushes.items);
        }
    }
    
    pub fn testCollision(self: *const BrushWorld, aabb: AABB) bool {
        for (self.brushes.items) |brush| {
            if (brush_mod.testAABBBrush(aabb, brush)) return true;
        }
        return false;
    }
    
    // Simple movement - test each axis separately
    pub fn movePlayer(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) Vec3 {
        var pos = start;
        
        // Move X axis
        if (@abs(delta.data[0]) > 0.001) {
            const test_pos = Vec3.new(pos.data[0] + delta.data[0], pos.data[1], pos.data[2]);
            const test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            if (!self.testCollision(test_aabb)) {
                pos.data[0] = test_pos.data[0];
            }
        }
        
        // Move Y axis
        if (@abs(delta.data[1]) > 0.001) {
            const test_pos = Vec3.new(pos.data[0], pos.data[1] + delta.data[1], pos.data[2]);
            const test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            if (!self.testCollision(test_aabb)) {
                pos.data[1] = test_pos.data[1];
            }
        }
        
        // Move Z axis
        if (@abs(delta.data[2]) > 0.001) {
            const test_pos = Vec3.new(pos.data[0], pos.data[1], pos.data[2] + delta.data[2]);
            const test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            if (!self.testCollision(test_aabb)) {
                pos.data[2] = test_pos.data[2];
            }
        }
        
        return pos;
    }
};

fn getBrushAABB(brush: Brush) BvhAABB {
    return BvhAABB{ .min = brush.bounds.min, .max = brush.bounds.max };
}