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
            if (brush_mod.testAABBBrush(aabb, brush)) {
                return true;
            }
        }
        return false;
    }
    
    // Simple and robust movement with basic step-up for stairs
    pub fn movePlayer(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) Vec3 {
        var pos = start;
        const step_height = 0.25; // Maximum step height for stairs
        
        // Try to move horizontally first (X and Z together)
        const horizontal_delta = Vec3.new(delta.data[0], 0, delta.data[2]);
        if (Vec3.length(horizontal_delta) > brush_mod.Plane.COLLISION_EPSILON) {
            var test_pos = Vec3.add(pos, horizontal_delta);
            var test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            
            if (self.testCollision(test_aabb)) {
                // Try stepping up for stairs/small obstacles
                test_pos.data[1] += step_height;
                test_aabb = AABB{
                    .min = Vec3.add(test_pos, player_aabb.min),
                    .max = Vec3.add(test_pos, player_aabb.max)
                };
                
                if (!self.testCollision(test_aabb)) {
                    // We can step up, now try to step back down to find the ground
                    var step_down_pos = test_pos;
                    var found_ground = false;
                    
                    // Step down in small increments to find the ground
                    var step_down: f32 = 0.05;
                    while (step_down <= step_height + 0.1) {
                        step_down_pos.data[1] = test_pos.data[1] - step_down;
                        const step_down_aabb = AABB{
                            .min = Vec3.add(step_down_pos, player_aabb.min),
                            .max = Vec3.add(step_down_pos, player_aabb.max)
                        };
                        
                        if (self.testCollision(step_down_aabb)) {
                            // Found ground, use previous position
                            step_down_pos.data[1] = test_pos.data[1] - (step_down - 0.05);
                            found_ground = true;
                            break;
                        }
                        step_down += 0.05;
                    }
                    
                    if (found_ground) {
                        pos.data[0] = step_down_pos.data[0];
                        pos.data[1] = step_down_pos.data[1];
                        pos.data[2] = step_down_pos.data[2];
                    } else {
                        // No ground found, just step up
                        pos.data[0] = test_pos.data[0];
                        pos.data[1] = test_pos.data[1];
                        pos.data[2] = test_pos.data[2];
                    }
                }
                // If we can't step up either, don't move horizontally
            } else {
                // No collision, move normally
                pos.data[0] = test_pos.data[0];
                pos.data[2] = test_pos.data[2];
            }
        }
        
        // Move vertically (Y axis)
        if (@abs(delta.data[1]) > brush_mod.Plane.COLLISION_EPSILON) {
            const test_pos = Vec3.new(pos.data[0], pos.data[1] + delta.data[1], pos.data[2]);
            const test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            if (!self.testCollision(test_aabb)) {
                pos.data[1] = test_pos.data[1];
            }
        }
        
        return pos;
    }
};

fn getBrushAABB(brush: Brush) BvhAABB {
    return BvhAABB{ .min = brush.bounds.min, .max = brush.bounds.max };
}