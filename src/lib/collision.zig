// Quake-style brush collision system
const std = @import("std");
const math = @import("math.zig");
const bvh = @import("bvh.zig");

const Vec3 = math.Vec3;

pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub const COLLISION_EPSILON = 0.001;
    
    pub fn new(normal: Vec3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
    
    pub fn isPointInFront(self: Plane, point: Vec3) bool {
        return self.distanceToPoint(point) > COLLISION_EPSILON;
    }
};

pub const AABB = struct {
    min: Vec3,
    max: Vec3,
    
    pub fn new(min: Vec3, max: Vec3) AABB {
        return .{ .min = min, .max = max };
    }
    
    pub fn fromCenterSize(center: Vec3, size: Vec3) AABB {
        const half = Vec3.scale(size, 0.5);
        return .{ .min = Vec3.sub(center, half), .max = Vec3.add(center, half) };
    }
    
    pub fn intersects(self: AABB, other: AABB) bool {
        return (self.min.data[0] <= other.max.data[0] and self.max.data[0] >= other.min.data[0] and
                self.min.data[1] <= other.max.data[1] and self.max.data[1] >= other.min.data[1] and
                self.min.data[2] <= other.max.data[2] and self.max.data[2] >= other.min.data[2]);
    }
    
    pub fn union_with(self: AABB, other: AABB) AABB {
        return .{ .min = Vec3.min(self.min, other.min), .max = Vec3.max(self.max, other.max) };
    }
    
    pub fn surface_area(self: AABB) f32 {
        const d = Vec3.sub(self.max, self.min);
        return 2.0 * (d.data[0] * d.data[1] + d.data[1] * d.data[2] + d.data[2] * d.data[0]);
    }
};

pub const Brush = struct {
    planes: []Plane,
    bounds: AABB,
    
    pub fn init(allocator: std.mem.Allocator, planes: []const Plane) !Brush {
        const brush_planes = try allocator.dupe(Plane, planes);
        const bounds = calculateBounds(brush_planes);
        return .{ .planes = brush_planes, .bounds = bounds };
    }
    
    pub fn deinit(self: *Brush, allocator: std.mem.Allocator) void {
        allocator.free(self.planes);
    }
    
    pub fn createBox(allocator: std.mem.Allocator, min: Vec3, max: Vec3) !Brush {
        var planes = try allocator.alloc(Plane, 6);
        
        // Box planes with normals pointing outward from solid
        planes[0] = Plane.new(Vec3.new(1, 0, 0), -max.data[0]);   // Right
        planes[1] = Plane.new(Vec3.new(-1, 0, 0), min.data[0]);   // Left
        planes[2] = Plane.new(Vec3.new(0, 1, 0), -max.data[1]);   // Top
        planes[3] = Plane.new(Vec3.new(0, -1, 0), min.data[1]);   // Bottom
        planes[4] = Plane.new(Vec3.new(0, 0, 1), -max.data[2]);   // Front
        planes[5] = Plane.new(Vec3.new(0, 0, -1), min.data[2]);   // Back
        
        return .{ .planes = planes, .bounds = AABB{ .min = min, .max = max } };
    }
    
    pub fn isPointInside(self: Brush, point: Vec3) bool {
        for (self.planes) |plane| {
            if (plane.distanceToPoint(point) > Plane.COLLISION_EPSILON) return false;
        }
        return true;
    }
    
    pub fn intersectsAABB(self: Brush, aabb: AABB) bool {
        if (!self.bounds.intersects(aabb)) return false;
        
        for (self.planes) |plane| {
            const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
            const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
            
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            
            if (plane.distanceToPoint(center) > radius + Plane.COLLISION_EPSILON) return false;
        }
        return true;
    }
};

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
    
    pub fn addBoxAndReturn(self: *BrushWorld, center: Vec3, size: Vec3) !Brush {
        const half = Vec3.scale(size, 0.5);
        const min = Vec3.sub(center, half);
        const max = Vec3.add(center, half);
        const brush = try Brush.createBox(self.allocator, min, max);
        try self.addBrush(brush);
        return brush;
    }
    
    pub fn build(self: *BrushWorld) !void {
        if (self.bvh_tree) |*tree| tree.deinit();
        if (self.brushes.items.len > 0) {
            self.bvh_tree = try bvh.BVH(Brush, getBrushAABB).init(self.allocator, self.brushes.items);
        }
    }
    
    pub fn testCollision(self: *const BrushWorld, aabb: AABB) bool {
        for (self.brushes.items) |brush| {
            if (testAABBBrush(aabb, brush)) return true;
        }
        return false;
    }
    
    // Player movement with step-up mechanics
    pub fn movePlayer(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) Vec3 {
        var pos = start;
        const step_height = 0.25;
        
        // Horizontal movement (X and Z) with step-up
        const horizontal_delta = Vec3.new(delta.data[0], 0, delta.data[2]);
        if (Vec3.length(horizontal_delta) > Plane.COLLISION_EPSILON) {
            var test_pos = Vec3.add(pos, horizontal_delta);
            var test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            
            if (self.testCollision(test_aabb)) {
                // Try stepping up
                test_pos.data[1] += step_height;
                test_aabb = AABB{
                    .min = Vec3.add(test_pos, player_aabb.min),
                    .max = Vec3.add(test_pos, player_aabb.max)
                };
                
                if (!self.testCollision(test_aabb)) {
                    // Step down to find ground
                    var step_down: f32 = 0.05;
                    while (step_down <= step_height + 0.1) {
                        const step_down_pos = Vec3.new(test_pos.data[0], test_pos.data[1] - step_down, test_pos.data[2]);
                        const step_down_aabb = AABB{
                            .min = Vec3.add(step_down_pos, player_aabb.min),
                            .max = Vec3.add(step_down_pos, player_aabb.max)
                        };
                        
                        if (self.testCollision(step_down_aabb)) {
                            pos = Vec3.new(test_pos.data[0], test_pos.data[1] - (step_down - 0.05), test_pos.data[2]);
                            break;
                        }
                        step_down += 0.05;
                    } else {
                        pos = test_pos;
                    }
                }
            } else {
                pos.data[0] = test_pos.data[0];
                pos.data[2] = test_pos.data[2];
            }
        }
        
        // Vertical movement (Y)
        if (@abs(delta.data[1]) > Plane.COLLISION_EPSILON) {
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

// Helper functions
fn getBrushAABB(brush: Brush) bvh.AABB {
    return bvh.AABB{ .min = brush.bounds.min, .max = brush.bounds.max };
}

fn calculateBounds(planes: []const Plane) AABB {
    var min = Vec3.new(-50, -50, -50);
    var max = Vec3.new(50, 50, 50);
    
    // Calculate bounds from axis-aligned planes
    for (planes) |plane| {
        const abs_normal = Vec3.new(@abs(plane.normal.data[0]), @abs(plane.normal.data[1]), @abs(plane.normal.data[2]));
        const max_component = @max(@max(abs_normal.data[0], abs_normal.data[1]), abs_normal.data[2]);
        
        if (abs_normal.data[0] == max_component and abs_normal.data[0] > 0.9) {
            if (plane.normal.data[0] > 0) {
                max.data[0] = @min(max.data[0], -plane.distance);
            } else {
                min.data[0] = @max(min.data[0], plane.distance);
            }
        } else if (abs_normal.data[1] == max_component and abs_normal.data[1] > 0.9) {
            if (plane.normal.data[1] > 0) {
                max.data[1] = @min(max.data[1], -plane.distance);
            } else {
                min.data[1] = @max(min.data[1], plane.distance);
            }
        } else if (abs_normal.data[2] == max_component and abs_normal.data[2] > 0.9) {
            if (plane.normal.data[2] > 0) {
                max.data[2] = @min(max.data[2], -plane.distance);
            } else {
                min.data[2] = @max(min.data[2], plane.distance);
            }
        }
    }
    
    // Ensure valid bounds
    if (min.data[0] > max.data[0]) { min.data[0] = 0; max.data[0] = 0; }
    if (min.data[1] > max.data[1]) { min.data[1] = 0; max.data[1] = 0; }
    if (min.data[2] > max.data[2]) { min.data[2] = 0; max.data[2] = 0; }
    
    return AABB{ .min = min, .max = max };
}

pub fn testAABBBrush(aabb: AABB, brush: Brush) bool {
    if (!aabb.intersects(brush.bounds)) return false;
    
    for (brush.planes) |plane| {
        const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
        const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
        
        const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                      @abs(plane.normal.data[1] * extents.data[1]) +
                      @abs(plane.normal.data[2] * extents.data[2]);
        
        if (plane.distanceToPoint(center) > radius + Plane.COLLISION_EPSILON) return false;
    }
    return true;
}