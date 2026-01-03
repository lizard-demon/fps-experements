// Quake-style brush collision system
const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const COLLISION_EPSILON: f32 = 0.001;
pub const STEP_HEIGHT: f32 = 0.25;
pub const GROUND_SNAP: f32 = 0.05;

pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub fn new(normal: Vec3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
};

pub const Brush = struct {
    planes: []Plane,
    bounds: AABB,
    allocator: std.mem.Allocator,
    
    pub fn createBox(allocator: std.mem.Allocator, min: Vec3, max: Vec3) !Brush {
        var planes = try allocator.alloc(Plane, 6);
        planes[0] = Plane.new(Vec3.new(1, 0, 0), -max.data[0]);
        planes[1] = Plane.new(Vec3.new(-1, 0, 0), min.data[0]);
        planes[2] = Plane.new(Vec3.new(0, 1, 0), -max.data[1]);
        planes[3] = Plane.new(Vec3.new(0, -1, 0), min.data[1]);
        planes[4] = Plane.new(Vec3.new(0, 0, 1), -max.data[2]);
        planes[5] = Plane.new(Vec3.new(0, 0, -1), min.data[2]);
        return .{ .planes = planes, .bounds = AABB{ .min = min, .max = max }, .allocator = allocator };
    }
    
    pub fn init(allocator: std.mem.Allocator, planes: []const Plane) !Brush {
        const brush_planes = try allocator.dupe(Plane, planes);
        const bounds = calculateBounds(brush_planes);
        return .{ .planes = brush_planes, .bounds = bounds, .allocator = allocator };
    }
    
    pub fn deinit(self: *Brush) void {
        self.allocator.free(self.planes);
    }
    
    pub fn intersectsAABB(self: Brush, aabb: AABB) bool {
        if (!self.bounds.intersects(aabb)) return false;
        
        for (self.planes) |plane| {
            const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
            const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            if (plane.distanceToPoint(center) > radius + COLLISION_EPSILON) return false;
        }
        return true;
    }
};

fn calculateBounds(planes: []const Plane) AABB {
    var min = Vec3.new(-100, -100, -100);
    var max = Vec3.new(100, 100, 100);
    
    for (planes) |plane| {
        const abs_normal = Vec3.new(@abs(plane.normal.data[0]), @abs(plane.normal.data[1]), @abs(plane.normal.data[2]));
        const max_component = @max(@max(abs_normal.data[0], abs_normal.data[1]), abs_normal.data[2]);
        
        if (max_component > 0.9) {
            if (abs_normal.data[0] == max_component) {
                if (plane.normal.data[0] > 0) {
                    max.data[0] = @min(max.data[0], -plane.distance);
                } else {
                    min.data[0] = @max(min.data[0], plane.distance);
                }
            } else if (abs_normal.data[1] == max_component) {
                if (plane.normal.data[1] > 0) {
                    max.data[1] = @min(max.data[1], -plane.distance);
                } else {
                    min.data[1] = @max(min.data[1], plane.distance);
                }
            } else if (abs_normal.data[2] == max_component) {
                if (plane.normal.data[2] > 0) {
                    max.data[2] = @min(max.data[2], -plane.distance);
                } else {
                    min.data[2] = @max(min.data[2], plane.distance);
                }
            }
        }
    }
    
    if (min.data[0] > max.data[0]) { min.data[0] = -1; max.data[0] = 1; }
    if (min.data[1] > max.data[1]) { min.data[1] = -1; max.data[1] = 1; }
    if (min.data[2] > max.data[2]) { min.data[2] = -1; max.data[2] = 1; }
    
    return AABB{ .min = min, .max = max };
}

pub const MoveResult = struct {
    position: Vec3,
    velocity_adjustment: Vec3,
    on_ground: bool,
    hit_ceiling: bool,
    hit_wall: bool,
};

pub const BrushWorld = struct {
    brushes: std.ArrayListUnmanaged(Brush) = .{},
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) BrushWorld {
        return .{ .allocator = allocator };
    }
    
    pub fn deinit(self: *BrushWorld) void {
        for (self.brushes.items) |*b| b.deinit();
        self.brushes.deinit(self.allocator);
    }
    
    pub fn addBrush(self: *BrushWorld, brush: Brush) !void {
        try self.brushes.append(self.allocator, brush);
    }
    
    pub fn addBox(self: *BrushWorld, center: Vec3, size: Vec3) !void {
        const half = Vec3.scale(size, 0.5);
        const brush = try Brush.createBox(self.allocator, Vec3.sub(center, half), Vec3.add(center, half));
        try self.brushes.append(self.allocator, brush);
    }
    
    pub fn addBoxAndReturn(self: *BrushWorld, center: Vec3, size: Vec3) !Brush {
        const half = Vec3.scale(size, 0.5);
        const brush = try Brush.createBox(self.allocator, Vec3.sub(center, half), Vec3.add(center, half));
        try self.brushes.append(self.allocator, brush);
        return brush;
    }
    
    pub fn build(self: *BrushWorld) !void {
        _ = self;
    }
    
    pub fn testCollision(self: *const BrushWorld, aabb: AABB) bool {
        for (self.brushes.items) |brush| {
            if (brush.intersectsAABB(aabb)) return true;
        }
        return false;
    }
    
    pub fn movePlayer(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var pos = start;
        var vel_adj = Vec3.zero();
        var hit_wall = false;
        var hit_ceiling = false;
        
        // Horizontal movement with step-up
        const h_delta = Vec3.new(delta.data[0], 0, delta.data[2]);
        if (Vec3.length(h_delta) > COLLISION_EPSILON) {
            var test_pos = Vec3.add(pos, h_delta);
            var test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
            
            if (self.testCollision(test_aabb)) {
                // Try step-up
                test_pos.data[1] += STEP_HEIGHT;
                test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
                
                if (!self.testCollision(test_aabb)) {
                    // Find ground
                    var step_down: f32 = GROUND_SNAP;
                    while (step_down <= STEP_HEIGHT + GROUND_SNAP) {
                        const ground_pos = Vec3.new(test_pos.data[0], test_pos.data[1] - step_down, test_pos.data[2]);
                        const ground_aabb = AABB{ .min = Vec3.add(ground_pos, player_aabb.min), .max = Vec3.add(ground_pos, player_aabb.max) };
                        if (self.testCollision(ground_aabb)) {
                            pos = Vec3.new(test_pos.data[0], test_pos.data[1] - (step_down - GROUND_SNAP), test_pos.data[2]);
                            break;
                        }
                        step_down += GROUND_SNAP;
                    } else pos = test_pos;
                } else {
                    // Try sliding
                    test_pos = Vec3.new(start.data[0] + h_delta.data[0], start.data[1], start.data[2]);
                    test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
                    if (!self.testCollision(test_aabb)) {
                        pos.data[0] = test_pos.data[0];
                    } else {
                        vel_adj.data[0] = -h_delta.data[0];
                        hit_wall = true;
                    }
                    
                    test_pos = Vec3.new(pos.data[0], start.data[1], start.data[2] + h_delta.data[2]);
                    test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
                    if (!self.testCollision(test_aabb)) {
                        pos.data[2] = test_pos.data[2];
                    } else {
                        vel_adj.data[2] = -h_delta.data[2];
                        hit_wall = true;
                    }
                }
            } else {
                pos.data[0] = test_pos.data[0];
                pos.data[2] = test_pos.data[2];
            }
        }
        
        // Vertical movement
        if (@abs(delta.data[1]) > COLLISION_EPSILON) {
            const test_pos = Vec3.new(pos.data[0], pos.data[1] + delta.data[1], pos.data[2]);
            const test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
            if (!self.testCollision(test_aabb)) {
                pos.data[1] = test_pos.data[1];
            } else {
                vel_adj.data[1] = -delta.data[1];
                if (delta.data[1] > 0) hit_ceiling = true;
            }
        }
        
        // Ground check
        const ground_test = Vec3.new(pos.data[0], pos.data[1] - GROUND_SNAP, pos.data[2]);
        const ground_aabb = AABB{ .min = Vec3.add(ground_test, player_aabb.min), .max = Vec3.add(ground_test, player_aabb.max) };
        const on_ground = self.testCollision(ground_aabb);
        
        return MoveResult{
            .position = pos,
            .velocity_adjustment = vel_adj,
            .on_ground = on_ground,
            .hit_ceiling = hit_ceiling,
            .hit_wall = hit_wall,
        };
    }
};