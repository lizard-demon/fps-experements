// Quake-style brush collision system
const std = @import("std");
const math = @import("math.zig");
const bvh = @import("bvh.zig");

const Vec3 = math.Vec3;
const BVH = bvh.BVH;

// Consistent epsilon for all collision calculations
pub const COLLISION_EPSILON: f32 = 0.001;
pub const STEP_HEIGHT: f32 = 0.25;
pub const GROUND_SNAP_DISTANCE: f32 = 0.05;

pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub fn new(normal: Vec3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
    
    pub fn isPointInFront(self: Plane, point: Vec3) bool {
        return self.distanceToPoint(point) > COLLISION_EPSILON;
    }
    
    pub fn isPointBehind(self: Plane, point: Vec3) bool {
        return self.distanceToPoint(point) < -COLLISION_EPSILON;
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
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator, planes: []const Plane) !Brush {
        const brush_planes = try allocator.dupe(Plane, planes);
        const bounds = calculateBounds(brush_planes);
        return .{ 
            .planes = brush_planes, 
            .bounds = bounds,
            .allocator = allocator
        };
    }
    
    pub fn deinit(self: *Brush) void {
        self.allocator.free(self.planes);
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
        
        return .{ 
            .planes = planes, 
            .bounds = AABB{ .min = min, .max = max },
            .allocator = allocator
        };
    }
    
    pub fn isPointInside(self: Brush, point: Vec3) bool {
        for (self.planes) |plane| {
            if (plane.distanceToPoint(point) > COLLISION_EPSILON) return false;
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
            
            if (plane.distanceToPoint(center) > radius + COLLISION_EPSILON) return false;
        }
        return true;
    }
    
    // Get the closest point on the brush surface to a given point
    pub fn getClosestPoint(self: Brush, point: Vec3) Vec3 {
        var closest = point;
        
        for (self.planes) |plane| {
            const dist = plane.distanceToPoint(closest);
            if (dist > COLLISION_EPSILON) {
                // Push point back onto the plane
                closest = Vec3.sub(closest, Vec3.scale(plane.normal, dist));
            }
        }
        
        return closest;
    }
    
    // Check if this brush supports walking (has a reasonable up-facing surface)
    pub fn isWalkable(self: Brush, up_direction: Vec3) bool {
        for (self.planes) |plane| {
            const dot = Vec3.dot(plane.normal, up_direction);
            if (dot < -0.7) { // Surface faces mostly upward
                return true;
            }
        }
        return false;
    }
};

pub const BrushWorld = struct {
    brushes: std.ArrayListUnmanaged(Brush) = .{},
    bvh_tree: ?BVH = null,
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) BrushWorld {
        return .{ .allocator = allocator };
    }
    
    pub fn deinit(self: *BrushWorld) void {
        for (self.brushes.items) |*b| {
            b.deinit();
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
            self.bvh_tree = try BVH.init(self.allocator, self.brushes.items);
        }
    }
    
    // High-performance collision test using ultraminimal BVH
    pub fn testCollision(self: *const BrushWorld, aabb: AABB) bool {
        if (self.bvh_tree) |*tree| {
            return tree.testCollision(aabb);
        } else {
            // Fallback to brute force
            for (self.brushes.items) |brush| {
                if (testAABBBrush(aabb, brush)) return true;
            }
            return false;
        }
    }
    
    // Get all brushes that intersect with the given AABB
    pub fn getIntersectingBrushes(self: *const BrushWorld, aabb: AABB, results: *std.ArrayList(u16)) !void {
        if (self.bvh_tree) |*tree| {
            try tree.queryBrushes(aabb, results);
        } else {
            results.clearRetainingCapacity();
            for (self.brushes.items, 0..) |brush, index| {
                if (testAABBBrush(aabb, brush)) {
                    try results.append(@intCast(index));
                }
            }
        }
    }
    
    // Enhanced player movement with better collision response
    pub fn movePlayer(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{
            .position = start,
            .velocity_adjustment = Vec3.zero(),
            .on_ground = false,
            .hit_ceiling = false,
            .hit_wall = false,
        };
        
        // Split movement into horizontal and vertical components
        const horizontal_delta = Vec3.new(delta.data[0], 0, delta.data[2]);
        const vertical_delta = Vec3.new(0, delta.data[1], 0);
        
        // Handle horizontal movement with step-up
        if (Vec3.length(horizontal_delta) > COLLISION_EPSILON) {
            result = self.moveHorizontalWithStepUp(result.position, horizontal_delta, player_aabb);
        }
        
        // Handle vertical movement
        if (@abs(vertical_delta.data[1]) > COLLISION_EPSILON) {
            const vertical_result = self.moveVertical(result.position, vertical_delta, player_aabb);
            result.position = vertical_result.position;
            result.hit_ceiling = vertical_result.hit_ceiling;
            result.velocity_adjustment.data[1] = vertical_result.velocity_adjustment.data[1];
        }
        
        // Ground detection
        result.on_ground = self.checkGroundContact(result.position, player_aabb);
        
        return result;
    }
    
    // Horizontal movement with step-up mechanics
    fn moveHorizontalWithStepUp(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        // Try direct horizontal movement first
        var test_pos = Vec3.add(start, delta);
        var test_aabb = AABB{
            .min = Vec3.add(test_pos, player_aabb.min),
            .max = Vec3.add(test_pos, player_aabb.max)
        };
        
        if (!self.testCollision(test_aabb)) {
            // Direct movement succeeded
            result.position = test_pos;
            return result;
        }
        
        // Try step-up
        test_pos.data[1] += STEP_HEIGHT;
        test_aabb = AABB{
            .min = Vec3.add(test_pos, player_aabb.min),
            .max = Vec3.add(test_pos, player_aabb.max)
        };
        
        if (!self.testCollision(test_aabb)) {
            // Step-up succeeded, now find the ground
            const ground_result = self.findGround(test_pos, player_aabb, STEP_HEIGHT + GROUND_SNAP_DISTANCE);
            result.position = ground_result.position;
            result.on_ground = ground_result.found_ground;
            return result;
        }
        
        // Try sliding along walls
        result = self.slideAlongWalls(start, delta, player_aabb);
        result.hit_wall = true;
        
        return result;
    }
    
    // Vertical movement handling
    fn moveVertical(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        const test_pos = Vec3.add(start, delta);
        const test_aabb = AABB{
            .min = Vec3.add(test_pos, player_aabb.min),
            .max = Vec3.add(test_pos, player_aabb.max)
        };
        
        if (!self.testCollision(test_aabb)) {
            result.position = test_pos;
        } else {
            // Hit something - determine if ceiling or floor
            if (delta.data[1] > 0) {
                result.hit_ceiling = true;
                result.velocity_adjustment.data[1] = -delta.data[1]; // Stop upward movement
            } else {
                result.velocity_adjustment.data[1] = -delta.data[1]; // Stop downward movement
            }
        }
        
        return result;
    }
    
    // Slide along walls when direct movement fails
    fn slideAlongWalls(self: *const BrushWorld, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        // Try X movement only
        var test_pos = Vec3.new(start.data[0] + delta.data[0], start.data[1], start.data[2]);
        var test_aabb = AABB{
            .min = Vec3.add(test_pos, player_aabb.min),
            .max = Vec3.add(test_pos, player_aabb.max)
        };
        
        if (!self.testCollision(test_aabb)) {
            result.position.data[0] = test_pos.data[0];
        } else {
            result.velocity_adjustment.data[0] = -delta.data[0];
        }
        
        // Try Z movement only
        test_pos = Vec3.new(result.position.data[0], start.data[1], start.data[2] + delta.data[2]);
        test_aabb = AABB{
            .min = Vec3.add(test_pos, player_aabb.min),
            .max = Vec3.add(test_pos, player_aabb.max)
        };
        
        if (!self.testCollision(test_aabb)) {
            result.position.data[2] = test_pos.data[2];
        } else {
            result.velocity_adjustment.data[2] = -delta.data[2];
        }
        
        return result;
    }
    
    // Find ground below a position
    fn findGround(self: *const BrushWorld, start: Vec3, player_aabb: AABB, max_distance: f32) GroundResult {
        var step_down: f32 = GROUND_SNAP_DISTANCE;
        
        while (step_down <= max_distance) {
            const test_pos = Vec3.new(start.data[0], start.data[1] - step_down, start.data[2]);
            const test_aabb = AABB{
                .min = Vec3.add(test_pos, player_aabb.min),
                .max = Vec3.add(test_pos, player_aabb.max)
            };
            
            if (self.testCollision(test_aabb)) {
                // Found ground, back up slightly
                const ground_pos = Vec3.new(start.data[0], start.data[1] - (step_down - GROUND_SNAP_DISTANCE), start.data[2]);
                return GroundResult{ .position = ground_pos, .found_ground = true };
            }
            
            step_down += GROUND_SNAP_DISTANCE;
        }
        
        // No ground found within range
        return GroundResult{ .position = start, .found_ground = false };
    }
    
    // Check if player is on ground
    fn checkGroundContact(self: *const BrushWorld, position: Vec3, player_aabb: AABB) bool {
        const ground_test_pos = Vec3.new(position.data[0], position.data[1] - GROUND_SNAP_DISTANCE, position.data[2]);
        const ground_aabb = AABB{
            .min = Vec3.add(ground_test_pos, player_aabb.min),
            .max = Vec3.add(ground_test_pos, player_aabb.max)
        };
        
        return self.testCollision(ground_aabb);
    }
};

// Result structures for movement operations
pub const MoveResult = struct {
    position: Vec3,
    velocity_adjustment: Vec3, // How much velocity should be adjusted due to collisions
    on_ground: bool,
    hit_ceiling: bool,
    hit_wall: bool,
};

pub const GroundResult = struct {
    position: Vec3,
    found_ground: bool,
};

// Helper functions

fn calculateBounds(planes: []const Plane) AABB {
    // Start with a reasonable default size
    var min = Vec3.new(-100, -100, -100);
    var max = Vec3.new(100, 100, 100);
    
    // Calculate bounds from axis-aligned planes
    for (planes) |plane| {
        const abs_normal = Vec3.new(@abs(plane.normal.data[0]), @abs(plane.normal.data[1]), @abs(plane.normal.data[2]));
        const max_component = @max(@max(abs_normal.data[0], abs_normal.data[1]), abs_normal.data[2]);
        
        // Only process planes that are reasonably axis-aligned
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
    
    // Ensure valid bounds - if calculation failed, use a small default box
    if (min.data[0] > max.data[0]) { min.data[0] = -1; max.data[0] = 1; }
    if (min.data[1] > max.data[1]) { min.data[1] = -1; max.data[1] = 1; }
    if (min.data[2] > max.data[2]) { min.data[2] = -1; max.data[2] = 1; }
    
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
        
        if (plane.distanceToPoint(center) > radius + COLLISION_EPSILON) return false;
    }
    return true;
}

// Swept collision detection for fast-moving objects
pub fn sweptAABBBrush(aabb: AABB, velocity: Vec3, brush: Brush, dt: f32) ?f32 {
    if (!aabb.intersects(brush.bounds) and !AABB.new(
        Vec3.add(aabb.min, Vec3.scale(velocity, dt)),
        Vec3.add(aabb.max, Vec3.scale(velocity, dt))
    ).intersects(brush.bounds)) {
        return null; // No potential collision
    }
    
    var t_enter: f32 = 0;
    var t_exit: f32 = dt;
    
    for (brush.planes) |plane| {
        const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
        const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
        
        const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                      @abs(plane.normal.data[1] * extents.data[1]) +
                      @abs(plane.normal.data[2] * extents.data[2]);
        
        const dist = plane.distanceToPoint(center);
        const vel_dot = Vec3.dot(plane.normal, velocity);
        
        if (@abs(vel_dot) < COLLISION_EPSILON) {
            // Moving parallel to plane
            if (dist > radius + COLLISION_EPSILON) {
                return null; // No collision possible
            }
        } else {
            // Calculate intersection times
            const t1 = (-(dist + radius + COLLISION_EPSILON)) / vel_dot;
            const t2 = (-(dist - radius - COLLISION_EPSILON)) / vel_dot;
            
            const t_near = @min(t1, t2);
            const t_far = @max(t1, t2);
            
            t_enter = @max(t_enter, t_near);
            t_exit = @min(t_exit, t_far);
            
            if (t_enter > t_exit) {
                return null; // No collision
            }
        }
    }
    
    if (t_enter >= 0 and t_enter <= dt) {
        return t_enter;
    }
    
    return null;
}