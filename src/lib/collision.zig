// Quake-style brush collision system
const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const EPSILON: f32 = 0.001;
pub const STEP_HEIGHT: f32 = 0.25;
pub const GROUND_SNAP: f32 = 0.05;

// Core geometric primitive: a plane in 3D space
pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    pub fn new(normal: Vec3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
    
    pub fn isInFront(self: Plane, point: Vec3) bool {
        return self.distanceToPoint(point) > EPSILON;
    }
};

// A convex volume defined by planes (Quake brush)
pub const Brush = struct {
    planes: []const Plane,
    bounds: AABB,
    
    pub fn new(planes: []const Plane, bounds: AABB) Brush {
        return .{ .planes = planes, .bounds = bounds };
    }
    
    pub fn intersects(self: Brush, aabb: AABB) bool {
        // Early rejection using bounding boxes
        if (!self.bounds.intersects(aabb)) return false;
        
        // Separating Axis Theorem: test each plane as separating axis
        for (self.planes) |plane| {
            const center = Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5);
            const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
            
            // Project AABB onto plane normal
            const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                          @abs(plane.normal.data[1] * extents.data[1]) +
                          @abs(plane.normal.data[2] * extents.data[2]);
            
            // If AABB is completely on positive side of plane, no intersection
            if (plane.distanceToPoint(center) > radius + EPSILON) return false;
        }
        return true;
    }
    
    pub fn contains(self: Brush, point: Vec3) bool {
        for (self.planes) |plane| {
            if (plane.isInFront(point)) return false;
        }
        return true;
    }
    
    // Sweep an AABB through this brush - returns exact collision point
    pub fn sweepAABB(self: Brush, start: Vec3, delta: Vec3, aabb: AABB) SweepResult {
        // Early rejection using expanded bounding box
        const expanded_bounds = AABB{
            .min = Vec3.sub(self.bounds.min, Vec3.sub(aabb.max, aabb.min)),
            .max = Vec3.sub(self.bounds.max, Vec3.sub(aabb.min, aabb.max)),
        };
        
        const ray_result = raycastAABB(start, delta, expanded_bounds);
        if (!ray_result.hit) {
            return SweepResult.miss(Vec3.add(start, delta));
        }
        
        // Test against each plane
        var closest_fraction: f32 = 1.0;
        var closest_normal = Vec3.zero();
        var closest_plane_index: u32 = 0;
        var found_hit = false;
        
        for (self.planes, 0..) |plane, i| {
            const result = sweepAABBPlane(start, delta, aabb, plane);
            if (result.hit and result.fraction < closest_fraction) {
                closest_fraction = result.fraction;
                closest_normal = result.normal;
                closest_plane_index = @intCast(i);
                found_hit = true;
            }
        }
        
        if (found_hit) {
            const hit_pos = Vec3.add(start, Vec3.scale(delta, closest_fraction));
            return SweepResult.collision(closest_fraction, closest_normal, hit_pos, closest_plane_index);
        }
        
        return SweepResult.miss(Vec3.add(start, delta));
    }
};

// Core sweep collision functions - mathematically precise
fn raycastAABB(start: Vec3, delta: Vec3, aabb: AABB) SweepResult {
    // Ray-AABB intersection using slab method
    var t_min: f32 = 0.0;
    var t_max: f32 = 1.0;
    
    for (0..3) |axis| {
        if (@abs(delta.data[axis]) < EPSILON) {
            // Ray is parallel to slab
            if (start.data[axis] < aabb.min.data[axis] or start.data[axis] > aabb.max.data[axis]) {
                return SweepResult.miss(Vec3.add(start, delta));
            }
        } else {
            // Calculate intersection distances
            const inv_dir = 1.0 / delta.data[axis];
            var t1 = (aabb.min.data[axis] - start.data[axis]) * inv_dir;
            var t2 = (aabb.max.data[axis] - start.data[axis]) * inv_dir;
            
            if (t1 > t2) {
                const temp = t1;
                t1 = t2;
                t2 = temp;
            }
            
            t_min = @max(t_min, t1);
            t_max = @min(t_max, t2);
            
            if (t_min > t_max) {
                return SweepResult.miss(Vec3.add(start, delta));
            }
        }
    }
    
    if (t_min >= 0.0 and t_min <= 1.0) {
        const hit_pos = Vec3.add(start, Vec3.scale(delta, t_min));
        return SweepResult.collision(t_min, Vec3.zero(), hit_pos, 0);
    }
    
    return SweepResult.miss(Vec3.add(start, delta));
}

fn sweepAABBPlane(start: Vec3, delta: Vec3, aabb: AABB, plane: Plane) SweepResult {
    // Calculate AABB extents projected onto plane normal
    const extents = Vec3.scale(Vec3.sub(aabb.max, aabb.min), 0.5);
    const radius = @abs(plane.normal.data[0] * extents.data[0]) +
                   @abs(plane.normal.data[1] * extents.data[1]) +
                   @abs(plane.normal.data[2] * extents.data[2]);
    
    // AABB center relative to plane
    const center = Vec3.add(start, Vec3.scale(Vec3.add(aabb.min, aabb.max), 0.5));
    const dist_to_plane = plane.distanceToPoint(center);
    
    // Velocity component along plane normal
    const velocity_dot = Vec3.dot(delta, plane.normal);
    
    // If moving away from plane or parallel, no collision
    if (velocity_dot >= -EPSILON) {
        return SweepResult.miss(Vec3.add(start, delta));
    }
    
    // Calculate intersection time
    const adjusted_distance = dist_to_plane - radius;
    
    // Already intersecting
    if (adjusted_distance <= EPSILON) {
        return SweepResult.collision(0.0, plane.normal, start, 0);
    }
    
    const t = -adjusted_distance / velocity_dot;
    
    if (t >= 0.0 and t <= 1.0) {
        const hit_pos = Vec3.add(start, Vec3.scale(delta, t));
        return SweepResult.collision(t, plane.normal, hit_pos, 0);
    }
    
    return SweepResult.miss(Vec3.add(start, delta));
}

// Factory for creating common brush shapes
pub const BrushFactory = struct {
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) BrushFactory {
        return .{ .allocator = allocator };
    }
    
    pub fn createBox(self: BrushFactory, min: Vec3, max: Vec3) !Brush {
        var planes = try self.allocator.alloc(Plane, 6);
        planes[0] = Plane.new(Vec3.new( 1,  0,  0), -max.data[0]); // Right
        planes[1] = Plane.new(Vec3.new(-1,  0,  0),  min.data[0]); // Left  
        planes[2] = Plane.new(Vec3.new( 0,  1,  0), -max.data[1]); // Top
        planes[3] = Plane.new(Vec3.new( 0, -1,  0),  min.data[1]); // Bottom
        planes[4] = Plane.new(Vec3.new( 0,  0,  1), -max.data[2]); // Front
        planes[5] = Plane.new(Vec3.new( 0,  0, -1),  min.data[2]); // Back
        
        return Brush.new(planes, AABB.new(min, max));
    }
    
    pub fn createFromPlanes(self: BrushFactory, planes: []const Plane) !Brush {
        const owned_planes = try self.allocator.dupe(Plane, planes);
        const bounds = calculateBounds(owned_planes);
        return Brush.new(owned_planes, bounds);
    }
    
    pub fn destroy(self: BrushFactory, brush: Brush) void {
        self.allocator.free(brush.planes);
    }
};

// Calculate tight AABB from axis-aligned planes
fn calculateBounds(planes: []const Plane) AABB {
    var min = Vec3.new(-100, -100, -100);
    var max = Vec3.new( 100,  100,  100);
    
    for (planes) |plane| {
        const abs_normal = Vec3.new(@abs(plane.normal.data[0]), @abs(plane.normal.data[1]), @abs(plane.normal.data[2]));
        
        // Find dominant axis
        var dominant_axis: u8 = 0;
        if (abs_normal.data[1] > abs_normal.data[0] and abs_normal.data[1] > abs_normal.data[2]) {
            dominant_axis = 1;
        } else if (abs_normal.data[2] > abs_normal.data[0] and abs_normal.data[2] > abs_normal.data[1]) {
            dominant_axis = 2;
        }
        
        if (abs_normal.data[dominant_axis] > 0.9) {
            if (plane.normal.data[dominant_axis] > 0) {
                max.data[dominant_axis] = @min(max.data[dominant_axis], -plane.distance);
            } else {
                min.data[dominant_axis] = @max(min.data[dominant_axis], plane.distance);
            }
        }
    }
    
    // Ensure valid bounds
    for (0..3) |i| {
        if (min.data[i] > max.data[i]) {
            min.data[i] = -1;
            max.data[i] = 1;
        }
    }
    
    return AABB.new(min, max);
}

// Sweep collision result - precise continuous collision detection
pub const SweepResult = struct {
    hit: bool,
    fraction: f32,      // 0.0 to 1.0, how far along the sweep we got
    normal: Vec3,       // surface normal at impact point
    position: Vec3,     // exact contact position
    plane_index: u32,   // which plane we hit (for debugging)
    
    pub fn miss(end_position: Vec3) SweepResult {
        return .{
            .hit = false,
            .fraction = 1.0,
            .normal = Vec3.zero(),
            .position = end_position,
            .plane_index = 0,
        };
    }
    
    pub fn collision(fraction: f32, normal: Vec3, position: Vec3, plane_index: u32) SweepResult {
        return .{
            .hit = true,
            .fraction = fraction,
            .normal = normal,
            .position = position,
            .plane_index = plane_index,
        };
    }
};

// Movement result with detailed collision information
pub const MoveResult = struct {
    position: Vec3,
    velocity_adjustment: Vec3,
    on_ground: bool,
    hit_ceiling: bool,
    hit_wall: bool,
};

// World containing brushes with spatial queries
pub const World = struct {
    brushes: std.ArrayListUnmanaged(Brush) = .{},
    factory: BrushFactory,
    
    pub fn init(allocator: std.mem.Allocator) World {
        return .{ .factory = BrushFactory.init(allocator) };
    }
    
    pub fn deinit(self: *World) void {
        for (self.brushes.items) |brush| {
            self.factory.destroy(brush);
        }
        self.brushes.deinit(self.factory.allocator);
    }
    
    pub fn addBrush(self: *World, brush: Brush) !void {
        try self.brushes.append(self.factory.allocator, brush);
    }
    
    pub fn addBox(self: *World, center: Vec3, size: Vec3) !void {
        const half = Vec3.scale(size, 0.5);
        const brush = try self.factory.createBox(Vec3.sub(center, half), Vec3.add(center, half));
        try self.addBrush(brush);
    }
    
    pub fn testCollision(self: *const World, aabb: AABB) bool {
        for (self.brushes.items) |brush| {
            if (brush.intersects(aabb)) return true;
        }
        return false;
    }
    
    // Sweep an AABB through the world - returns first collision
    pub fn sweepAABB(self: *const World, start: Vec3, delta: Vec3, aabb: AABB) SweepResult {
        var closest_result = SweepResult.miss(Vec3.add(start, delta));
        
        for (self.brushes.items) |brush| {
            const result = brush.sweepAABB(start, delta, aabb);
            if (result.hit and result.fraction < closest_result.fraction) {
                closest_result = result;
            }
        }
        
        return closest_result;
    }
    
    // Quake-style player movement with step-up and wall sliding
    pub fn movePlayer(self: *const World, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{
            .position = start,
            .velocity_adjustment = Vec3.zero(),
            .on_ground = false,
            .hit_ceiling = false,
            .hit_wall = false,
        };
        
        // Horizontal movement with step-up
        const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
        if (Vec3.length(horizontal) > EPSILON) {
            result = self.moveHorizontal(result.position, horizontal, player_aabb);
        }
        
        // Vertical movement
        if (@abs(delta.data[1]) > EPSILON) {
            const vertical_result = self.moveVertical(result.position, delta.data[1], player_aabb);
            result.position.data[1] = vertical_result.position.data[1];
            result.velocity_adjustment.data[1] = vertical_result.velocity_adjustment.data[1];
            result.hit_ceiling = vertical_result.hit_ceiling;
        }
        
        // Ground detection
        result.on_ground = self.isOnGround(result.position, player_aabb);
        
        return result;
    }
    
    fn moveHorizontal(self: *const World, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        const target = Vec3.add(start, delta);
        const target_aabb = AABB{ .min = Vec3.add(target, player_aabb.min), .max = Vec3.add(target, player_aabb.max) };
        
        if (!self.testCollision(target_aabb)) {
            // Direct movement succeeded
            result.position = target;
            return result;
        }
        
        // Try step-up
        const step_up = Vec3.new(target.data[0], target.data[1] + STEP_HEIGHT, target.data[2]);
        const step_aabb = AABB{ .min = Vec3.add(step_up, player_aabb.min), .max = Vec3.add(step_up, player_aabb.max) };
        
        if (!self.testCollision(step_aabb)) {
            // Step-up succeeded, find ground
            result.position = self.findGround(step_up, player_aabb);
            return result;
        }
        
        // Try wall sliding
        result = self.slideAlongWalls(start, delta, player_aabb);
        result.hit_wall = true;
        return result;
    }
    
    fn moveVertical(self: *const World, start: Vec3, delta_y: f32, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        const target = Vec3.new(start.data[0], start.data[1] + delta_y, start.data[2]);
        const target_aabb = AABB{ .min = Vec3.add(target, player_aabb.min), .max = Vec3.add(target, player_aabb.max) };
        
        if (!self.testCollision(target_aabb)) {
            result.position = target;
        } else {
            result.velocity_adjustment.data[1] = -delta_y;
            if (delta_y > 0) result.hit_ceiling = true;
        }
        
        return result;
    }
    
    fn slideAlongWalls(self: *const World, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
        var result = MoveResult{ .position = start, .velocity_adjustment = Vec3.zero(), .on_ground = false, .hit_ceiling = false, .hit_wall = false };
        
        // Try X-axis movement
        const x_target = Vec3.new(start.data[0] + delta.data[0], start.data[1], start.data[2]);
        const x_aabb = AABB{ .min = Vec3.add(x_target, player_aabb.min), .max = Vec3.add(x_target, player_aabb.max) };
        
        if (!self.testCollision(x_aabb)) {
            result.position.data[0] = x_target.data[0];
        } else {
            result.velocity_adjustment.data[0] = -delta.data[0];
        }
        
        // Try Z-axis movement
        const z_target = Vec3.new(result.position.data[0], start.data[1], start.data[2] + delta.data[2]);
        const z_aabb = AABB{ .min = Vec3.add(z_target, player_aabb.min), .max = Vec3.add(z_target, player_aabb.max) };
        
        if (!self.testCollision(z_aabb)) {
            result.position.data[2] = z_target.data[2];
        } else {
            result.velocity_adjustment.data[2] = -delta.data[2];
        }
        
        return result;
    }
    
    fn findGround(self: *const World, start: Vec3, player_aabb: AABB) Vec3 {
        var step_down: f32 = GROUND_SNAP;
        while (step_down <= STEP_HEIGHT + GROUND_SNAP) {
            const test_pos = Vec3.new(start.data[0], start.data[1] - step_down, start.data[2]);
            const test_aabb = AABB{ .min = Vec3.add(test_pos, player_aabb.min), .max = Vec3.add(test_pos, player_aabb.max) };
            
            if (self.testCollision(test_aabb)) {
                return Vec3.new(start.data[0], start.data[1] - (step_down - GROUND_SNAP), start.data[2]);
            }
            step_down += GROUND_SNAP;
        }
        return start;
    }
    
    fn isOnGround(self: *const World, position: Vec3, player_aabb: AABB) bool {
        const ground_test = Vec3.new(position.data[0], position.data[1] - GROUND_SNAP, position.data[2]);
        const ground_aabb = AABB{ .min = Vec3.add(ground_test, player_aabb.min), .max = Vec3.add(ground_test, player_aabb.max) };
        return self.testCollision(ground_aabb);
    }
};