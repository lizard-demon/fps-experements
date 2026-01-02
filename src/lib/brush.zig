// Quake 1 style brush collision system
const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;

pub const Plane = struct {
    normal: Vec3,
    distance: f32,
    
    // Standard plane equation: dot(normal, point) + distance = 0
    // Points inside the brush satisfy: dot(normal, point) + distance <= 0
    pub fn new(normal: Vec3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn fromPoints(a: Vec3, b: Vec3, c: Vec3) Plane {
        const ab = Vec3.sub(b, a);
        const ac = Vec3.sub(c, a);
        const normal = Vec3.normalize(Vec3.cross(ab, ac));
        const distance = -Vec3.dot(normal, a);
        return .{ .normal = normal, .distance = distance };
    }
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 {
        return Vec3.dot(self.normal, point) + self.distance;
    }
    
    // Use consistent epsilon for all collision checks
    pub const COLLISION_EPSILON = 0.001;
    
    pub fn isPointInFront(self: Plane, point: Vec3) bool {
        return self.distanceToPoint(point) > COLLISION_EPSILON;
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
        
        // Create box planes with normals pointing OUTWARD from the solid
        // Using plane equation: dot(normal, point) + distance = 0
        // Points inside satisfy: dot(normal, point) + distance <= 0
        
        // Right face (x = max.x): normal (1,0,0) pointing outward
        planes[0] = Plane.new(Vec3.new(1, 0, 0), -max.data[0]);
        
        // Left face (x = min.x): normal (-1,0,0) pointing outward
        planes[1] = Plane.new(Vec3.new(-1, 0, 0), min.data[0]);
        
        // Top face (y = max.y): normal (0,1,0) pointing outward
        planes[2] = Plane.new(Vec3.new(0, 1, 0), -max.data[1]);
        
        // Bottom face (y = min.y): normal (0,-1,0) pointing outward (downward)
        planes[3] = Plane.new(Vec3.new(0, -1, 0), min.data[1]);
        
        // Front face (z = max.z): normal (0,0,1) pointing outward
        planes[4] = Plane.new(Vec3.new(0, 0, 1), -max.data[2]);
        
        // Back face (z = min.z): normal (0,0,-1) pointing outward
        planes[5] = Plane.new(Vec3.new(0, 0, -1), min.data[2]);
        
        return .{ .planes = planes, .bounds = AABB{ .min = min, .max = max } };
    }
    
    pub fn isPointInside(self: Brush, point: Vec3) bool {
        // A point is inside if it's behind (or on) all planes
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

// Simple bounds calculation for brushes
fn calculateBounds(planes: []const Plane) AABB {
    // Start with a reasonable bounding box and shrink it based on planes
    var min = Vec3.new(-50, -50, -50);
    var max = Vec3.new(50, 50, 50);
    
    // For axis-aligned planes, calculate exact bounds
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

// Simple collision test
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

// Simple movement collision
pub const MoveResult = struct {
    position: Vec3,
    hit: bool = false,
    normal: Vec3 = Vec3.zero(),
};

pub fn moveAABBAgainstBrush(start: Vec3, delta: Vec3, aabb: AABB, brush: Brush) MoveResult {
    var result = MoveResult{ .position = start };
    var pos = start;
    
    // X axis
    if (@abs(delta.data[0]) > Plane.COLLISION_EPSILON) {
        const test_pos = Vec3.new(pos.data[0] + delta.data[0], pos.data[1], pos.data[2]);
        const test_aabb = AABB{
            .min = Vec3.add(test_pos, aabb.min),
            .max = Vec3.add(test_pos, aabb.max)
        };
        
        if (testAABBBrush(test_aabb, brush)) {
            result.hit = true;
            result.normal = if (delta.data[0] > 0) Vec3.new(-1, 0, 0) else Vec3.new(1, 0, 0);
        } else {
            pos.data[0] = test_pos.data[0];
        }
    }
    
    // Y axis
    if (@abs(delta.data[1]) > Plane.COLLISION_EPSILON) {
        const test_pos = Vec3.new(pos.data[0], pos.data[1] + delta.data[1], pos.data[2]);
        const test_aabb = AABB{
            .min = Vec3.add(test_pos, aabb.min),
            .max = Vec3.add(test_pos, aabb.max)
        };
        
        if (testAABBBrush(test_aabb, brush)) {
            result.hit = true;
            result.normal = if (delta.data[1] > 0) Vec3.new(0, -1, 0) else Vec3.new(0, 1, 0);
        } else {
            pos.data[1] = test_pos.data[1];
        }
    }
    
    // Z axis
    if (@abs(delta.data[2]) > Plane.COLLISION_EPSILON) {
        const test_pos = Vec3.new(pos.data[0], pos.data[1], pos.data[2] + delta.data[2]);
        const test_aabb = AABB{
            .min = Vec3.add(test_pos, aabb.min),
            .max = Vec3.add(test_pos, aabb.max)
        };
        
        if (testAABBBrush(test_aabb, brush)) {
            result.hit = true;
            result.normal = if (delta.data[2] > 0) Vec3.new(0, 0, -1) else Vec3.new(0, 0, 1);
        } else {
            pos.data[2] = test_pos.data[2];
        }
    }
    
    result.position = pos;
    return result;
}