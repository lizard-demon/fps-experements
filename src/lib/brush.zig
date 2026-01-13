const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const HullType = enum { point, standing };
pub const CollisionResult = struct { normal: Vec3, distance: f32 };

pub const Capsule = struct {
    start: Vec3, 
    end: Vec3, 
    radius: f32,
    
    pub fn fromHull(pos: Vec3, hull_type: HullType) Capsule {
        return switch (hull_type) {
            .point => .{ .start = pos, .end = pos, .radius = 0.0 },
            .standing => .{ 
                .start = Vec3.add(pos, Vec3.new(0, -0.7, 0)), 
                .end = Vec3.add(pos, Vec3.new(0, 0.7, 0)), 
                .radius = 0.3 
            },
        };
    }
    
    pub fn center(self: Capsule) Vec3 {
        return Vec3.scale(Vec3.add(self.start, self.end), 0.5);
    }
    
    pub fn bounds(self: Capsule) AABB {
        const r = Vec3.new(self.radius, self.radius, self.radius);
        return AABB{
            .min = Vec3.sub(Vec3.min(self.start, self.end), r),
            .max = Vec3.add(Vec3.max(self.start, self.end), r),
        };
    }
};

pub const Plane = struct {
    normal: Vec3, 
    distance: f32,
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 { 
        return Vec3.dot(self.normal, point) + self.distance; 
    }
};

pub const Brush = struct {
    planes: []const Plane, 
    bounds: AABB,
    
    pub fn checkCapsule(self: Brush, capsule: Capsule) ?CollisionResult {
        var closest_normal: ?Vec3 = null;
        var closest_distance: f32 = std.math.floatMax(f32);
        
        for (self.planes) |plane| {
            const distance = plane.distanceToPoint(capsule.center()) - capsule.radius;
            if (distance > 0.001) return null;
            if (distance > -closest_distance) {
                closest_distance = -distance;
                closest_normal = plane.normal;
            }
        }
        
        return if (closest_normal) |normal| 
            CollisionResult{ .normal = normal, .distance = closest_distance } 
        else null;
    }
    
    pub fn getBounds(self: Brush) AABB { 
        return self.bounds; 
    }
};