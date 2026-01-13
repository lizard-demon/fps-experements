const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const HullType = enum { point, standing };
pub const CollisionResult = struct { normal: Vec3, distance: f32 };

pub const Config = struct {
    standing_height: f32 = 0.7,
    standing_radius: f32 = 0.3,
    collision_epsilon: f32 = 0.001,
};

pub const Capsule = struct {
    start: Vec3, 
    end: Vec3, 
    radius: f32,
    
    pub fn fromHull(pos: Vec3, hull_type: HullType, config: Config) Capsule {
        return switch (hull_type) {
            .point => .{ .start = pos, .end = pos, .radius = 0.0 },
            .standing => .{ 
                .start = Vec3.add(pos, Vec3.new(0, -config.standing_height, 0)), 
                .end = Vec3.add(pos, Vec3.new(0, config.standing_height, 0)), 
                .radius = config.standing_radius 
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
    
    pub fn rayIntersect(self: Plane, ray_start: Vec3, ray_dir: Vec3) ?f32 {
        const denom = Vec3.dot(self.normal, ray_dir);
        if (@abs(denom) < 0.0001) return null; // Ray parallel to plane
        
        const t = -(Vec3.dot(self.normal, ray_start) + self.distance) / denom;
        return if (t >= 0) t else null;
    }
};

pub const Brush = struct {
    planes: []const Plane, 
    bounds: AABB,
    
    pub fn checkCapsule(self: Brush, capsule: Capsule, config: Config) ?CollisionResult {
        var closest_normal: ?Vec3 = null;
        var closest_distance: f32 = std.math.floatMax(f32);
        
        for (self.planes) |plane| {
            const distance = plane.distanceToPoint(capsule.center()) - capsule.radius;
            if (distance > config.collision_epsilon) return null;
            if (distance > -closest_distance) {
                closest_distance = -distance;
                closest_normal = plane.normal;
            }
        }
        
        return if (closest_normal) |normal| 
            CollisionResult{ .normal = normal, .distance = closest_distance } 
        else null;
    }
    
    pub fn rayIntersect(self: Brush, ray_start: Vec3, ray_dir: Vec3, max_distance: f32) ?CollisionResult {
        var entry_time: f32 = 0;
        var exit_time: f32 = max_distance;
        var hit_normal: Vec3 = Vec3.zero();
        
        for (self.planes) |plane| {
            if (plane.rayIntersect(ray_start, ray_dir)) |t| {
                const dot = Vec3.dot(plane.normal, ray_dir);
                if (dot < 0) { // Entering
                    if (t > entry_time) {
                        entry_time = t;
                        hit_normal = plane.normal;
                    }
                } else { // Exiting
                    if (t < exit_time) {
                        exit_time = t;
                    }
                }
            } else {
                // Ray parallel to plane - check if we're on the wrong side
                if (plane.distanceToPoint(ray_start) > 0) return null;
            }
        }
        
        if (entry_time <= exit_time and entry_time <= max_distance) {
            return CollisionResult{ .normal = hit_normal, .distance = entry_time };
        }
        
        return null;
    }
    
    pub fn getBounds(self: Brush) AABB { 
        return self.bounds; 
    }
};