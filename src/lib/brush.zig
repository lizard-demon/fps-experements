const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const CollisionResult = struct { normal: Vec3, distance: f32 };

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