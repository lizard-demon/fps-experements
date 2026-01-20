const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;
const AABB = math.AABB;

pub const Plane = struct { normal: Vec3, distance: f32 };

pub const Planes = struct {
    normals: []const Vec3,
    distances: []const f32,
    
    pub fn len(self: Planes) usize {
        return self.normals.len;
    }
    
    pub fn distanceToPoint(self: Planes, index: usize, point: Vec3) f32 {
        return Vec3.dot(self.normals[index], point) + self.distances[index];
    }
    
    pub fn rayIntersect(self: Planes, index: usize, ray_start: Vec3, ray_dir: Vec3) ?f32 {
        const normal = self.normals[index];
        const distance = self.distances[index];
        
        const denom = Vec3.dot(normal, ray_dir);
        if (@abs(denom) < std.math.floatEpsAt(f32, denom)) return null; // Ray parallel to plane
        
        const t = -(Vec3.dot(normal, ray_start) + distance) / denom;
        return if (t >= 0) t else null;
    }
};

// Simple mesh structure for brush geometry
pub const BrushMesh = struct {
    vertices: []Vec3,
    indices: []u16,
    allocator: std.mem.Allocator,
    
    pub fn deinit(self: *BrushMesh) void {
        self.allocator.free(self.vertices);
        self.allocator.free(self.indices);
    }
};

pub const Brush = struct {
    planes: Planes,
    bounds: AABB,
    
    pub fn expand(self: Brush, radius: f32, allocator: std.mem.Allocator) !Brush {
        // Expand each plane outward by the radius
        var expanded_normals = try allocator.alloc(Vec3, self.planes.len());
        var expanded_distances = try allocator.alloc(f32, self.planes.len());
        
        for (0..self.planes.len()) |i| {
            expanded_normals[i] = self.planes.normals[i];
            expanded_distances[i] = self.planes.distances[i] - radius; // Move plane outward
        }
        
        // Expand bounds by radius
        const radius_vec = Vec3.new(radius, radius, radius);
        const expanded_bounds = AABB{
            .min = Vec3.sub(self.bounds.min, radius_vec),
            .max = Vec3.add(self.bounds.max, radius_vec),
        };
        
        return Brush{
            .planes = .{
                .normals = expanded_normals,
                .distances = expanded_distances,
            },
            .bounds = expanded_bounds,
        };
    }
    
    pub fn rayIntersect(self: Brush, ray_start: Vec3, ray_dir: Vec3, max_distance: f32) ?Plane {
        var entry_time: f32 = 0;
        var exit_time: f32 = max_distance;
        var hit_normal: Vec3 = Vec3.zero();
        
        for (0..self.planes.len()) |i| {
            const normal = self.planes.normals[i];
            
            if (self.planes.rayIntersect(i, ray_start, ray_dir)) |t| {
                const dot = Vec3.dot(normal, ray_dir);
                if (dot < 0) { // Entering
                    if (t > entry_time) {
                        entry_time = t;
                        hit_normal = normal;
                    }
                } else { // Exiting
                    if (t < exit_time) {
                        exit_time = t;
                    }
                }
            } else {
                // Ray parallel to plane - check if we're on the wrong side
                if (self.planes.distanceToPoint(i, ray_start) > 0) return null;
            }
        }
        
        if (entry_time <= exit_time and entry_time <= max_distance and entry_time >= std.math.floatEpsAt(f32, entry_time)) {
            return Plane{ .normal = hit_normal, .distance = entry_time };
        }
        
        return null;
    }
    
    pub fn generateMesh(self: Brush, allocator: std.mem.Allocator) !BrushMesh {
        // Find all vertices by intersecting three planes
        var hull_vertices = std.ArrayListUnmanaged(Vec3){};
        defer hull_vertices.deinit(allocator);
        
        for (0..self.planes.len()) |i| {
            for (i + 1..self.planes.len()) |j| {
                for (j + 1..self.planes.len()) |k| {
                    if (intersectThreePlanes(self.planes, i, j, k)) |vertex| {
                        // Check if vertex is inside brush and not duplicate
                        if (isVertexInsideBrush(vertex, self.planes) and !isDuplicateVertex(vertex, hull_vertices.items)) {
                            try hull_vertices.append(allocator, vertex);
                        }
                    }
                }
            }
        }
        
        if (hull_vertices.items.len < 4) {
            return BrushMesh{
                .vertices = &[_]Vec3{},
                .indices = &[_]u16{},
                .allocator = allocator,
            };
        }
        
        // Generate convex hull faces
        const hull_faces = try generateConvexHullFaces(hull_vertices.items, allocator);
        defer allocator.free(hull_faces);
        
        // Copy vertices and indices
        const vertices = try allocator.dupe(Vec3, hull_vertices.items);
        var indices = std.ArrayListUnmanaged(u16){};
        
        for (hull_faces) |face| {
            try indices.append(allocator, face[0]);
            try indices.append(allocator, face[1]);
            try indices.append(allocator, face[2]);
        }
        
        return BrushMesh{
            .vertices = vertices,
            .indices = try indices.toOwnedSlice(allocator),
            .allocator = allocator,
        };
    }
};

fn intersectThreePlanes(planes: Planes, i: usize, j: usize, k: usize) ?Vec3 {
    const n1 = planes.normals[i];
    const n2 = planes.normals[j];
    const n3 = planes.normals[k];
    const d1 = planes.distances[i];
    const d2 = planes.distances[j];
    const d3 = planes.distances[k];
    
    const det = Vec3.dot(n1, Vec3.cross(n2, n3));
    if (@abs(det) < 0.001) return null;
    
    const c1 = Vec3.cross(n2, n3);
    const c2 = Vec3.cross(n3, n1);
    const c3 = Vec3.cross(n1, n2);
    
    return Vec3.scale(Vec3.add(Vec3.add(Vec3.scale(c1, -d1), Vec3.scale(c2, -d2)), Vec3.scale(c3, -d3)), 1.0 / det);
}

fn isVertexInsideBrush(vertex: Vec3, planes: Planes) bool {
    for (0..planes.len()) |plane_idx| {
        if (planes.distanceToPoint(plane_idx, vertex) > 0.001) return false;
    }
    return true;
}

fn isDuplicateVertex(vertex: Vec3, existing: []Vec3) bool {
    for (existing) |existing_vertex| {
        if (Vec3.dist(vertex, existing_vertex) < 0.01) return true;
    }
    return false;
}

fn generateConvexHullFaces(vertices: []Vec3, allocator: std.mem.Allocator) ![][3]u16 {
    if (vertices.len < 4) return &[_][3]u16{};
    
    var faces = std.ArrayListUnmanaged([3]u16){};
    
    // Simple approach: test all triangles, keep those that are hull faces
    for (0..vertices.len) |i| {
        for (i + 1..vertices.len) |j| {
            for (j + 1..vertices.len) |k| {
                const v0 = vertices[i];
                const v1 = vertices[j]; 
                const v2 = vertices[k];
                
                const edge1 = Vec3.sub(v1, v0);
                const edge2 = Vec3.sub(v2, v0);
                const normal = Vec3.cross(edge1, edge2);
                
                if (Vec3.length(normal) < 0.001) continue;
                
                // Check if all other vertices are on one side of this triangle
                var is_hull_face = true;
                var side_sign: ?f32 = null;
                
                for (vertices, 0..) |test_vertex, idx| {
                    if (idx == i or idx == j or idx == k) continue;
                    
                    const dot = Vec3.dot(normal, Vec3.sub(test_vertex, v0));
                    if (@abs(dot) > 0.001) {
                        if (side_sign == null) {
                            side_sign = dot;
                        } else if (side_sign.? * dot < 0) {
                            is_hull_face = false;
                            break;
                        }
                    }
                }
                
                if (is_hull_face) {
                    const face: [3]u16 = if ((side_sign orelse 1) > 0) 
                        .{ @intCast(i), @intCast(k), @intCast(j) }
                    else 
                        .{ @intCast(i), @intCast(j), @intCast(k) };
                    try faces.append(allocator, face);
                }
            }
        }
    }
    
    return try faces.toOwnedSlice(allocator);
}