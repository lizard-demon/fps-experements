const std = @import("std");
const math = @import("math");
const Vec3 = math.Vec3;

pub const Mesh = struct {
    vertices: []Vec3,
    indices: []u32,
    allocator: std.mem.Allocator,
    
    pub fn deinit(self: *Mesh) void {
        self.allocator.free(self.vertices);
        self.allocator.free(self.indices);
    }
};

pub fn loadOBJ(path: []const u8, allocator: std.mem.Allocator) !Mesh {
    const file = std.fs.cwd().openFile(path, .{}) catch |err| {
        std.log.err("Failed to open OBJ file: {s}", .{path});
        return err;
    };
    defer file.close();
    
    const content = try file.readToEndAlloc(allocator, 1024 * 1024); // 1MB max
    defer allocator.free(content);
    
    var vertices = std.ArrayListUnmanaged(Vec3){};
    var indices = std.ArrayListUnmanaged(u32){};
    defer vertices.deinit(allocator);
    defer indices.deinit(allocator);
    
    var lines = std.mem.splitScalar(u8, content, '\n');
    while (lines.next()) |line| {
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len == 0 or trimmed[0] == '#') continue;
        
        if (std.mem.startsWith(u8, trimmed, "v ")) {
            // Parse vertex: "v x y z"
            var parts = std.mem.tokenizeAny(u8, trimmed[2..], " \t");
            const x = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
            const y = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
            const z = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
            try vertices.append(allocator, Vec3.new(x, y, z));
        } else if (std.mem.startsWith(u8, trimmed, "f ")) {
            // Parse face: "f v1 v2 v3" or "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3"
            var parts = std.mem.tokenizeAny(u8, trimmed[2..], " \t");
            var face_indices: [3]u32 = undefined;
            var i: usize = 0;
            
            while (parts.next()) |part| {
                if (i >= 3) break; // Only handle triangles
                
                // Extract vertex index (before first '/' if present)
                var vertex_part = std.mem.splitScalar(u8, part, '/');
                const vertex_str = vertex_part.next() orelse "1";
                const vertex_idx = std.fmt.parseInt(u32, vertex_str, 10) catch 1;
                face_indices[i] = vertex_idx - 1; // OBJ is 1-indexed
                i += 1;
            }
            
            if (i == 3) {
                try indices.append(allocator, face_indices[0]);
                try indices.append(allocator, face_indices[1]);
                try indices.append(allocator, face_indices[2]);
            }
        }
    }
    
    return Mesh{
        .vertices = try vertices.toOwnedSlice(allocator),
        .indices = try indices.toOwnedSlice(allocator),
        .allocator = allocator,
    };
}