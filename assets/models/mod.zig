const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;

pub const Mesh = struct {
    name: []const u8,
    vertices: []Vec3,
    indices: []u32,
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    meshes: std.ArrayListUnmanaged(Mesh) = .{},
    
    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator };
    }
    
    pub fn deinit(self: *Registry) void {
        for (self.meshes.items) |mesh| {
            self.allocator.free(mesh.vertices);
            self.allocator.free(mesh.indices);
        }
        self.meshes.deinit(self.allocator);
    }
    
    pub fn load(self: *Registry, path: []const u8, name: []const u8) !*Mesh {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();
        
        const content = try file.readToEndAlloc(self.allocator, 1024 * 1024);
        defer self.allocator.free(content);
        
        var vertices = std.ArrayListUnmanaged(Vec3){};
        var indices = std.ArrayListUnmanaged(u32){};
        defer vertices.deinit(self.allocator);
        defer indices.deinit(self.allocator);
        
        var lines = std.mem.splitScalar(u8, content, '\n');
        while (lines.next()) |line| {
            const trimmed = std.mem.trim(u8, line, " \t\r");
            if (trimmed.len == 0 or trimmed[0] == '#') continue;
            
            if (std.mem.startsWith(u8, trimmed, "v ")) {
                var parts = std.mem.tokenizeAny(u8, trimmed[2..], " \t");
                const x = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                const y = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                const z = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                try vertices.append(self.allocator, Vec3.new(x, y, z));
            } else if (std.mem.startsWith(u8, trimmed, "f ")) {
                var parts = std.mem.tokenizeAny(u8, trimmed[2..], " \t");
                var face_indices: [3]u32 = undefined;
                var i: usize = 0;
                
                while (parts.next()) |part| {
                    if (i >= 3) break;
                    
                    var vertex_part = std.mem.splitScalar(u8, part, '/');
                    const vertex_str = vertex_part.next() orelse "1";
                    const vertex_idx = std.fmt.parseInt(u32, vertex_str, 10) catch 1;
                    face_indices[i] = vertex_idx - 1;
                    i += 1;
                }
                
                if (i == 3) {
                    try indices.append(self.allocator, face_indices[0]);
                    try indices.append(self.allocator, face_indices[1]);
                    try indices.append(self.allocator, face_indices[2]);
                }
            }
        }
        
        const mesh = Mesh{
            .name = name,
            .vertices = try vertices.toOwnedSlice(self.allocator),
            .indices = try indices.toOwnedSlice(self.allocator),
        };
        
        try self.meshes.append(self.allocator, mesh);
        return &self.meshes.items[self.meshes.items.len - 1];
    }
    
    pub fn get(self: *const Registry, model_name: []const u8) ?*const Mesh {
        for (self.meshes.items) |*mesh| {
            if (std.mem.eql(u8, mesh.name, model_name)) {
                return mesh;
            }
        }
        return null;
    }
};