// Models Module - Registry system
// Usage: const models = @import("models");

const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;

pub const Mesh = struct {
    name: []const u8,
    vertices: []Vec3,
    indices: []u32,
    allocator: std.mem.Allocator,
    
    pub fn deinit(self: *Mesh) void {
        self.allocator.free(self.name);
        self.allocator.free(self.vertices);
        self.allocator.free(self.indices);
    }
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    meshes: std.StringHashMap(Mesh),
    
    pub fn init(allocator: std.mem.Allocator) !Registry {
        var registry = Registry{
            .allocator = allocator,
            .meshes = std.StringHashMap(Mesh).init(allocator),
        };
        
        // Open directory
        var dir = std.fs.cwd().openDir("assets/models", .{ .iterate = true }) catch |err| {
            std.log.err("Could not open models directory: assets/models ({})", .{err});
            return err;
        };
        defer dir.close();
        
        var loaded_count: u32 = 0;
        var iterator = dir.iterate();
        
        while (try iterator.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".obj")) continue;
            
            // Extract name without extension
            const name_end = std.mem.lastIndexOf(u8, entry.name, ".") orelse entry.name.len;
            const mesh_name = try allocator.dupe(u8, entry.name[0..name_end]);
            
            // Build full path
            const full_path = try std.fmt.allocPrint(allocator, "assets/models/{s}", .{entry.name});
            defer allocator.free(full_path);
            
            // Load mesh file
            if (registry.load(full_path, mesh_name)) |mesh| {
                try registry.meshes.put(mesh_name, mesh);
                loaded_count += 1;
                std.log.info("Loaded model: {s} ({} vertices, {} triangles)", .{ mesh_name, mesh.vertices.len, mesh.indices.len / 3 });
            } else |err| {
                std.log.warn("Failed to load model {s}: {}", .{ entry.name, err });
                allocator.free(mesh_name);
            }
        }
        
        if (loaded_count == 0) {
            std.log.warn("No OBJ model files found in assets/models", .{});
        } else {
            std.log.info("Model registry loaded {} models from assets/models", .{loaded_count});
        }
        
        return registry;
    }
    
    pub fn deinit(self: *Registry) void {
        var iterator = self.meshes.iterator();
        while (iterator.next()) |entry| {
            entry.value_ptr.deinit();
        }
        self.meshes.deinit();
    }
    
    fn load(self: *Registry, path: []const u8, name: []const u8) !Mesh {
        const file = std.fs.cwd().openFile(path, .{}) catch |err| {
            std.log.err("Failed to open OBJ file: {s}", .{path});
            return err;
        };
        defer file.close();
        
        const content = try file.readToEndAlloc(self.allocator, 1024 * 1024); // 1MB max
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
                // Parse vertex: "v x y z"
                var parts = std.mem.tokenizeAny(u8, trimmed[2..], " \t");
                const x = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                const y = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                const z = std.fmt.parseFloat(f32, parts.next() orelse "0") catch 0;
                try vertices.append(self.allocator, Vec3.new(x, y, z));
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
                    try indices.append(self.allocator, face_indices[0]);
                    try indices.append(self.allocator, face_indices[1]);
                    try indices.append(self.allocator, face_indices[2]);
                }
            }
        }
        
        return Mesh{
            .name = name,
            .vertices = try vertices.toOwnedSlice(self.allocator),
            .indices = try indices.toOwnedSlice(self.allocator),
            .allocator = self.allocator,
        };
    }
    
    pub fn get(self: *const Registry, model_name: []const u8) ?*const Mesh {
        return self.meshes.getPtr(model_name);
    }
};