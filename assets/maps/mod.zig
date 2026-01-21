// Map Module - Registry system
// Usage: const maps = @import("maps");

const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;

pub const BrushJson = struct {
    type: []const u8,
    position: [3]f32,
    size: ?[3]f32 = null,
    width: ?f32 = null,
    height: ?f32 = null,
    angle: ?f32 = null,
};

pub const MapJson = struct {
    name: []const u8,
    spawn_position: [3]f32,
    brushes: []BrushJson,
};

pub const BrushData = union(enum) {
    box: struct {
        position: [3]f32,
        size: [3]f32,
    },
    slope: struct {
        position: [3]f32,
        width: f32,
        height: f32,
        angle: f32,
    },
};

pub const Data = struct {
    name: []const u8,
    spawn_position: [3]f32,
    brushes: []BrushData,
    
    pub fn getSpawnPosition(self: *const Data) Vec3 {
        return Vec3.new(
            self.spawn_position[0],
            self.spawn_position[1],
            self.spawn_position[2]
        );
    }
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    maps: std.StringHashMap(Data),
    
    pub fn init(allocator: std.mem.Allocator) !Registry {
        var registry = Registry{
            .allocator = allocator,
            .maps = std.StringHashMap(Data).init(allocator),
        };
        
        // Open directory
        var dir = std.fs.cwd().openDir("assets/maps", .{ .iterate = true }) catch |err| {
            std.log.err("Could not open maps directory: assets/maps ({})", .{err});
            return err;
        };
        defer dir.close();
        
        var loaded_count: u32 = 0;
        var iterator = dir.iterate();
        
        while (try iterator.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".json")) continue;
            
            // Extract name without extension
            const name_end = std.mem.lastIndexOf(u8, entry.name, ".") orelse entry.name.len;
            const map_name = try allocator.dupe(u8, entry.name[0..name_end]);
            
            // Build full path
            const full_path = try std.fmt.allocPrint(allocator, "assets/maps/{s}", .{entry.name});
            defer allocator.free(full_path);
            
            // Load map file
            if (registry.load(full_path, map_name)) |map_data| {
                try registry.maps.put(map_name, map_data);
                loaded_count += 1;
                std.log.info("Loaded map: {s}", .{map_name});
            } else |err| {
                std.log.warn("Failed to load map {s}: {}", .{ entry.name, err });
                allocator.free(map_name);
            }
        }
        
        if (loaded_count == 0) {
            std.log.warn("No JSON map files found in assets/maps", .{});
        } else {
            std.log.info("Map registry loaded {} maps from assets/maps", .{loaded_count});
        }
        
        return registry;
    }
    
    pub fn deinit(self: *Registry) void {
        var iterator = self.maps.iterator();
        while (iterator.next()) |entry| {
            self.allocator.free(entry.key_ptr.*);
            self.allocator.free(entry.value_ptr.brushes);
        }
        self.maps.deinit();
    }
    
    fn load(self: *Registry, path: []const u8, name: []const u8) !Data {
        _ = name;
        
        // Read file
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();
        
        const file_size = try file.getEndPos();
        const contents = try self.allocator.alloc(u8, file_size);
        defer self.allocator.free(contents);
        _ = try file.readAll(contents);
        
        // Parse JSON
        const parsed = try std.json.parseFromSlice(MapJson, self.allocator, contents, .{});
        defer parsed.deinit();
        
        // Convert JSON format to internal format
        const brush_data = try self.allocator.alloc(BrushData, parsed.value.brushes.len);
        
        for (parsed.value.brushes, 0..) |json_brush, i| {
            if (std.mem.eql(u8, json_brush.type, "box")) {
                if (json_brush.size == null) return error.InvalidMapFormat;
                brush_data[i] = .{ .box = .{
                    .position = json_brush.position,
                    .size = json_brush.size.?,
                }};
            } else if (std.mem.eql(u8, json_brush.type, "slope")) {
                if (json_brush.width == null or json_brush.height == null or json_brush.angle == null) {
                    return error.InvalidMapFormat;
                }
                brush_data[i] = .{ .slope = .{
                    .position = json_brush.position,
                    .width = json_brush.width.?,
                    .height = json_brush.height.?,
                    .angle = json_brush.angle.?,
                }};
            } else {
                return error.UnsupportedBrushType;
            }
        }
        
        return Data{
            .name = parsed.value.name,
            .spawn_position = parsed.value.spawn_position,
            .brushes = brush_data,
        };
    }
    
    pub fn get(self: *const Registry, map_name: []const u8) ?*const Data {
        return self.maps.getPtr(map_name);
    }
};