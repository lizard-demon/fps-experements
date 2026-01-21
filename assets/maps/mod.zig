const std = @import("std");
const math = @import("math");

const Vec3 = math.Vec3;

pub const BrushData = struct {
    // Use owned strings to avoid lifetime issues
    texture: []u8, // Owned string
    data: union(enum) {
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
    },
    
    pub fn deinit(self: *BrushData, allocator: std.mem.Allocator) void {
        allocator.free(self.texture);
    }
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
    maps: std.ArrayListUnmanaged(Data) = .{},
    
    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator };
    }
    
    pub fn deinit(self: *Registry) void {
        for (self.maps.items) |*map_data| {
            for (map_data.brushes) |*brush| {
                brush.deinit(self.allocator);
            }
            self.allocator.free(map_data.brushes);
        }
        self.maps.deinit(self.allocator);
    }
    
    pub fn load(self: *Registry, path: []const u8, name: []const u8) !*Data {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();
        
        const file_size = try file.getEndPos();
        const contents = try self.allocator.alloc(u8, file_size);
        defer self.allocator.free(contents);
        _ = try file.readAll(contents);
        
        // Parse JSON
        const MapJson = struct {
            name: []const u8,
            spawn_position: [3]f32,
            brushes: []struct {
                type: []const u8,
                position: [3]f32,
                size: ?[3]f32 = null,
                width: ?f32 = null,
                height: ?f32 = null,
                angle: ?f32 = null,
                texture: ?[]const u8 = null, // Optional texture field
            },
        };
        
        const parsed = try std.json.parseFromSlice(MapJson, self.allocator, contents, .{});
        defer parsed.deinit();
        
        // Convert to internal format
        const brush_data = try self.allocator.alloc(BrushData, parsed.value.brushes.len);
        
        for (parsed.value.brushes, 0..) |json_brush, i| {
            const texture_name = json_brush.texture orelse "concrete";
            const owned_texture = try self.allocator.dupe(u8, texture_name);
            
            if (std.mem.eql(u8, json_brush.type, "box")) {
                brush_data[i] = .{ 
                    .texture = owned_texture,
                    .data = .{ .box = .{
                        .position = json_brush.position,
                        .size = json_brush.size.?,
                    }},
                };
            } else if (std.mem.eql(u8, json_brush.type, "slope")) {
                brush_data[i] = .{ 
                    .texture = owned_texture,
                    .data = .{ .slope = .{
                        .position = json_brush.position,
                        .width = json_brush.width.?,
                        .height = json_brush.height.?,
                        .angle = json_brush.angle.?,
                    }},
                };
            }
        }
        
        const map_data = Data{
            .name = name,
            .spawn_position = parsed.value.spawn_position,
            .brushes = brush_data,
        };
        
        try self.maps.append(self.allocator, map_data);
        return &self.maps.items[self.maps.items.len - 1];
    }
    
    pub fn get(self: *const Registry, map_name: []const u8) ?*const Data {
        for (self.maps.items) |*map_data| {
            if (std.mem.eql(u8, map_data.name, map_name)) {
                return map_data;
            }
        }
        return null;
    }
};