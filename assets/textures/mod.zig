// Texture Module - Registry system
// Usage: const texture = @import("texture");

const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;

const Vec2 = struct {
    x: f32,
    y: f32,
    
    pub fn new(x: f32, y: f32) Vec2 {
        return .{ .x = x, .y = y };
    }
};

pub const Entry = struct {
    uv_offset: Vec2,
    uv_size: Vec2,
    original_size: Vec2,
};

pub const Info = struct {
    name: []const u8,
    pixels: []u32,
    width: u32,
    height: u32,
    // Packing results
    atlas_x: u32 = 0,
    atlas_y: u32 = 0,
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    atlas_texture: sg.Image,
    atlas_view: sg.View,
    atlas_size: u32,
    texture_map: std.StringHashMap(Entry),
    
    pub fn init(allocator: std.mem.Allocator) !Registry {
        return Registry.initFromDirectory(allocator, "assets/textures");
    }
    
    fn initFromDirectory(allocator: std.mem.Allocator, dir_path: []const u8) !Registry {
        var registry = Registry{
            .allocator = allocator,
            .atlas_texture = .{},
            .atlas_view = .{},
            .atlas_size = 0,
            .texture_map = std.StringHashMap(Entry).init(allocator),
        };
        
        // Open directory
        var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch |err| {
            std.log.err("Could not open texture directory: {s} ({})", .{ dir_path, err });
            return err;
        };
        defer dir.close();
        
        // Collect all .rgba files
        var textures = std.ArrayListUnmanaged(Info){};
        defer {
            for (textures.items) |tex| {
                allocator.free(tex.name);
                allocator.free(tex.pixels);
            }
            textures.deinit(allocator);
        }
        
        var iterator = dir.iterate();
        while (try iterator.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".rgba")) continue;
            
            // Load texture file
            if (registry.load(allocator, dir, entry.name)) |texture_info| {
                try textures.append(allocator, texture_info);
                std.log.info("Loaded texture: {s} ({}x{})", .{ texture_info.name, texture_info.width, texture_info.height });
            } else |err| {
                std.log.warn("Failed to load texture {s}: {}", .{ entry.name, err });
            }
        }
        
        if (textures.items.len == 0) {
            std.log.err("No valid textures found in {s}", .{dir_path});
            return error.NoTexturesFound;
        }
        
        // Pack textures into atlas
        try registry.pack(textures.items);
        
        std.log.info("Created texture atlas: {}x{} with {} textures", .{ registry.atlas_size, registry.atlas_size, textures.items.len });
        return registry;
    }
    
    pub fn deinit(self: *Registry) void {
        // Free all stored names
        var iterator = self.texture_map.iterator();
        while (iterator.next()) |entry| {
            self.allocator.free(entry.key_ptr.*);
        }
        self.texture_map.deinit();
    }
    
    fn load(self: *Registry, allocator: std.mem.Allocator, dir: std.fs.Dir, filename: []const u8) !Info {
        _ = self;
        const file = try dir.openFile(filename, .{});
        defer file.close();
        
        const file_size = try file.getEndPos();
        
        // RGBA files should have dimensions that make sense
        // Try common sizes and see which one fits
        const possible_sizes = [_]u32{ 16, 32, 64, 100, 128, 256, 512, 1024 };
        
        for (possible_sizes) |size| {
            const expected_size = size * size * 4; // 4 bytes per pixel (RGBA)
            if (file_size == expected_size) {
                // Found matching size, load the texture
                const pixels = try allocator.alloc(u32, size * size);
                try file.seekTo(0);
                const bytes_read = try file.readAll(std.mem.sliceAsBytes(pixels));
                if (bytes_read != expected_size) {
                    allocator.free(pixels);
                    return error.InvalidFileSize;
                }
                
                // Extract name without extension
                const name_end = std.mem.lastIndexOf(u8, filename, ".") orelse filename.len;
                const name = try allocator.dupe(u8, filename[0..name_end]);
                
                return Info{
                    .name = name,
                    .pixels = pixels,
                    .width = size,
                    .height = size,
                };
            }
        }
        
        return error.UnsupportedTextureSize;
    }
    
    fn pack(self: *Registry, textures: []Info) !void {
        if (textures.len == 0) return;
        
        // Calculate required atlas size
        var total_area: u32 = 0;
        var max_dimension: u32 = 0;
        
        for (textures) |tex| {
            total_area += tex.width * tex.height;
            max_dimension = @max(max_dimension, @max(tex.width, tex.height));
        }
        
        // Start with a reasonable atlas size
        var atlas_size: u32 = @max(256, max_dimension);
        while (atlas_size * atlas_size < total_area * 2) { // 2x for packing inefficiency
            atlas_size *= 2;
        }
        atlas_size = @min(atlas_size, 2048); // Cap at 2048x2048
        
        // Simple shelf packing algorithm
        var current_y: u32 = 0;
        var current_x: u32 = 0;
        var row_height: u32 = 0;
        
        for (textures) |*tex| {
            // Check if texture fits in current row
            if (current_x + tex.width > atlas_size) {
                // Start new row
                current_y += row_height;
                current_x = 0;
                row_height = 0;
                
                // Check if we have vertical space
                if (current_y + tex.height > atlas_size) {
                    return error.AtlasTooSmall;
                }
            }
            
            tex.atlas_x = current_x;
            tex.atlas_y = current_y;
            current_x += tex.width;
            row_height = @max(row_height, tex.height);
        }
        
        // Create atlas texture
        var atlas_pixels = try self.allocator.alloc(u32, atlas_size * atlas_size);
        defer self.allocator.free(atlas_pixels);
        
        // Clear atlas
        @memset(atlas_pixels, 0xFF000000); // Black background
        
        // Copy textures into atlas
        for (textures) |tex| {
            for (0..tex.height) |y| {
                for (0..tex.width) |x| {
                    const src_idx = y * tex.width + x;
                    const dst_idx = (tex.atlas_y + y) * atlas_size + (tex.atlas_x + x);
                    atlas_pixels[dst_idx] = tex.pixels[src_idx];
                }
            }
            
            // Add to texture map
            const uv_x = @as(f32, @floatFromInt(tex.atlas_x)) / @as(f32, @floatFromInt(atlas_size));
            const uv_y = @as(f32, @floatFromInt(tex.atlas_y)) / @as(f32, @floatFromInt(atlas_size));
            const uv_w = @as(f32, @floatFromInt(tex.width)) / @as(f32, @floatFromInt(atlas_size));
            const uv_h = @as(f32, @floatFromInt(tex.height)) / @as(f32, @floatFromInt(atlas_size));
            
            const owned_name = try self.allocator.dupe(u8, tex.name);
            try self.texture_map.put(owned_name, .{
                .uv_offset = Vec2.new(uv_x, uv_y),
                .uv_size = Vec2.new(uv_w, uv_h),
                .original_size = Vec2.new(@floatFromInt(tex.width), @floatFromInt(tex.height)),
            });
        }
        
        // Create GPU texture
        self.atlas_texture = sg.makeImage(.{
            .width = @intCast(atlas_size),
            .height = @intCast(atlas_size),
            .data = init: {
                var data = sg.ImageData{};
                data.mip_levels[0] = sg.asRange(atlas_pixels);
                break :init data;
            },
        });
        
        self.atlas_view = sg.makeView(.{ .texture = .{ .image = self.atlas_texture } });
        self.atlas_size = atlas_size;
    }
    
    pub fn getAtlasUV(self: *const Registry, texture_name: []const u8) [2]f32 {
        if (self.texture_map.get(texture_name)) |entry| {
            return .{ entry.uv_offset.x, entry.uv_offset.y };
        }
        
        std.log.warn("Texture '{s}' not found, using first available texture", .{texture_name});
        
        // Return first texture as fallback
        var iterator = self.texture_map.iterator();
        if (iterator.next()) |first_entry| {
            return .{ first_entry.value_ptr.uv_offset.x, first_entry.value_ptr.uv_offset.y };
        }
        
        std.log.err("No textures available at all!", .{});
        return .{ 0.0, 0.0 };
    }
};