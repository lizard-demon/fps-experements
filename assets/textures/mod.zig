const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;

pub const Registry = struct {
    allocator: std.mem.Allocator,
    atlas_texture: sg.Image = .{},
    atlas_view: sg.View = .{},
    atlas_size: u32 = 0,
    texture_map: std.StringHashMap([2]f32),
    
    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{
            .allocator = allocator,
            .texture_map = std.StringHashMap([2]f32).init(allocator),
        };
    }
    
    pub fn deinit(self: *Registry) void {
        var iterator = self.texture_map.iterator();
        while (iterator.next()) |entry| {
            self.allocator.free(entry.key_ptr.*);
        }
        self.texture_map.deinit();
    }
    
    pub fn loadDirectory(self: *Registry, dir_path: []const u8) !void {
        var dir = try std.fs.cwd().openDir(dir_path, .{ .iterate = true });
        defer dir.close();
        
        // Collect textures
        var textures = std.ArrayListUnmanaged(struct {
            name: []const u8,
            pixels: []u32,
            width: u32,
            height: u32,
            atlas_x: u32 = 0,
            atlas_y: u32 = 0,
        }){};
        defer {
            for (textures.items) |tex| {
                self.allocator.free(tex.name);
                self.allocator.free(tex.pixels);
            }
            textures.deinit(self.allocator);
        }
        
        var iterator = dir.iterate();
        while (try iterator.next()) |entry| {
            if (entry.kind != .file or !std.mem.endsWith(u8, entry.name, ".rgba")) continue;
            
            const file = dir.openFile(entry.name, .{}) catch continue;
            defer file.close();
            
            const file_size = file.getEndPos() catch continue;
            
            // Try common texture sizes
            const sizes = [_]u32{ 16, 32, 64, 100, 128, 256, 512, 1024 };
            for (sizes) |size| {
                if (file_size == size * size * 4) {
                    const pixels = self.allocator.alloc(u32, size * size) catch continue;
                    file.seekTo(0) catch continue;
                    if ((file.readAll(std.mem.sliceAsBytes(pixels)) catch 0) == file_size) {
                        const name_end = std.mem.lastIndexOf(u8, entry.name, ".") orelse entry.name.len;
                        const name = self.allocator.dupe(u8, entry.name[0..name_end]) catch continue;
                        try textures.append(self.allocator, .{
                            .name = name,
                            .pixels = pixels,
                            .width = size,
                            .height = size,
                        });
                        break;
                    }
                    self.allocator.free(pixels);
                }
            }
        }
        
        if (textures.items.len == 0) return error.NoTexturesFound;
        
        // Pack into atlas
        var atlas_size: u32 = 256;
        var total_area: u32 = 0;
        for (textures.items) |tex| total_area += tex.width * tex.height;
        while (atlas_size * atlas_size < total_area * 2) atlas_size *= 2;
        atlas_size = @min(atlas_size, 2048);
        
        // Simple shelf packing
        var current_y: u32 = 0;
        var current_x: u32 = 0;
        var row_height: u32 = 0;
        
        for (textures.items) |*tex| {
            if (current_x + tex.width > atlas_size) {
                current_y += row_height;
                current_x = 0;
                row_height = 0;
            }
            tex.atlas_x = current_x;
            tex.atlas_y = current_y;
            current_x += tex.width;
            row_height = @max(row_height, tex.height);
        }
        
        // Create atlas
        var atlas_pixels = try self.allocator.alloc(u32, atlas_size * atlas_size);
        defer self.allocator.free(atlas_pixels);
        @memset(atlas_pixels, 0xFF000000);
        
        for (textures.items) |tex| {
            for (0..tex.height) |y| {
                for (0..tex.width) |x| {
                    const src_idx = y * tex.width + x;
                    const dst_idx = (tex.atlas_y + y) * atlas_size + (tex.atlas_x + x);
                    atlas_pixels[dst_idx] = tex.pixels[src_idx];
                }
            }
            
            // Store UV coordinates
            const uv_x = @as(f32, @floatFromInt(tex.atlas_x)) / @as(f32, @floatFromInt(atlas_size));
            const uv_y = @as(f32, @floatFromInt(tex.atlas_y)) / @as(f32, @floatFromInt(atlas_size));
            const owned_name = try self.allocator.dupe(u8, tex.name);
            try self.texture_map.put(owned_name, .{ uv_x, uv_y });
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
        if (self.texture_map.get(texture_name)) |uv| {
            return uv;
        }
        
        // Return first texture as fallback
        var iterator = self.texture_map.iterator();
        if (iterator.next()) |first_entry| {
            return first_entry.value_ptr.*;
        }
        
        return .{ 0.0, 0.0 };
    }
};