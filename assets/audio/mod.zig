// Audio Module - Mixer-Registry system
// Usage: const audio = @import("audio");

const std = @import("std");

pub const Options = struct {
    loop: bool = false,
    volume: f32 = 1.0,
};

pub const Clip = struct {
    name: []const u8,
    samples: []f32,
    sample_rate: u32,
};

pub const Voice = struct {
    clip: ?*const Clip = null,
    position: f32 = 0,
    looping: bool = false,
    volume: f32 = 1.0,
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    clips: std.ArrayListUnmanaged(Clip) = .{},
    clip_map: std.StringHashMap(usize) = undefined,
    voices: [16]Voice = [_]Voice{.{}} ** 16,
    
    pub fn init(allocator: std.mem.Allocator) !Registry {
        return Registry.initFromDirectory(allocator, "assets/audio", 44100);
    }
    
    fn initFromDirectory(allocator: std.mem.Allocator, dir_path: []const u8, sample_rate: u32) !Registry {
        var registry = Registry{
            .allocator = allocator,
            .clip_map = std.StringHashMap(usize).init(allocator),
        };
        
        // Open directory
        var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch |err| {
            std.log.err("Could not open audio directory: {s} ({})", .{ dir_path, err });
            return err;
        };
        defer dir.close();
        
        var loaded_count: u32 = 0;
        var iterator = dir.iterate();
        
        while (try iterator.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".pcm")) continue;
            
            // Extract name without extension
            const name_end = std.mem.lastIndexOf(u8, entry.name, ".") orelse entry.name.len;
            const clip_name = try allocator.dupe(u8, entry.name[0..name_end]);
            
            // Build full path
            const full_path = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ dir_path, entry.name });
            defer allocator.free(full_path);
            
            // Load audio file
            if (registry.load(full_path, clip_name, sample_rate)) {
                loaded_count += 1;
                std.log.info("Loaded audio clip: {s}", .{clip_name});
            } else |err| {
                std.log.warn("Failed to load audio {s}: {}", .{ entry.name, err });
                allocator.free(clip_name);
            }
        }
        
        if (loaded_count == 0) {
            std.log.warn("No PCM files found in {s}", .{dir_path});
        } else {
            std.log.info("Audio registry loaded {} clips from {s}", .{ loaded_count, dir_path });
        }
        
        return registry;
    }
    
    pub fn deinit(self: *Registry) void {
        // Free clip data
        for (self.clips.items) |clip| {
            self.allocator.free(clip.name);
            self.allocator.free(clip.samples);
        }
        self.clips.deinit(self.allocator);
        self.clip_map.deinit();
    }
    
    fn load(self: *Registry, path: []const u8, name: []const u8, sample_rate: u32) !void {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();
        
        const file_size = try file.getEndPos();
        const num_samples = file_size / 2; // 16-bit = 2 bytes per sample
        const samples = try self.allocator.alloc(f32, num_samples);
        
        // Read and convert samples directly from raw PCM
        for (0..num_samples) |i| {
            var sample_bytes: [2]u8 = undefined;
            _ = try file.readAll(&sample_bytes);
            const sample_i16 = std.mem.readInt(i16, &sample_bytes, .little);
            samples[i] = @as(f32, @floatFromInt(sample_i16)) / 32768.0;
        }
        
        const clip = Clip{
            .name = name,
            .samples = samples,
            .sample_rate = sample_rate,
        };
        
        try self.clips.append(self.allocator, clip);
        const clip_index = self.clips.items.len - 1;
        try self.clip_map.put(name, clip_index);
    }
    
    pub fn play(self: *Registry, sound_name: []const u8, options: Options) void {
        const clip_index = self.clip_map.get(sound_name) orelse {
            std.log.warn("Audio clip '{s}' not found", .{sound_name});
            return;
        };
        
        // Find free voice
        for (&self.voices) |*voice| {
            if (voice.clip == null) {
                voice.* = Voice{
                    .clip = &self.clips.items[clip_index],
                    .position = 0,
                    .looping = options.loop,
                    .volume = options.volume,
                };
                return;
            }
        }
        
        std.log.warn("No free voice slots available for '{s}'", .{sound_name});
    }
    
    pub fn stop(self: *Registry, sound_name: []const u8) void {
        const clip_index = self.clip_map.get(sound_name) orelse return;
        const target_clip = &self.clips.items[clip_index];
        
        for (&self.voices) |*voice| {
            if (voice.clip == target_clip) {
                voice.clip = null;
            }
        }
    }
    
    pub fn stopAll(self: *Registry) void {
        for (&self.voices) |*voice| {
            voice.clip = null;
        }
    }
    
    pub fn sample(self: *Registry, target_sample_rate: f32) f32 {
        var mixed_sample: f32 = 0;
        
        for (&self.voices) |*voice| {
            if (voice.clip == null) continue;
            
            const clip = voice.clip.?;
            const pos_int = @as(usize, @intFromFloat(voice.position));
            
            if (pos_int >= clip.samples.len) {
                if (voice.looping) {
                    voice.position = 0;
                } else {
                    voice.clip = null;
                    continue;
                }
            }
            
            const sample_pos = @as(usize, @intFromFloat(voice.position));
            if (sample_pos < clip.samples.len) {
                mixed_sample += clip.samples[sample_pos] * voice.volume;
            }
            
            // Advance position
            const rate_ratio = @as(f32, @floatFromInt(clip.sample_rate)) / target_sample_rate;
            voice.position += rate_ratio;
        }
        
        return std.math.clamp(mixed_sample, -1.0, 1.0);
    }
};