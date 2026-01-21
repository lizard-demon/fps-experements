const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const saudio = sokol.audio;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("math");
const bvh = @import("lib/bvh.zig");
const brush = @import("lib/brush.zig");
const audio_lib = @import("audio");
const map_lib = @import("maps");
const models_lib = @import("models");
const physics_mod = @import("resources/physics.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = math.AABB;

// Module types
const Physics = physics_mod.Physics(.{});
const render_mod = @import("resources/render.zig");
const Renderer = render_mod.Renderer(.{});
const World = physics_mod.World;
const Map = struct {
    data: *const map_lib.Data,
    world: World,
    brushes: []brush.Brush,
    plane_normals: []Vec3,
    plane_distances: []f32,
    allocator: std.mem.Allocator,
    
    const MAX_BRUSHES = 32;
    const MAX_PLANES = 256;
    
    pub fn loadFromMapData(allocator: std.mem.Allocator, map_data: *const map_lib.Data) !Map {
        if (map_data.brushes.len > MAX_BRUSHES) {
            return error.TooManyBrushes;
        }
        
        // Allocate storage
        const brushes = try allocator.alloc(brush.Brush, map_data.brushes.len);
        const plane_normals = try allocator.alloc(Vec3, MAX_PLANES);
        const plane_distances = try allocator.alloc(f32, MAX_PLANES);
        
        var plane_idx: usize = 0;
        
        // Convert brush data to actual brushes
        for (map_data.brushes, 0..) |brush_data, i| {
            const planes_needed: usize = 6; // Both box and slope use 6 planes
            
            if (plane_idx + planes_needed > MAX_PLANES) {
                allocator.free(brushes);
                allocator.free(plane_normals);
                allocator.free(plane_distances);
                return error.TooManyPlanes;
            }
            
            switch (brush_data) {
                .box => |box| {
                    createBoxBrush(
                        brushes[i..i+1],
                        plane_normals[plane_idx..plane_idx + 6],
                        plane_distances[plane_idx..plane_idx + 6],
                        Vec3.new(box.position[0], box.position[1], box.position[2]),
                        Vec3.new(box.size[0], box.size[1], box.size[2])
                    );
                    plane_idx += 6;
                },
                .slope => |slope| {
                    createSlopeBrush(
                        brushes[i..i+1],
                        plane_normals[plane_idx..plane_idx + 6],
                        plane_distances[plane_idx..plane_idx + 6],
                        Vec3.new(slope.position[0], slope.position[1], slope.position[2]),
                        slope.width,
                        slope.height,
                        slope.angle
                    );
                    plane_idx += 6;
                },
            }
        }
        
        // Create world
        const world = World.init(brushes, allocator) catch |err| {
            allocator.free(brushes);
            allocator.free(plane_normals);
            allocator.free(plane_distances);
            return err;
        };
        
        return Map{
            .data = map_data,
            .world = world,
            .brushes = brushes,
            .plane_normals = plane_normals,
            .plane_distances = plane_distances,
            .allocator = allocator,
        };
    }
    
    pub fn getSpawnPosition(self: *const Map) Vec3 {
        return self.data.getSpawnPosition();
    }
    
    pub fn deinit(self: *Map) void {
        self.world.deinit();
        self.allocator.free(self.brushes);
        self.allocator.free(self.plane_normals);
        self.allocator.free(self.plane_distances);
    }
    
    fn createBoxBrush(
        brushes: []brush.Brush,
        normals: []Vec3,
        distances: []f32,
        center: Vec3,
        size: Vec3
    ) void {
        const half_size = Vec3.scale(size, 0.5);
        
        const box_normals = [6]Vec3{
            Vec3.new( 1,  0,  0), Vec3.new(-1,  0,  0),
            Vec3.new( 0,  1,  0), Vec3.new( 0, -1,  0),
            Vec3.new( 0,  0,  1), Vec3.new( 0,  0, -1),
        };
        const box_distances = [6]f32{
            -(center.data[0] + half_size.data[0]), center.data[0] - half_size.data[0],
            -(center.data[1] + half_size.data[1]), center.data[1] - half_size.data[1],
            -(center.data[2] + half_size.data[2]), center.data[2] - half_size.data[2],
        };
        
        @memcpy(normals[0..6], &box_normals);
        @memcpy(distances[0..6], &box_distances);
        
        brushes[0] = .{
            .planes = .{
                .normals = normals[0..6],
                .distances = distances[0..6],
            },
            .bounds = AABB.new(Vec3.sub(center, half_size), Vec3.add(center, half_size))
        };
    }
    
    fn createSlopeBrush(
        brushes: []brush.Brush,
        normals: []Vec3,
        distances: []f32,
        center: Vec3,
        width: f32,
        height: f32,
        angle_degrees: f32
    ) void {
        const angle = angle_degrees * (std.math.pi / 180.0);
        const slope_normal = Vec3.normalize(Vec3.new(0, @cos(angle), -@sin(angle)));
        const slope_point = Vec3.new(0, height, center.data[2] + width/2);
        
        const slope_normals = [6]Vec3{
            Vec3.new( 0, -1,  0), Vec3.new(-1,  0,  0), Vec3.new( 1,  0,  0),
            Vec3.new( 0,  0, -1), Vec3.new( 0,  0,  1), slope_normal,
        };
        const slope_distances = [6]f32{
            0.0, -width/2, -width/2,
            center.data[2] - width/2, -(center.data[2] + width/2),
            -Vec3.dot(slope_normal, slope_point),
        };
        
        @memcpy(normals[0..6], &slope_normals);
        @memcpy(distances[0..6], &slope_distances);
        
        brushes[0] = .{
            .planes = .{
                .normals = normals[0..6],
                .distances = distances[0..6],
            },
            .bounds = AABB.new(
                Vec3.new(-width/2, 0.0, center.data[2] - width/2),
                Vec3.new(width/2, height, center.data[2] + width/2)
            )
        };
    }
};

// Game state
const Game = struct {
    allocator: std.mem.Allocator,
    delta_time: f32 = 0,
    mouse_locked: bool = false,
    
    // Input state
    keys: packed struct { 
        w: bool = false, 
        a: bool = false, 
        s: bool = false, 
        d: bool = false, 
        sp: bool = false 
    } = .{},
    mouse_dx: f32 = 0,
    mouse_dy: f32 = 0,
    
    // Systems
    physics: Physics = undefined,
    renderer: Renderer = undefined,
    
    // World data
    map: Map = undefined,
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.physics = Physics.init(&registry);
        
        // Initialize renderer with better error handling
        self.renderer = Renderer.init(allocator, &model_registry) catch |err| {
            std.log.err("Failed to initialize renderer: {}", .{err});
            switch (err) {
                error.NoTexturesFound => {
                    std.log.err("Make sure you have texture files in assets/textures/", .{});
                    std.log.err("Run: ./tools/formatting/textures.sh to convert PNG files", .{});
                },
                else => {},
            }
            // Create a minimal fallback renderer or exit gracefully
            std.log.err("Cannot continue without renderer. Exiting.", .{});
            std.process.exit(1);
        };
        
        // Load map from registry
        if (map_registry.get("test")) |map_data| {
            self.map = Map.loadFromMapData(allocator, map_data) catch |err| {
                std.log.err("Failed to create map from data: {}", .{err});
                self.createFallbackMap(allocator);
                return;
            };
        } else {
            std.log.err("Map 'test' not found in registry", .{});
            self.createFallbackMap(allocator);
            return;
        }
        
        std.log.info("Loaded map: {s}", .{self.map.data.name});
        std.log.info("Created {} brushes", .{self.map.brushes.len});
        for (self.map.brushes, 0..) |b, i| {
            std.log.info("  Brush {}: {} planes, bounds: ({d:.1},{d:.1},{d:.1}) to ({d:.1},{d:.1},{d:.1})", .{
                i, b.planes.len(),
                b.bounds.min.data[0], b.bounds.min.data[1], b.bounds.min.data[2],
                b.bounds.max.data[0], b.bounds.max.data[1], b.bounds.max.data[2]
            });
        }
    }
    
    fn createFallbackMap(self: *@This(), allocator: std.mem.Allocator) void {
        // Create a simple ground plane as fallback
        const fallback_brushes = allocator.alloc(map_lib.BrushData, 1) catch {
            std.log.err("Failed to allocate fallback brushes", .{});
            return;
        };
        defer allocator.free(fallback_brushes);
        
        fallback_brushes[0] = .{ .box = .{
            .position = .{ 0.0, -2.25, 0.0 },
            .size = .{ 100.0, 4.5, 100.0 }
        }};
        
        const fallback_data = map_lib.Data{
            .name = "Fallback Map",
            .spawn_position = .{ 0.0, 3.0, 0.0 },
            .brushes = fallback_brushes,
        };
        
        self.map = Map.loadFromMapData(allocator, &fallback_data) catch |err| {
            std.log.err("Failed to create fallback map: {}", .{err});
            return;
        };
    }
    
    fn buildWorldMesh(self: *@This()) !void {
        // Use the actual texture file we have: test.rgba
        const texture_name = "test";
        
        // Add brushes to renderer with the test texture
        for (self.map.brushes) |b| {
            try self.renderer.addBrush(b, .{ 0.8, 0.8, 0.8, 1.0 }, texture_name);
        }
        
        // Add model with the same texture - handle missing file gracefully
        const model_position = Vec3.new(0.0, 5.0, -15.0);
        const model_color = [4]f32{ 1.0, 1.0, 1.0, 1.0 };
        self.renderer.addObjModelAutoScale("cube", model_color, model_position, 3.0, texture_name) catch |err| {
            std.log.warn("Failed to load cube.obj: {} (continuing without model)", .{err});
        };
        
        try self.renderer.buildBuffers();
    }
    
    fn deinit(self: *@This()) void {
        self.map.deinit();
        self.renderer.deinit();
    }
};

// Configuration constants
const MOUSE_SENSITIVITY: f32 = 0.002;
const PITCH_LIMIT: f32 = 1.5;
const EYE_HEIGHT: f32 = 0.6;

// Global state
var game: Game = undefined;
var initialized: bool = false;
var registry: audio_lib.Registry = undefined;
var map_registry: map_lib.Registry = undefined;
var model_registry: models_lib.Registry = undefined;

export fn init() void {
    const allocator = std.heap.c_allocator;
    sg.setup(.{ .environment = sokol.glue.environment() });
    saudio.setup(.{ .stream_cb = audio });
    simgui.setup(.{});
    
    registry = audio_lib.Registry.init(allocator) catch |err| blk: {
        std.log.err("Failed to load audio registry: {}", .{err});
        break :blk audio_lib.Registry{
            .allocator = allocator,
            .clip_map = std.StringHashMap(usize).init(allocator),
        };
    };
    
    map_registry = map_lib.Registry.init(allocator) catch |err| blk: {
        std.log.err("Failed to load map registry: {}", .{err});
        break :blk map_lib.Registry{
            .allocator = allocator,
            .maps = std.StringHashMap(map_lib.Data).init(allocator),
        };
    };
    
    model_registry = models_lib.Registry.init(allocator) catch |err| blk: {
        std.log.err("Failed to load model registry: {}", .{err});
        break :blk models_lib.Registry{
            .allocator = allocator,
            .meshes = std.StringHashMap(models_lib.Mesh).init(allocator),
        };
    };
    
    // Start background music
    registry.play("music", .{ .loop = true, .volume = 0.3 });
    
    game = .{ .allocator = allocator };
    game.init(allocator);
    
    game.buildWorldMesh() catch |err| {
        std.log.err("Failed to build world mesh: {}", .{err});
    };
    
    // Set initial player position from map
    game.physics.state.pos = game.map.getSpawnPosition();
    
    initialized = true;
}

export fn frame() void {
    game.delta_time = @as(f32, @floatCast(sapp.frameDuration()));
    
    // Handle mouse input
    game.physics.state.yaw += game.mouse_dx * MOUSE_SENSITIVITY;
    game.physics.state.pitch = std.math.clamp(game.physics.state.pitch + game.mouse_dy * MOUSE_SENSITIVITY, -PITCH_LIMIT, PITCH_LIMIT);
    game.mouse_dx = 0;
    game.mouse_dy = 0;
    
    // Calculate input direction
    const fwd: f32 = if (game.keys.w) 1 else if (game.keys.s) -1 else 0;
    const right: f32 = if (game.keys.d) 1 else if (game.keys.a) -1 else 0;
    const wish_dir = math.wishdir(fwd, right, game.physics.state.yaw);
    
    // Update physics
    game.physics.update(&game.map.world, wish_dir, game.keys.sp, game.delta_time);
    
    // Render
    simgui.newFrame(.{ .width = sapp.width(), .height = sapp.height(), .delta_time = sapp.frameDuration() });
    
    const view = math.Mat4.viewMatrix(game.physics.state.pos, game.physics.state.yaw, game.physics.state.pitch, EYE_HEIGHT);
    game.renderer.render(view);
    game.renderer.renderCrosshair();
    
    simgui.render(); 
    sg.endPass(); 
    sg.commit();
}

export fn cleanup() void {
    initialized = false;
    saudio.shutdown();
    
    // Stop all audio
    registry.stopAll();
    
    game.deinit();
    registry.deinit();
    map_registry.deinit();
    model_registry.deinit();
    simgui.shutdown(); 
    sg.shutdown();
}

export fn event(e: [*c]const sapp.Event) void {
    if (e == null) return;
    _ = simgui.handleEvent(e.*);
    
    if (!initialized) return;
    
    const d = e.*.type == .KEY_DOWN;
    
    switch (e.*.type) {
        .KEY_DOWN, .KEY_UP => switch (e.*.key_code) {
            .W => game.keys.w = d,
            .A => game.keys.a = d,
            .S => game.keys.s = d,
            .D => game.keys.d = d,
            .SPACE => game.keys.sp = d,
            .ESCAPE => {
                if (d and game.mouse_locked) {
                    game.mouse_locked = false;
                    sapp.showMouse(true);
                    sapp.lockMouse(false);
                }
            },
            else => {},
        },
        .MOUSE_DOWN => if (e.*.mouse_button == .LEFT and !game.mouse_locked) {
            game.mouse_locked = true;
            sapp.showMouse(false);
            sapp.lockMouse(true);
        },
        .MOUSE_MOVE => {
            if (game.mouse_locked) {
                game.mouse_dx += e.*.mouse_dx;
                game.mouse_dy += e.*.mouse_dy;
            }
        },
        else => {},
    }
}

fn audio(buf: [*c]f32, n: i32, c: i32) callconv(.c) void {
    if (n <= 0 or c <= 0) return;
    
    const n_usize: usize = @intCast(n);
    const c_usize: usize = @intCast(c);
    const total_samples = std.math.mul(usize, n_usize, c_usize) catch return;
    
    if (!initialized) { 
        for (0..total_samples) |i| buf[i] = 0; 
        return; 
    }
    
    for (0..n_usize) |f| {
        const sample = registry.sample(44100.0);
        for (0..c_usize) |ch| buf[f * c_usize + ch] = sample;
    }
}

pub fn main() void {
    sapp.run(.{ 
        .init_cb = init, 
        .frame_cb = frame, 
        .cleanup_cb = cleanup, 
        .event_cb = event, 
        .width = 1024, 
        .height = 768, 
        .window_title = "FPS" 
    });
}