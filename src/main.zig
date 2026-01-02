const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const saudio = sokol.audio;
const simgui = sokol.imgui;
const ig = @import("cimgui");
const math = @import("lib/math.zig");
const shader = @import("shader/cube.glsl.zig");
const ecs = @import("lib/ecs.zig");
const bvh = @import("lib/bvh.zig");
const movement = @import("lib/physics.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = bvh.AABB;

const Vertex = extern struct { pos: [3]f32, col: [4]f32 };
const Brush = struct { 
    box: AABB, 
    col: [4]f32,
    
    fn get_aabb(self: Brush) AABB {
        return self.box;
    }
};
const Node = struct { box: AABB, left: u32 = 0, right: u32 = 0, first: u32 = 0, count: u32 = 0 };

const Physics = struct { 
    pos: Vec3, 
    vel: Vec3,
    on_ground: bool = false,
};
const Input = struct { yaw: f32 = 0, pitch: f32 = 0, mdx: f32 = 0, mdy: f32 = 0, keys: packed struct { w: bool=false, a: bool=false, s: bool=false, d: bool=false, sp: bool=false } = .{}, lock: bool = false };
const Player = struct { physics: Physics, input: Input };
const Entities = struct { players: ecs.List(Player) };

const World = struct {
    brushes: std.ArrayListUnmanaged(Brush) = .{},
    collision_bvh: ?bvh.BVH(Brush, Brush.get_aabb) = null,
    verts: std.ArrayListUnmanaged(Vertex) = .{}, 
    inds: std.ArrayListUnmanaged(u16) = .{}, 
    mem: std.mem.Allocator,

    fn init(mem: std.mem.Allocator) World { return .{ .mem = mem }; }
    
    fn deinit(w: *World) void { 
        w.brushes.deinit(w.mem); 
        if (w.collision_bvh) |*bvh_tree| bvh_tree.deinit();
        w.verts.deinit(w.mem); 
        w.inds.deinit(w.mem); 
    }
    
    fn add(w: *World, pos: Vec3, size: Vec3, col: [4]f32) !void { 
        const h = Vec3.scale(size, 0.5); 
        try w.brushes.append(w.mem, .{ 
            .box = .{ .min = Vec3.sub(pos, h), .max = Vec3.add(pos, h) }, 
            .col = col 
        }); 
    }

    fn build(w: *World) !void {
        if (w.collision_bvh) |*bvh_tree| bvh_tree.deinit();
        if (w.brushes.items.len > 0) {
            w.collision_bvh = try bvh.BVH(Brush, Brush.get_aabb).init(w.mem, w.brushes.items);
        }
    }

    fn hit(w: *const World, box: AABB) bool {
        if (w.collision_bvh) |*bvh_tree| {
            return bvh_tree.intersect(box);
        }
        return false;
    }

    fn bake(w: *World) !void {
        w.verts.clearRetainingCapacity(); 
        w.inds.clearRetainingCapacity();
        
        const v_tab = [_][3]f32{ 
            .{ 1,-1,-1}, .{-1,-1,-1}, .{-1, 1,-1}, .{ 1, 1,-1}, 
            .{-1,-1, 1}, .{ 1,-1, 1}, .{ 1, 1, 1}, .{-1, 1, 1}, 
            .{-1,-1,-1}, .{-1,-1, 1}, .{-1, 1, 1}, .{-1, 1,-1}, 
            .{ 1,-1, 1}, .{ 1,-1,-1}, .{ 1, 1,-1}, .{ 1, 1, 1}, 
            .{-1,-1, 1}, .{-1,-1,-1}, .{ 1,-1,-1}, .{ 1,-1, 1}, 
            .{-1, 1,-1}, .{-1, 1, 1}, .{ 1, 1, 1}, .{ 1, 1,-1} 
        };
        
        for (w.brushes.items) |b| {
            const cen = Vec3.scale(Vec3.add(b.box.min, b.box.max), 0.5);
            const ext = Vec3.scale(Vec3.sub(b.box.max, b.box.min), 0.5);
            const base = @as(u16, @intCast(w.verts.items.len));
            
            for (v_tab) |v| {
                var c = b.col; 
                if (v[1] > 0.01) { 
                    c[0]*=1.1; c[1]*=1.1; c[2]*=1.1; 
                } else if (v[1] < -0.01) { 
                    c[0]*=0.7; c[1]*=0.7; c[2]*=0.7; 
                }
                try w.verts.append(w.mem, .{ 
                    .pos = .{ cen.data[0] + v[0]*ext.data[0], cen.data[1] + v[1]*ext.data[1], cen.data[2] + v[2]*ext.data[2] }, 
                    .col = c 
                });
            }
            
            inline for (0..6) |f| 
                for ([_]u16{0,1,2,0,2,3}) |q| 
                    try w.inds.append(w.mem, @intCast(base + f*4 + q));
        }
    }
};

fn sys_PlayerUpdate(phy: []Physics, inp: []Input) void {
    const dt = @as(f32, @floatCast(sapp.frameDuration()));
    
    for (phy, inp) |*p, *i| {
        // Update camera
        i.yaw += i.mdx * 0.002; 
        i.pitch = std.math.clamp(i.pitch + i.mdy * 0.002, -1.5, 1.5); 
        i.mdx = 0; i.mdy = 0;
        
        // Get input values
        const input_forward: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const input_strafe: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const input_jump = i.keys.sp;
        
        // Collision function for movement system
        const CollisionContext = struct {
            fn check_collision(aabb: bvh.AABB) bool {
                return st.world.hit(aabb);
            }
        };
        
        // Simple Quake movement
        movement.quake_movement(
            &p.pos,
            &p.vel,
            &p.on_ground,
            input_forward,
            input_strafe,
            input_jump,
            i.yaw,
            dt,
            CollisionContext.check_collision
        );
        
        // Play jump sound
        if (input_jump and p.on_ground) {
            st.jmp = true;
        }
    }
}

fn get_view_matrix(phy: Physics, inp: Input) Mat4 {
    const eye = Vec3.add(phy.pos, Vec3.new(0, 0.6, 0)); // Lower camera for better feel
    const cy, const sy = .{ @cos(inp.yaw), @sin(inp.yaw) };
    const cp, const sp = .{ @cos(inp.pitch), @sin(inp.pitch) };
    return .{ .data = .{ cy, sy*sp, -sy*cp, 0, 0, cp, sp, 0, sy, -cy*sp, cy*cp, 0, -eye.data[0]*cy - eye.data[2]*sy, -eye.data[0]*sy*sp - eye.data[1]*cp + eye.data[2]*cy*sp, eye.data[0]*sy*cp - eye.data[1]*sp - eye.data[2]*cy*cp, 1 } };
}

const Renderer = struct {
    pip: sg.Pipeline = undefined, bind: sg.Bindings = undefined, cnt: u32 = 0, pass: sg.PassAction = undefined,
    fn init(r: *Renderer, w: *const World) void {
        var lay = sg.VertexLayoutState{}; lay.attrs[0].format = .FLOAT3; lay.attrs[1].format = .FLOAT4;
        r.pip = sg.makePipeline(.{ .shader = sg.makeShader(shader.cubeShaderDesc(sg.queryBackend())), .layout = lay, .index_type = .UINT16, .depth = .{ .compare = .LESS_EQUAL, .write_enabled = true }, .cull_mode = .FRONT });
        r.pass = .{ .colors = .{ .{ .load_action = .CLEAR, .clear_value = .{ .r = 0.15, .g = 0.15, .b = 0.18, .a = 1.0 } }, .{}, .{}, .{}, .{}, .{}, .{}, .{} } };
        r.upload(w);
    }
    fn upload(r: *Renderer, w: *const World) void {
        if (r.bind.vertex_buffers[0].id != 0) sg.destroyBuffer(r.bind.vertex_buffers[0]);
        if (r.bind.index_buffer.id != 0) sg.destroyBuffer(r.bind.index_buffer);
        if (w.verts.items.len > 0) {
            r.bind.vertex_buffers[0] = sg.makeBuffer(.{ .data = .{ .ptr = w.verts.items.ptr, .size = w.verts.items.len * @sizeOf(Vertex) } });
            r.bind.index_buffer = sg.makeBuffer(.{ .usage = .{ .index_buffer = true }, .data = .{ .ptr = w.inds.items.ptr, .size = w.inds.items.len * @sizeOf(u16) } });
        }
        r.cnt = @intCast(w.inds.items.len);
    }
    fn draw(r: *const Renderer, view: Mat4) void {
        const mvp = Mat4.mul(math.perspective(90, 1.33, 0.1, 200), view);
        sg.beginPass(.{ .action = r.pass, .swapchain = sokol.glue.swapchain() });
        sg.applyPipeline(r.pip); sg.applyBindings(r.bind); sg.applyUniforms(0, sg.asRange(&mvp));
        sg.draw(0, r.cnt, 1);
    }
};

var st = struct { world: World = undefined, ren: Renderer = .{}, jmp: bool = false, t_jmp: f32 = 0, mem: std.mem.Allocator = undefined, store: ecs.Store(Entities) = undefined }{};

export fn init() void {
    st.mem = std.heap.c_allocator;
    sg.setup(.{ .environment = sokol.glue.environment() }); saudio.setup(.{ .stream_cb = audio }); simgui.setup(.{});
    st.world = World.init(st.mem);
    st.store = ecs.Store(Entities){ .registry = .{ .players = .{} } };
    st.store.registry.players.append(st.mem, .{ 
        .physics = .{ .pos = Vec3.new(0, 5, -10), .vel = Vec3.zero() }, 
        .input = .{} 
    }) catch {};
    st.world.add(Vec3.new(0, -1, 0), Vec3.new(60, 2, 60), .{ 0.3, 0.3, 0.35, 1 }) catch {};
    const walls = [_][6]f32{ .{30,5,0,2,10,60}, .{-30,5,0,2,10,60}, .{0,5,30,60,10,2}, .{0,5,-30,60,10,2}, .{0,5,0,4,10,4} };
    for (walls) |d| st.world.add(Vec3.new(d[0], d[1], d[2]), Vec3.new(d[3], d[4], d[5]), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    for (0..4) |i| st.world.add(Vec3.new(10 + @as(f32,@floatFromInt(i)) * 2, 0.5 + @as(f32,@floatFromInt(i)), 10), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    st.world.add(Vec3.new(18, 3.5, 10), Vec3.new(8, 1, 8), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    st.world.build() catch {}; st.world.bake() catch {}; st.ren.init(&st.world);
}

export fn frame() void {
    st.store.run(sys_PlayerUpdate);
    simgui.newFrame(.{ .width = sapp.width(), .height = sapp.height(), .delta_time = sapp.frameDuration() });
    
    if (st.store.registry.players.items(.physics).len > 0) {
        const player_physics = st.store.registry.players.items(.physics)[0];
        const player_input = st.store.registry.players.items(.input)[0];
        
        st.ren.draw(get_view_matrix(player_physics, player_input));
    }
    
    const cx = @as(f32, @floatFromInt(sapp.width())) * 0.5; 
    const cy = @as(f32, @floatFromInt(sapp.height())) * 0.5;
    const dl = ig.igGetBackgroundDrawList();
    ig.ImDrawList_AddLine(dl, .{ .x = cx - 8, .y = cy }, .{ .x = cx + 8, .y = cy }, 0xFF00FF00);
    ig.ImDrawList_AddLine(dl, .{ .x = cx, .y = cy - 8 }, .{ .x = cx, .y = cy + 8 }, 0xFF00FF00);
    
    simgui.render(); 
    sg.endPass(); 
    sg.commit();
}

export fn cleanup() void { 
    st.world.deinit(); 
    st.store.registry.players.deinit(st.mem); 
    simgui.shutdown(); 
    saudio.shutdown(); 
    sg.shutdown(); 
}

export fn event(e: [*c]const sapp.Event) void {
    _ = simgui.handleEvent(e.*);
    if (st.store.registry.players.items(.input).len == 0) return;
    
    const inp = &st.store.registry.players.items(.input)[0];
    const d = e.*.type == .KEY_DOWN;
    
    switch (e.*.type) {
        .KEY_DOWN, .KEY_UP => switch (e.*.key_code) {
            .W => inp.keys.w = d, 
            .A => inp.keys.a = d, 
            .S => inp.keys.s = d, 
            .D => inp.keys.d = d, 
            .SPACE => inp.keys.sp = d,
            .ESCAPE => if (d and inp.lock) { 
                inp.lock = false; 
                sapp.showMouse(true); 
                sapp.lockMouse(false); 
            }, 
            else => {},
        },
        .MOUSE_DOWN => if (e.*.mouse_button == .LEFT and !inp.lock) { 
            inp.lock = true; 
            sapp.showMouse(false); 
            sapp.lockMouse(true); 
        },
        .MOUSE_MOVE => if (inp.lock) { 
            inp.mdx += e.*.mouse_dx; 
            inp.mdy += e.*.mouse_dy; 
        }, 
        else => {},
    }
}

fn audio(buf: [*c]f32, n: i32, c: i32) callconv(.c) void {
    for (0..@intCast(n)) |i| {
        if (st.jmp) { st.jmp = false; st.t_jmp = 0.15; }
        const s: f32 = if (st.t_jmp > 0) blk: {
            const t = 1.0 - st.t_jmp / 0.15; st.t_jmp -= 1.0 / 44100.0;
            break :blk @sin((0.15 - st.t_jmp) * 500.0 * std.math.pi) * @exp(-t * 8.0) * 0.3;
        } else 0;
        for (0..@as(usize, @intCast(c))) |ch| buf[i * @as(usize, @intCast(c)) + ch] = s;
    }
}

pub fn main() void { sapp.run(.{ .init_cb = init, .frame_cb = frame, .cleanup_cb = cleanup, .event_cb = event, .width = 1024, .height = 768, .window_title = "FPS" }); }