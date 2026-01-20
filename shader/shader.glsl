@header const math = @import("math")
@ctype mat4 math.Mat4

@vs vs
layout(binding = 0) uniform vs_params { mat4 mvp; };
in vec4 position; in vec4 color0; in vec2 texcoord0; in vec2 atlas_uv;
out vec4 color; out vec3 world_pos; out vec2 uv; out vec2 atlas_offset;
void main() {
    gl_Position = mvp * position;
    world_pos = position.xyz;
    color = color0;
    uv = texcoord0;
    atlas_offset = atlas_uv;
}
@end

@fs fs
layout(binding = 0) uniform texture2D atlas;
layout(binding = 0) uniform sampler smp;
in vec4 color; in vec3 world_pos; in vec2 uv; in vec2 atlas_offset;
out vec4 frag_color;

void main() {
    vec3 n = normalize(cross(dFdx(world_pos), dFdy(world_pos)));
    
    // Use provided UV or fall back to world-space mapping
    vec2 tex_uv = (uv.x != 0.0 || uv.y != 0.0) ? uv : 
        (abs(n.x) > 0.5) ? world_pos.yz * 0.25 : 
        (abs(n.y) > 0.5) ? world_pos.xz * 0.25 : 
        world_pos.xy * 0.25;
    
    // Sample from texture atlas using atlas offset
    vec2 atlas_coord = fract(tex_uv) * 0.25 + atlas_offset; // 0.25 = 1/4 for 4x4 atlas
    vec4 tex_color = texture(sampler2D(atlas, smp), atlas_coord);
    
    // Subtle grid effect
    vec2 grid = fract(tex_uv * 2.0);
    float outline = min(min(grid.x, 1.0 - grid.x), min(grid.y, 1.0 - grid.y));
    float edge = smoothstep(0.01, 0.02, outline) * 0.3 + 0.7;
    
    // Simple lighting
    float light = dot(n, vec3(0.0, -1.0, 0.0)) * 0.3 + 0.7;
    
    // Blend texture with vertex color
    vec3 final_color = mix(tex_color.rgb, color.rgb * tex_color.rgb, 0.3);
    frag_color = vec4(final_color * edge * light, color.a * tex_color.a);
}
@end

@program shader vs fs
