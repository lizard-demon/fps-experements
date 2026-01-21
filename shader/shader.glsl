@header const math = @import("math")
@ctype mat4 math.Mat4

@vs vs
layout(binding = 0) uniform vs_params { mat4 mvp; };
in vec4 position; in vec4 color0; in vec2 texcoord0; in vec4 atlas_info;
out vec4 color; out vec3 world_pos; out vec2 uv; out vec4 atlas_data;
void main() {
    gl_Position = mvp * position;
    world_pos = position.xyz;
    color = color0;
    uv = texcoord0;
    atlas_data = atlas_info; // [offset_x, offset_y, size_x, size_y]
}
@end

@fs fs
layout(binding = 0) uniform texture2D atlas;
layout(binding = 0) uniform sampler smp;
in vec4 color; in vec3 world_pos; in vec2 uv; in vec4 atlas_data;
out vec4 frag_color;

void main() {
    vec3 n = normalize(cross(dFdx(world_pos), dFdy(world_pos)));
    
    // Use calculated UV coordinates from vertex data
    // Only fall back to world-space mapping if UV is exactly (0,0)
    vec2 tex_uv = uv;
    if (uv.x == 0.0 && uv.y == 0.0) {
        // Fallback to world-space mapping for models without proper UVs
        tex_uv = (abs(n.x) > 0.5) ? world_pos.yz * 0.25 : 
                 (abs(n.y) > 0.5) ? world_pos.xz * 0.25 : 
                 world_pos.xy * 0.25;
    }
    
    // Sample from texture atlas using proper bounds
    // atlas_data = [offset_x, offset_y, size_x, size_y]
    vec2 atlas_offset = atlas_data.xy;
    vec2 atlas_size = atlas_data.zw;
    
    // Use fract to tile the texture and clamp to [0,1] for safety
    vec2 tiled_uv = clamp(fract(tex_uv), 0.0, 1.0);
    vec2 atlas_coord = atlas_offset + tiled_uv * atlas_size;
    
    vec4 tex_color = texture(sampler2D(atlas, smp), atlas_coord);
    
    // Simple lighting
    float light = dot(n, vec3(0.0, -1.0, 0.0)) * 0.3 + 0.7;
    
    // Blend texture with vertex color
    vec3 final_color = mix(tex_color.rgb, color.rgb * tex_color.rgb, 0.3);
    frag_color = vec4(final_color * light, color.a * tex_color.a);
}
@end

@program shader vs fs
