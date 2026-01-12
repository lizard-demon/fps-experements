// Game configuration constants organized in nested structs

pub const Physics = struct {
    pub const Movement = struct {
        pub const gravity: f32 = 6.0;
        pub const jump_velocity: f32 = 2.0;
        pub const max_speed: f32 = 4.0;
        pub const air_speed: f32 = 0.7;
        pub const acceleration: f32 = 70.0;
        pub const friction: f32 = 5.0;
    };
    
    pub const Collision = struct {
        pub const margin: f32 = 0.02;
        pub const ground_snap: f32 = 0.1;
        pub const slope_limit: f32 = 0.707; // 45 degrees
        pub const slide_damping: f32 = 0.95;
        pub const slide_damping_step: f32 = 0.05;
        pub const max_slide_iterations: u32 = 3;
        pub const trace_samples_min: u32 = 4;
        pub const trace_samples_max: u32 = 16;
        pub const trace_samples_multiplier: f32 = 10.0;
        pub const binary_search_iterations: u32 = 8;
    };
    
    pub const Hull = struct {
        pub const standing_height: f32 = 0.7;
        pub const standing_radius: f32 = 0.3;
    };
};

pub const Audio = struct {
    pub const jump_sound_duration: f32 = 0.15;
    pub const sample_rate: f32 = 44100.0;
    pub const jump_frequency: f32 = 500.0;
    pub const jump_decay: f32 = 8.0;
    pub const jump_volume: f32 = 0.3;
};

pub const Input = struct {
    pub const mouse_sensitivity: f32 = 0.002;
    pub const pitch_limit: f32 = 1.5;
};

pub const Rendering = struct {
    pub const eye_height: f32 = 0.6;
    pub const crosshair_size: f32 = 8.0;
    pub const fov: f32 = 90.0;
    pub const aspect_ratio: f32 = 1.33;
    pub const near_plane: f32 = 0.1;
    pub const far_plane: f32 = 500.0;
    
    pub const ClearColor = struct {
        pub const r: f32 = 0.15;
        pub const g: f32 = 0.15;
        pub const b: f32 = 0.18;
        pub const a: f32 = 1.0;
    };
    
    pub const CrosshairColor = struct {
        pub const rgba: u32 = 0xFF00FF00;
    };
    
    pub const BrushColor = struct {
        pub const r: f32 = 0.4;
        pub const g: f32 = 0.4;
        pub const b: f32 = 0.4;
        pub const a: f32 = 1.0;
    };
};

pub const World = struct {
    pub const Geometry = struct {
        pub const ground_center_y: f32 = -2.25;
        pub const ground_size_x: f32 = 100.0;
        pub const ground_size_y: f32 = 4.5;
        pub const ground_size_z: f32 = 100.0;
        
        pub const slope_width: f32 = 30.0;
        pub const slope_height: f32 = 20.0;
        pub const slope_center_z: f32 = 20.0;
        pub const slope_angle_degrees: f32 = 46.0;
        pub const slope_ground_level: f32 = 0.0;
    };
    
    pub const Player = struct {
        pub const spawn_x: f32 = 0.0;
        pub const spawn_y: f32 = 3.0;
        pub const spawn_z: f32 = -20.0;
    };
    
    pub const BVH = struct {
        pub const traversal_cost: f32 = 0.3;
        pub const max_leaf_size: u32 = 4;
        pub const epsilon: f32 = 1e-6;
        pub const split_candidates: u32 = 8; // Reduced from testing every primitive
        pub const max_stack_depth: u32 = 64; // For traversal stack
    };
};

pub const Window = struct {
    pub const width: i32 = 1024;
    pub const height: i32 = 768;
    pub const title: [*:0]const u8 = "FPS";
};

pub const Math = struct {
    pub const epsilon: f32 = 1e-6;
    pub const pi: f32 = 3.14159265359;
    pub const degrees_to_radians: f32 = pi / 180.0;
};