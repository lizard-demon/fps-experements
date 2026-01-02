# Stupid Simple DAG Architecture

This FPS game demonstrates the **stupidest possible** implementation of a DAG-embedded architecture. No clever abstractions, no complex types, no fancy compile-time magic. Just **structure IS execution order**.

## Core Philosophy

**"Robustness hates complexity"**

The entire "DAG" is literally just this:

```zig
const DAG = struct {
    fn execute(dt: f32) void {
        input(dt);      // Input first
        movement(dt);   // Movement depends on input
        audio(dt);      // Audio parallel to render
        render(dt);     // Render last
    }
};
```

That's it. The dependency graph is the function call order. No abstractions, no types, no complexity.

## Architecture

### 1. Global Data
- `world_blocks`, `world_vertices`, `world_indices` - just arrays
- `player_pos`, `player_vel`, `player_yaw` - just variables  
- `render_pipeline`, `render_bindings` - just globals
- `jump_time` - just a float

### 2. Pure Functions
- `worldGet()`, `worldCollision()`, `worldMesh()` - operate on globals
- `playerView()` - returns view matrix from globals
- `renderSetup()`, `renderDraw()` - setup and draw with globals
- `input()`, `movement()`, `render()`, `audio()` - the "nodes"

### 3. Execution Order
```zig
DAG.execute(dt);  // Calls functions in dependency order
```

## The "DAG"

```
input() -> movement() -> render()
              |
              v
           audio()
```

This is encoded as:
```zig
input(dt);      // Handle input
movement(dt);   // Move player (needs input)
audio(dt);      // Update audio (parallel)
render(dt);     // Draw everything (needs movement)
```

## Why This Works

**1. Zero Abstraction Overhead**
- No types, no structs, no indirection
- Direct function calls, direct memory access
- Compiler can inline everything

**2. Obvious Dependencies**
- You can see the execution order by reading the code
- No hidden complexity, no magic
- Dependencies are explicit in call order

**3. Maximum Robustness**
- Nothing can break because there's nothing complex
- No type system to fight with
- No abstractions to leak

**4. Perfect Performance**
- Global variables = perfect cache locality
- Direct function calls = no virtual dispatch
- Simple code = aggressive optimization

## Adding New Nodes

Want a physics system? Just add it:

```zig
fn physics(dt: f32) void {
    // Physics logic using globals
}

const DAG = struct {
    fn execute(dt: f32) void {
        input(dt);
        physics(dt);    // Add here in dependency order
        movement(dt);
        audio(dt);
        render(dt);
    }
};
```

That's it. No types to update, no registries to modify, no abstractions to extend.

## The Result

This achieves:

✅ **Zero Complexity** - Just functions and globals  
✅ **Obvious Structure** - Execution order is call order  
✅ **Maximum Performance** - No abstractions, no overhead  
✅ **Perfect Robustness** - Nothing complex to break  
✅ **Easy Extension** - Just add function calls  
✅ **Complete Visibility** - Compiler sees everything  

The entire "architecture" is 5 lines of code:

```zig
input(dt);
movement(dt);
audio(dt);
render(dt);
```

This is the ultimate expression of **"structure IS execution"** - the simplest thing that could possibly work, and it works perfectly.

Sometimes the best architecture is no architecture at all.