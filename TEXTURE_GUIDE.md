# Texture System Guide

## Overview

The FPS Advanced engine features a robust per-brush texture mapping system that allows each brush and object to specify its own texture. The system supports automatic UV generation with proper scaling and tiling for surfaces of any size.

## Available Textures

The engine currently includes these textures:
- **concrete** - Gray concrete texture for structural elements
- **wood** - Brown wood grain texture for wooden objects  
- **glass** - Transparent/translucent glass texture for windows
- **grass** - Green grass texture for outdoor surfaces

## Per-Brush Texture Assignment

### Map Format
Each brush in your map JSON can specify its texture:

```json
{
  "type": "box",
  "position": [0.0, 0.0, 0.0],
  "size": [4.0, 6.0, 4.0],
  "texture": "wood"
}
```

### Default Behavior
- If no texture is specified, brushes default to "concrete"
- The system gracefully handles missing textures by falling back to available ones

### Example Map with Mixed Textures
```json
{
  "name": "Mixed Materials Demo",
  "spawn_position": [0.0, 3.0, 0.0],
  "brushes": [
    {
      "type": "box",
      "position": [0.0, -1.0, 0.0],
      "size": [20.0, 2.0, 20.0],
      "texture": "concrete"
    },
    {
      "type": "box", 
      "position": [5.0, 2.0, 0.0],
      "size": [2.0, 4.0, 2.0],
      "texture": "wood"
    },
    {
      "type": "box",
      "position": [-5.0, 3.0, 0.0], 
      "size": [4.0, 6.0, 0.2],
      "texture": "glass"
    }
  ]
}
```

## How It Works

### Automatic UV Generation
- Each brush face gets proper UV coordinates during mesh generation
- The system chooses the best projection plane based on face normal
- Textures tile seamlessly across large surfaces without distortion
- Each brush uses its specified texture independently

### Texture Scaling
The texture scale is controlled globally in `src/primitives/brush.zig`:

```zig
const TextureConfig = struct {
    scale: f32 = 4.0, // 4 world units = 1 texture repeat
};
```

**Adjusting Texture Size:**
- **Smaller values** (e.g., `2.0`) = Larger textures, fewer repeats
- **Larger values** (e.g., `8.0`) = Smaller textures, more repeats

### Texture Atlas System
- All textures are packed into a single atlas for performance
- Each texture gets its own UV region in the atlas
- Proper bounds checking prevents texture bleeding
- Seamless tiling within each texture's atlas region

## Material Properties

### Concrete
- **Use for**: Floors, walls, structural elements, platforms
- **Appearance**: Gray, industrial texture
- **Best for**: Large surfaces that need a neutral, solid appearance

### Wood  
- **Use for**: Crates, platforms, decorative elements, furniture
- **Appearance**: Brown wood grain pattern
- **Best for**: Medium-sized objects that should look organic

### Glass
- **Use for**: Windows, barriers, transparent surfaces
- **Appearance**: Translucent with subtle patterns
- **Best for**: Surfaces that should appear see-through or reflective

### Grass
- **Use for**: Outdoor ground surfaces, natural areas
- **Appearance**: Green grass texture
- **Best for**: Terrain and outdoor environments

## Usage Examples

### Concrete Foundation
```json
{
  "type": "box",
  "position": [0.0, -2.0, 0.0],
  "size": [50.0, 4.0, 50.0],
  "texture": "concrete"
}
```

### Wooden Crate
```json
{
  "type": "box", 
  "position": [10.0, 1.0, 5.0],
  "size": [2.0, 2.0, 2.0],
  "texture": "wood"
}
```

### Glass Window
```json
{
  "type": "box",
  "position": [0.0, 3.0, 10.0],
  "size": [8.0, 4.0, 0.2],
  "texture": "glass"
}
```

### Grass Terrain
```json
{
  "type": "box",
  "position": [0.0, -0.5, 0.0], 
  "size": [100.0, 1.0, 100.0],
  "texture": "grass"
}
```

## Technical Details

### Performance Benefits
- Single texture atlas reduces draw calls
- Per-brush textures with no performance penalty
- Efficient UV coordinate calculation during mesh generation
- Proper frustum culling maintains high frame rates

### Memory Efficiency
- Textures loaded once into shared atlas
- Minimal per-vertex memory overhead (position + UV)
- No duplicate texture data

### Robustness
- Graceful fallback for missing textures
- Automatic UV generation prevents texture stretching
- Seamless tiling across surfaces of any size
- Proper atlas bounds checking prevents visual artifacts

## Adding New Textures

To add a new texture:

1. **Create RGBA file**: Convert your texture to `.rgba` format using the conversion script
2. **Place in assets/textures/**: Add the `.rgba` file to the textures directory  
3. **Use in maps**: Reference the texture name (without extension) in your map JSON
4. **Rebuild**: The texture system will automatically include it in the atlas

Example conversion:
```bash
cd assets/textures
./convert.sh your_texture.png
# Creates your_texture.rgba
```

Then use in maps:
```json
{
  "texture": "your_texture"
}
```

## Troubleshooting

### Texture Not Found
- Check that the `.rgba` file exists in `assets/textures/`
- Verify the texture name matches the filename (without extension)
- Check console output for texture loading messages

### Texture Too Large/Small
- Adjust the global `scale` value in `generateFaceUVs()`
- Smaller scale = larger textures, larger scale = smaller textures

### Texture Stretching
- The system should prevent stretching automatically
- If it occurs, check face normal calculations in brush generation

### Atlas Issues
- The system automatically handles atlas packing
- If textures appear corrupted, check atlas size limits in texture registry