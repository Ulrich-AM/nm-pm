# Dependencies

## raylib 6.2

Used by every native Sprite Projection harness for:

- native desktop windowing
- input
- fonts and text
- 2D rendering
- offscreen capture textures
- PNG loading/export

Project:

https://github.com/raysan5/raylib

## Assimp 6.0.5

Used only by the Phase 8 body-animation harness for FBX animation import.

The project pins Assimp 6.0.5. When it is fetched by CMake, the build is configured as:

- static library
- tests disabled
- tools disabled
- samples disabled
- installation disabled
- exporters disabled
- all importers disabled except FBX

Assimp does not render the body and does not define the Sprite Projection pose model. It is only an input bridge from an FBX hierarchy and animation clip to sampled joint transforms.

Project:

https://github.com/assimp/assimp

## Local research code

The research-specific components remain local project code:

- coordinate and orientation math
- yaw anchor selection
- pitch-grid selection and interpolation
- capture-bank layout
- deformation heuristics
- DUAL and GRID4 view reconstruction
- screen-space roll conversion
- silhouette metrics
- articulated forward kinematics
- simplified-body retargeting
- fixed-proportion body pose generation
- FBX bone-name mapping policy
- E1 through E8 experiment exports

No ImGui, raygui, GLM, OpenCV, or external JSON library is required.
