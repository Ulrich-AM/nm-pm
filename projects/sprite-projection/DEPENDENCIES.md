# Dependencies

## raylib 6.2

The only external runtime/build dependency.

Used for:

- native desktop windowing
- input
- fonts and text
- 2D rendering
- offscreen capture textures
- PNG loading/export

The research-specific components remain local project code:

- coordinate and orientation math
- yaw anchor selection
- pitch-grid selection and interpolation
- capture-bank layout
- deformation heuristics
- GRID4 view reconstruction
- silhouette metrics
- E1, E3, and E4 experiment exports

Project:

https://github.com/raysan5/raylib

No ImGui, raygui, GLM, OpenCV, or external JSON library is required.
