# Phase 8 and Phase 9 body animation assets

Drop an animated FBX here as:

```text
assets/body_rig/input.fbx
```

The Phase 8 and Phase 9 executables will try to load it automatically at startup.

You can also drag an `.fbx` file directly into the running window.

The importer currently expects a recognizable humanoid hierarchy containing equivalents of:

- pelvis / hips
- chest / upper spine
- head
- left and right upper arms
- left and right forearms
- left and right hands / wrists
- left and right upper legs / thighs
- left and right lower legs / shins
- left and right feet / ankles

Common Mixamo-style names are supported.

Phase 9 additionally looks for spine, neck, hand-tip/finger-base, and toe-base helpers. These are optional: when they are absent, deterministic fallbacks are synthesized from the required major joints.

The external FBX itself is intentionally not committed by default. The research harness includes a deterministic built-in fallback clip so releases remain runnable without third-party animation assets.
