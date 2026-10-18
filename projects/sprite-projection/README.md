# Sprite Projection

**NM-PM study:** View-Dependent 2D Sprite Projection for 3D Articulated Motion

This study investigates how mathematically three-dimensional orientation can be approximated at runtime using finite sets of two-dimensional raster sprite views, geometric transformation, interpolation, and depth-aware composition.

The study is application-independent.

## Current research state

The active prototype has reached Phase 4:

- orthographic reference projection;
- rigid C0 and asymmetric A1 subjects;
- frozen disk-backed yaw and pitch capture banks;
- NEAR, STRETCH, SHEAR, AFFINE, DUAL, and GRID4 reconstruction methods;
- screen-space roll reconstruction;
- quantitative silhouette IoU, boundary error, and centroid displacement evaluation;
- living technical documentation;
- retained visual evidence and failed approaches.

The next major phase should begin only after the current Phase 4 maintenance build is verified in native testing.

## Intended folder structure

```text
projects/sprite-projection/
├── DOCUMENTATION.html
├── README.md
├── src/
├── experiments/
├── figures/
└── assets/
```

`DOCUMENTATION.html` is the primary living notebook.

## Experiment IDs

The study uses durable `SP-###` identifiers in addition to legacy phase and E-number labels.

See `experiments/INDEX.md`.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
