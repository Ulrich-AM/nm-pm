# Locomotion Releases

GitHub Releases preserve useful native checkpoints. The experiment registry and `DOCUMENTATION.html` remain the canonical research history.

## Tag format

```text
locomotion-v0.1.0
```

Matching release requests or tags trigger `.github/workflows/locomotion-release.yml`.

The workflow:

1. configures the CMake project on Windows;
2. builds the Release configuration;
3. runs CTest;
4. packages the current executable and research documentation;
5. creates the GitHub Release and attaches the ZIP.

## v0.1.0 - LO-001 Anatomy Foundation

The first milestone freezes the initial static contract:

- +X right, +Y forward, +Z up;
- deterministic reference body;
- stable hierarchy;
- comfortable and outer joint envelopes;
- heel, forefoot, and toe contact geometry;
- native grayscale inspection harness;
- automated anatomy validation.

No walking controller is included in this release.
