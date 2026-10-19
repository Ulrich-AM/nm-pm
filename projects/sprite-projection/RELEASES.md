# Sprite Projection Releases

GitHub Releases are milestone checkpoints, not the experiment history. The canonical research record remains on the repository branch and in `DOCUMENTATION.html`.

## Tag format

```text
sprite-projection-v0.1.0
```

Pushing a matching tag triggers `.github/workflows/sprite-projection-release.yml`.

The workflow:

1. configures the CMake project;
2. builds the Windows Release configuration;
3. runs the test suite;
4. packages the executable with project documentation and the NM-PM AI disclosure;
5. creates a GitHub Release and attaches the ZIP.

The internal research harness build number may differ from the public milestone tag. The current development harness is 1.8.0 (Phase 6 articulation). The published sprite-projection-v0.1.0 release remains the Phase 5 rigid-stress checkpoint.

No public milestone release is created until a release tag is intentionally pushed.
