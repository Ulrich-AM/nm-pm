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

The internal research harness build number may differ from the public milestone tag. The current development harness is 2.0.1 (Phase 8 animation-driven simplified body plus sprite-sheet export). Published `sprite-projection-v0.1.0` preserves Phase 5 rigid stress, `sprite-projection-v0.2.0` preserves Phase 6 two-link articulation, `sprite-projection-v0.3.0` preserves Phase 7 three-link articulation, and `sprite-projection-v0.4.0` preserves the initial Phase 8 body-animation milestone. The sprite-sheet export maintenance checkpoint uses `sprite-projection-v0.4.1`.

No public milestone release is created until a release tag is intentionally pushed.
