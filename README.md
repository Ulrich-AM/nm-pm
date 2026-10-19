# NM-PM — New Methodology, Powered by Machines

NM-PM is a monorepo for independent experimental technical systems developed through human-directed research with substantial assistance from generative AI and machine-learning-based tools.

Each study is intended to remain useful and understandable independently. Studies should not be framed around a specific game or application unless that context is explicitly part of the research question.

## Studies

### Active

- [Blood Simulation](projects/blood-simulation/) — real-time hybrid blood simulation for a true-XYZ 2.5D isometric environment.

### Reference

- [Sprite Projection](projects/sprite-projection/) — completed reference study for view-dependent 2D sprite projection over mathematically 3D rigid and articulated objects.

### Planned
- Anatomically informed procedural human locomotion.
- Context-aware physical human reactions.
- Persistent surface mapping across directional sprites.

Additional studies may be added later.

## Repository structure

Studies live under:

```text
projects/<study>/
```

A study generally contains:

```text
DOCUMENTATION.html
README.md
src/
experiments/
figures/
assets/
```

`DOCUMENTATION.html` is the primary living technical notebook. Failed and superseded experiments remain part of the research history.

## Experiment identifiers

Studies may assign durable identifiers such as:

```text
SP-001
SP-002
SP-003
```

An unsuccessful experiment is still a research result and should not be deleted merely because it failed.

## Authorship and AI transparency

NM-PM does not present the work as though every algorithm, mathematical derivation, implementation detail, line of code, or document passage was produced manually by the human researcher.

See [AI_DISCLOSURE.md](AI_DISCLOSURE.md) for the repository-wide development model.

## Releases

The repository preserves source code, research history, experiments, and documentation.

GitHub Releases are reserved for reasonably stable, useful checkpoints of individual studies rather than every experiment. Project-specific release tags use forms such as:

```text
sprite-projection-v0.1.0
locomotion-v0.2.0
blood-simulation-v0.1.0
```

Sprite Projection and Blood Simulation include Windows milestone release workflows. Blood Simulation advances one milestone release line per implementation phase, beginning with `blood-simulation-v0.1.0` for Phase 0. Matching release requests/tags build the native harness, run its tests, package the executable and documentation, and create the GitHub Release.
