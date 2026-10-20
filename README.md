# NM-PM: New Methodology, Powered by Machines

NM-PM is a monorepo for independent experimental technical systems developed through human-directed research with substantial assistance from generative AI and machine-learning-based tools.

Each study is intended to remain useful and understandable independently. Studies should not be framed around a specific game or application unless that context is explicitly part of the research question.

## Studies

### Active

- [Locomotion](projects/locomotion/) - active study for anatomically constrained procedural human locomotion. LO-002 adds segment mass, whole-body COM, support polygons, signed margins, and XCoM diagnostics without active balancing.

### Reference

- [Sprite Projection](projects/sprite-projection/) - completed reference study for view-dependent 2D sprite projection over mathematically 3D rigid and articulated objects.
- [Blood Simulation](projects/blood-simulation/) - completed reference study for real-time hybrid blood simulation in a true-XYZ 2.5D environment.

### Planned
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
locomotion-v0.1.0
blood-simulation-v0.1.0
```

Sprite Projection, Blood Simulation, and Locomotion include Windows milestone release workflows. Blood Simulation preserves `blood-simulation-v0.8.1` as its frozen Phase 7 reference and extends it with `blood-simulation-v0.9.0`, which adds immediate dense-impact gravity rivulets through the existing full-runner pipeline. Matching release requests/tags build the native harness, run its tests, package the executable and documentation, and create the GitHub Release.
