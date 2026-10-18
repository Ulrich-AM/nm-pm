# NM-PM

**New Methodology, Powered by Machines**

NM-PM is a monorepo for independent experimental technical systems developed through human-directed research with substantial assistance from generative AI and machine-learning-based tools.

Each study should remain useful and understandable independently. Studies should not be framed around a specific game or application unless that context is explicitly part of the research question.

## Planned studies

Current and planned topics include:

- view-dependent 2D sprite projection over mathematically 3D objects;
- fast hybrid blood simulation;
- anatomically informed procedural human locomotion;
- context-aware physical human reactions;
- persistent surface mapping across directional sprites.

Additional studies may be added later.

## Repository structure

Studies live under:

```text
projects/<study>/
```

A study should generally contain:

```text
DOCUMENTATION.html
README.md
src/
experiments/
figures/
assets/
```

The primary documentation is a living standalone `DOCUMENTATION.html` technical notebook. Failed and superseded experiments remain part of the research history.

## Experiment identifiers

When useful, studies assign durable identifiers such as:

```text
SP-001
SP-002
SP-003
```

An unsuccessful experiment is still a research result and should not be deleted merely because it failed.

## Authorship and AI transparency

NM-PM does not present the work as though every algorithm, mathematical derivation, implementation detail, line of code, or document passage was produced manually by the human researcher.

See [AI_DISCLOSURE.md](AI_DISCLOSURE.md) for the development model.

## Releases

The repository preserves source code, research history, experiments, and documentation.

GitHub Releases are reserved for reasonably stable, useful checkpoints of individual studies. They are not required for every experiment.

Project-specific release tags may eventually use forms such as:

```text
sprite-projection-v0.1.0
locomotion-v0.2.0
blood-simulation-v0.1.0
```
