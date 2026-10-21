# LO-006 Reference Motion Analysis

**Purpose:** derive broad whole-body coordination envelopes from supplied reference motion without replaying or tracing the source animations.

## Source set

The supplied reference set contains four in-place FBX clips:

- walk;
- slow run;
- medium run;
- fast run.

All four use the same Mixamo skeleton and compatible bind transforms. This makes joint-space comparisons internally consistent.

The source FBX files are not redistributed by NM-PM. Only derived measurements are retained in `LO006_FBX_REFERENCE.json`.

## Interpretation rule

The clip names are movement labels, not measured physical velocities.

The clips therefore must **not** be treated as samples on one numeric speed axis:

```text
walk -> slow run -> medium run -> fast run
```

is useful as qualitative movement context, but not as a regression table.

This is reinforced by the observed cycle timing: the clip named `medium run` cycles faster than the clip named `fast run`.

## Measurements

Joint trajectories were reconstructed in world space from the common skeleton and sampled over each clip's keyed frames.

The analysis records:

- clip cycle duration;
- cadence if the clip is interpreted as one complete left/right cycle;
- shoulder width;
- wrist lateral distance from the torso center normalized by shoulder width;
- elbow flexion;
- pelvis yaw;
- thorax yaw;
- relative thorax-pelvis yaw;
- pelvis obliquity;
- root lateral excursion;
- root vertical excursion;
- correlation between signed leg separation and contralateral hand motion.

## Primary findings used by LO-006

### 1. The old procedural arms were too wide before they even swung

The LO-001 reference wrist is roughly 1.52 shoulder-widths from the sagittal center.

The supplied walk reference averages about 0.78 shoulder-widths, with most of the clip between about 0.67 and 0.83.

LO-006 therefore does not change anatomical arm lengths. It adds an upper-limb posture solve whose wrist target remains inside a torso-relative corridor.

### 2. Running uses substantially more elbow flexion

The walk clip shows roughly 15-58 degrees across the two elbows.

The slow-run and fast-run clips spend much of their cycles around 60-120 degrees.

This means larger arm motion should not be implemented by simply rotating a straight arm farther from the body.

The retained walking controller uses the walk envelope only. The run measurements remain evidence for later continuous walk-run transition work.

### 3. Contralateral coordination is strongest in walking but not perfectly rigid across running

For the walk clip, signed leg separation and contralateral hand forward motion are almost perfectly correlated.

The same relationship remains present in the run clips but weakens.

LO-006 therefore uses contralateral leg state as a drive target, then passes it through a damped arm state instead of directly assigning an arm angle.

### 4. Pelvis and thorax should not rotate as one rigid block

The walk clip shows pelvis and thorax rotation approximately opposing one another, producing a larger relative thorax-pelvis twist than either segment alone.

The revised controller gives pelvis and thorax separate signed yaw targets.

### 5. Reference clips are envelopes, not goals

The four clips disagree in several supposedly speed-dependent values.

That disagreement is useful.

It prevents the methodology from assuming that one exact pelvis angle, arm angle, or cadence defines a locomotion speed.

The procedural system should remain inside plausible coordination regions while preserving its own continuous state.

## Biomechanics context

Collins, Adamczyk, and Kuo (2009) showed that arm swing can emerge substantially from passive dynamics and that normal arm phasing reduces vertical ground reaction moments:

https://doi.org/10.1098/rspb.2009.0664

The broader walk-run literature also supports treating gait transition as a reorganization of coordination rather than a one-frame animation switch. Diedrich and Warren reported changes in relative phase and hysteresis around preferred walk-run transition:

https://pubmed.ncbi.nlm.nih.gov/7707029/

These references motivate the architecture, while the supplied FBXs provide visual/kinematic examples. Neither source defines one mandatory animation curve.

## Procedural consequence

The revised upper-body path is:

```text
leg/contact state
      |
      +--> pelvis yaw / obliquity / lateral shift
      |
      +--> thorax counter-rotation
      |
      +--> contralateral arm drive target
                    |
                    v
             damped arm state
                    |
                    v
          torso-relative wrist corridor
                    |
                    v
          two-link shoulder/elbow solve
```

The original Mixamo animation curves do not enter runtime.
