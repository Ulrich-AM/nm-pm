# LO-006 Reference Motion Data

This directory stores derived measurements used by the locomotion study.

The original FBX files are **not** redistributed. They were supplied as reference material and are used only to extract kinematic relationships.

The four labels are:

- walk;
- slow run;
- medium run;
- fast run.

The clips use the same Mixamo skeleton, so joint-space comparisons are consistent. Their names are qualitative movement labels, not measured physical speeds because the animations are in-place.

The study does not replay or fit the clips frame-for-frame. It uses them to establish broad reference envelopes for:

- pelvis and thorax rotation;
- pelvis obliquity and root excursion;
- contralateral arm/leg timing;
- elbow-flexion behavior;
- torso-relative hand corridor;
- changes in coordination between walking and running.

The retained procedural controller remains independent of Mixamo assets.

`LO006_FBX_REFERENCE.json` contains the derived summary used by the LO-006 refinement.