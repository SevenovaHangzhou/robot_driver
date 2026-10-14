# Golden baseline preparation

M2d generates and freezes golden bodies from the approved source commit.
This directory has no generated baseline and establishes no composition
comparison. It is excluded from R5 because legacy outputs contain legacy names
(ADR-19). Source, synthetic fixtures and conformance inputs are still scanned.

`manifest.yaml` contains only the draft `rename_map`. Identifier and relative
path substitutions are exact, never regular expressions. The runtime joint
path is an identity mapping: no new runtime subdirectory is authorized by this
draft. In M2d, expand the documented `<joint>` placeholder to the finite set
of actual baseline names before exact matching; it is not a wildcard rule.
All other output differences remain failures. A0 reviews this draft; M2d adds
source commit, generation command, and input/output hashes with the bodies.
