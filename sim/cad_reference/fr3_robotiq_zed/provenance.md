# FR3_2F85_ZED CAD reference provenance

## Active transferred reference — 2026-10-13, Fusion design version 13

The native F3Z manifest root is
`58be0ee1-37f1-4f98-bdf5-318b0f2d7cb6.f3d`. Its design description calls
this `FR3_2F85_ZED` version 13, created at 2026-10-13 08:21:15 UTC. The STEP
header reports export at 2026-10-13 10:22:50 +02:00. The frame export reports
2026-10-13 10:24:02 local; the face export reports 08:33:00 UTC.

| Active file | SHA-256 |
| --- | --- |
| `native/FR3_2F85_ZED.f3z` | `9728784a0c7d3191d31478c6e4dc5bbb4cd5fbc0c59622c7a2ef0583441cad83` |
| `step/FR3_2F85_ZED.step` | `b5a7adbd32179786a2b408d9cd789e7d33e195e40bd0715a5208db3aa5cfaa84` |
| `measurements/fusion_frames_20261013_102402.json` | `3a0b45189d5e2622f4d1835f2ad2298f5ee79a834771aa80c73d47313f36d15d` |
| `measurements/fusion_occurrences_20261013_102402.csv` | `908541c9db749d771e2025f4c7b521b665d7c861a56dfa11bc92c6f63b500d97` |
| `measurements/hg1_fusion_faces_20261013T083300663567Z.json` | `b854c3172c39d6fd08347a8741350e7ebf871fbed71d690383eda0e3f396117e` |
| `measurements/hg1_fusion_faces_20261013T083300663567Z.csv` | `7d533bbce5b302f7cf1dcf0602b48ee18ba0a962a9fc9b0e5ba944ef6ec29497` |

The STEP placement graph contains 38 assembly occurrences. All 38 names and
placement matrices match the active Fusion occurrence export; the maximum
entry difference is `8.32e-13` (millimetres or unitless rotation entries).
Six Fusion FR3 subcomponent occurrences are absent from that STEP graph.
This checks STEP assembly placements, not equality of all native and STEP
BRep surfaces. The F3Z archive integrity and root metadata were verified;
native F3D BRep geometry was not independently parsed.

The six-face file uses `fusion_cad_planar_face_audit_v2`. Every face remains
`plane_qualified_trim_containment_unresolved`. Its supporting-plane and normal
measurements may be used as conditional geometry evidence, but full trimmed
face validation has not been established. Operator `left_pad_contact` is
`fingertips.step:2`; `right_pad_contact` is `fingertips.step:1`. Their mapping
to the USD finger prims remains unverified.

## Superseded transferred reference — 2026-10-13, Fusion design version 11

The earlier HG1-A source inventory recorded the following hashes before the
operator replaced the active files:

| Earlier file | SHA-256 |
| --- | --- |
| F3Z | `b3d7062f0e6777db2236c302aeda066c9d3e37c250952826558fe32ca16e67b8` |
| STEP | `62625b86a90f9edc66dd5bd8635283d247d48c43ba84919f8cdf78895be26933` |
| Fusion frames JSON (09:10:09) | `41e7a7a2ba33f636dff889bb67e1b280dc137522282bcbe507fc32af1fac292f` |
| Fusion occurrences CSV (09:10:09) | `e90f6610d11001ad3b80927098613b10cfd2eb6f60b48f8f0aa5c3f67009c12a` |

The derived transform package committed with the original Robotiq simulation
records a *different, older* STEP hash:
`d5066f1e77bc549beead37cebd1d3c59b46c68be3423113c4c54947327e69812`.
Its mounting and camera transforms are preserved in Git under
`sim/assets/FR3_2F85_ZED_transforms/`; the earlier simulation USD is preserved
at `sim/assets/fr3_robotiq_droid.usda`. Historical joint-5 outputs remain
unchanged.

The CAD reference directory was untracked when version 13 replaced version 11
in the working tree. These prior hashes preserve identity evidence, but the
superseded F3Z/STEP bytes are not recoverable from Git unless separately
retained by the operator. This file records the active version-13 source
identity and does not itself preserve the superseded source bytes.
