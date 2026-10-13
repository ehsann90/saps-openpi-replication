# HG1-A supplemental Fusion face export

This read-only Fusion script collects six operator-selected planar BRep faces.
It writes new JSON and CSV files beside the script under `exports/`. It does
not save or edit the active Fusion design.

## Run in Fusion 360

1. Close Fusion. Replace the earlier `hg1_fusion_face_export.py` and
   `hg1_fusion_face_export.manifest` with the updated copies in the Windows
   folder named `hg1_fusion_face_export`. Keep the filenames unchanged, then
   reopen Fusion. For a first installation, create that folder and copy both
   files into it.
2. Open the native `FR3_2F85_ZED` assembly in Fusion, at the design version
   corresponding to the transferred `.f3z`. Make the assembly document active.
   Do not run against the STEP import or an isolated linked component.
3. Open **Utilities → Add-Ins → Scripts and Add-Ins**. Use **+ → Script or
   add-in from device** and choose the `hg1_fusion_face_export` folder. Select
   its script in **My Scripts**, then click **Run**. Autodesk documents this
   folder-based script installation in its [Fusion scripting guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WritingDebugging_UM.htm).
4. Select one planar solid face at each prompt, in this exact order:

   | Prompt | Face to select |
   | --- | --- |
   | `fr3_flange_mating` | Physical FR3 flange mating face |
   | `coupling_fr3_side` | Coupling face that mates to the FR3 |
   | `coupling_robotiq_side` | Opposite coupling face that mates to Robotiq |
   | `robotiq_base_mating` | Physical gripper-base mating face |
   | `left_pad_contact` | Left fingertip's planar object-contact face, in open configuration |
   | `right_pad_contact` | Right fingertip's planar object-contact face, in open configuration |

5. Transfer both new `hg1_fusion_faces_*.json` and
   `hg1_fusion_faces_*.csv` from that script folder's `exports/` subfolder
   into `sim/cad_reference/fr3_robotiq_zed/measurements/` in this repository.
   Keep the original files and names; do not overwrite an earlier export.

The exporter stops without writing if a selection, rigid transform, plane,
normal, or inverse check fails. Its error includes the occurrence, face tokens,
both sample points, transformed coordinates, and plane residuals where those
data are available. Six selections produce one JSON and one CSV file with the
same new timestamp stem.

The v2 export records separate native-face and selected assembly-proxy
`pointOnFace` samples. Each is an interior sample of its own face, not a
centroid, bolt-pattern centre, or chosen mechanical datum. The native sample
and its transformed root coordinate retain the v1 `point_component_*` and
`point_root_*` fields; `point_proxy_assembly_*` adds the proxy sample. The
script compares the supporting planes and outward normals and records four
trimmed-face containment checks. If the planes agree but any containment
check fails, it writes the diagnostic with status
`plane_qualified_trim_containment_unresolved` and names the label in the final
dialog. That status does **not** qualify the face's trimmed geometry.

The measurements will qualify only the selected physical face references.
Internal Robotiq finger pivot axes and joint definitions still require
separate CAD evidence. Do not proceed to HG1-B based only on this export.
