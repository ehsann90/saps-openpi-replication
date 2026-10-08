# Robotiq 2F-85 source asset

This directory is an unmodified copy of
[`grippers/Robotiq_2F_85`](https://github.com/robotiq/isaacsim_assets/tree/ef313b8416096d50c3ab56d67adb21f92d086660/grippers/Robotiq_2F_85)
at Robotiq `isaacsim_assets` revision
`ef313b8416096d50c3ab56d67adb21f92d086660`, plus this provenance file and
the repository-level `UPSTREAM-LICENSE`. The selected configuration is
`configuration/Robotiq_2F_85_config_physics_parallel_grip.usda`, using the
standard fingertip. Simulator-specific mounting and drive tuning are authored
separately in `../fr3_robotiq_droid.usda`.

The USD files in `parts/`, `payloads/`, and `materials/` include Git LFS content.
For a fresh upstream copy, clone the pinned revision with Git LFS installed and
run `git lfs pull`. In this repository these files are stored as the resolved
binary assets, so the simulator does not need Git LFS at runtime. At import,
nine LFS files were downloaded from GitHub's media endpoint and their SHA-256
digests were checked against the pinned Git LFS pointers.

The upstream repository identifies its original work as BSD 3-Clause and
inherited NVIDIA asset content as CC BY 4.0. Preserve `UPSTREAM-LICENSE`,
`PACKAGE-LICENSES/LICENSE`, and `materials/LICENSE` when redistributing this
directory; these contain the full terms and attribution information.
