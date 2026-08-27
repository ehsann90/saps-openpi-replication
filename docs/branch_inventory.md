# Branch Inventory at Simulation-Baseline Closeout

This inventory records the branch topology inspected on 2026-08-27 after
refreshing `origin`. It distinguishes branch cleanup from experiment provenance;
no published history is rewritten.

| Branch at inspection | Purpose | Tip | In current simulation line? | In `main`? | Unique required commits? | Safe action |
|---|---|---|---:|---:|---|---|
| `feature/gate2-operator-pilot` | Matched-pilot protocol, collection support, accounting analysis, and closeout | `2d2d8fe5efa0a59a05ce8e59a6814f1c1895209f` before closeout edits | Yes | No; seven commits ahead | Yes, until integrated | Fast-forward `main` after verification, tag the final archive, then delete the feature branch |
| `main` | Primary integrated project history | `9fa5a0aeff89541f6a596ba8fc6310ff31eb80e0` before closeout | Yes; ancestor | Yes | No commits absent from current | Retain and advance to the final archive |
| `feature/operator-experiment-sweep` | Earlier operator-session and SpaceMouse workflow development | `70b6d2b1832e2788da4d9e9d51276d5b990f28d4` | Yes; ancestor | Yes; ancestor | No | Delete remote branch after final verification |
| `experiment/latency-aware-saps` | Divergent latency-aware scheduling experiment | `7ae6a6f8e0ad6ec7edbd8198468af3cf3c3ed1b4` | No | No | Yes: `054c4a8` and `7ae6a6f`; two earlier commits are patch-equivalent to integrated work | Retain until its experimental scheduler and documentation are deliberately reviewed or archived |

Only `feature/gate2-operator-pilot` and `main` existed as local branches at
inspection. The operator-sweep branch was remote-only and fully merged. The
latency-aware branch remains remote-only and is not merged merely to simplify
the branch list, because doing so would mix a separate scheduler experiment into
the validated simulation baseline.

After closeout, the intended durable branch state is `main` at the descriptive
`simulation-baseline-v1` tag plus the retained divergent latency experiment.
Historical collection and analysis commits remain discoverable through the
archive documentation and tag history even after deletion of merged feature
branches.
