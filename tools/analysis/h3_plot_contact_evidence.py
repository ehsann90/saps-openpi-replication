"""Reproduce SIM-P13G H3 contact-evidence plots from archived JSON."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "outputs/isaac_joint5_diagnostics"
OUT = ROOT / "sim/docs/figures/h3"
OUT.mkdir(parents=True, exist_ok=True)


def load(path):
    return json.loads(path.read_text())


def samples(folder):
    return load(DATA / folder / "samples.json")


def impulses(folder):
    records = load(DATA / folder / "contacts.json")
    result = {}
    for record in records:
        if record.get("loaded_fingertip_table_contact"):
            step = int(record["step"])
            result[step] = result.get(step, 0) + float(
                record["impulse_norm_sum_ns"]
            )
    return result


def finish(name):
    plt.tight_layout()
    plt.savefig(OUT / name, dpi=180)
    plt.close()
    print("Created:", OUT / name)


candidate = "hg1_b2_candidate/"
on = samples(candidate + "robotiq_table")
off = samples(candidate + "robotiq_no_table")
contact = impulses(candidate + "robotiq_table")

assert len(on) == len(off) == 67
assert all(a["step"] == b["step"] for a, b in zip(on, off))
assert all(a["target_q_rad"] == b["target_q_rad"]
           for a, b in zip(on, off))

steps = [s["step"] for s in on]
time = [s["time_s"] for s in on]
q5 = [a["measured_q_rad"][4] - b["measured_q_rad"][4]
      for a, b in zip(on, off)]
finger = [s["finger_q"][0] for s in on]
target = [s["finger_target"][0] for s in on]
impulse = [contact.get(k, 0) for k in steps]

loaded = [k for k, v in contact.items() if v > 1e-8]
assert loaded
first, last = min(loaded), max(loaded)


def shade():
    plt.axvspan(first, last, alpha=0.12,
                label="Loaded contact interval")


# F1: joint-5 position difference
plt.figure(figsize=(8, 4))
plt.plot(steps, q5, label="Table ON minus OFF")
shade()
plt.xlabel("Simulation step (60 Hz)")
plt.ylabel("Joint-5 position difference (rad)")
plt.legend()
plt.grid(alpha=0.25)
finish("h3_f1_q5_contact_response.png")


# F2: driven finger position against its target
plt.figure(figsize=(8, 4))
plt.plot(steps, finger, label="Measured finger_joint")
plt.plot(steps, target, "--", label="Commanded target")
shade()
plt.xlabel("Simulation step (60 Hz)")
plt.ylabel("Finger-joint position (rad)")
plt.legend()
plt.grid(alpha=0.25)
finish("h3_f2_finger_deflection.png")


# F3: aligned signals, different units
fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
axes[0].plot(steps, impulse)
axes[0].set_ylabel("Summed impulse norms (N s)")
axes[1].plot(steps, finger)
axes[1].set_ylabel("Finger position (rad)")
axes[2].plot(steps, q5)
axes[2].set_ylabel("q5 ON−OFF (rad)")
axes[2].set_xlabel("Simulation step (60 Hz)")
for ax in axes:
    ax.axvspan(first, last, alpha=0.12)
    ax.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(OUT / "h3_f3_aligned_contact_signals.png", dpi=180)
plt.close(fig)
print("Created:", OUT / "h3_f3_aligned_contact_signals.png")


# F4: descriptive, not wrench-matched, contact comparison
runs = {
    "Franka": "matched_contact_comparison/franka_table",
    "Robotiq": "matched_contact_comparison/robotiq_table_multijoint",
}
metrics = []
for label, folder in runs.items():
    values = list(impulses(folder).values())
    assert values
    metrics.append((label, max(values), sum(values), len(values)))

fig, axes = plt.subplots(1, 2, figsize=(9, 4))
labels = [m[0] for m in metrics]
axes[0].bar(labels, [m[1] for m in metrics])
axes[0].set_title("Peak per-step impulse magnitude")
axes[0].set_ylabel("N s")
axes[1].bar(labels, [m[2] for m in metrics])
axes[1].set_title("Accumulated impulse magnitudes")
axes[1].set_ylabel("N s")
fig.suptitle("Different contact conditions; not a matched-wrench test")
fig.tight_layout()
fig.savefig(OUT / "h3_f4_contact_load_comparison.png", dpi=180)
plt.close(fig)
print("Created:", OUT / "h3_f4_contact_load_comparison.png")
