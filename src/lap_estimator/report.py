"""Sim output artifacts: trace CSV, comparison PNG, generic speed-overlay helper."""
from __future__ import annotations

import csv
import os
import warnings


def write_trace_csv(result, output_path):
    """Write the per-point trace CSV used for downstream QA.

    Columns: distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s.
    `ai_speed_kmh` is left blank when no AI reference is available.
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    n = len(result.distances)
    ai = result.ai_speeds
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["distance_m", "sim_speed_ms", "sim_speed_kmh", "ai_speed_kmh", "time_s"])
        for i in range(n):
            d = float(result.distances[i])
            v = float(result.speeds[i])
            ai_val = "" if ai is None else f"{float(ai[i]) * 3.6:.4f}"
            t = float(result.times[i]) if len(result.times) else 0.0
            w.writerow([f"{d:.3f}", f"{v:.4f}", f"{v * 3.6:.4f}", ai_val, f"{t:.4f}"])


def plot_speed_overlay(distances, series_dict, title, output_path,
                       *, subtitle=None):
    """Single-axes line plot, x = distance_m, y = speed (km/h).

    `series_dict` maps label -> 1D array of speeds in km/h. Skips silently
    (with a stderr warning) if matplotlib isn't installed.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        warnings.warn("matplotlib not installed; skipping plot")
        return False

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(14, 5))
    for label, vals in series_dict.items():
        if vals is None:
            continue
        ax.plot(distances[: len(vals)], vals, label=label, linewidth=1.2)
    ax.set_xlabel("Distance [m]")
    ax.set_ylabel("Speed [km/h]")
    ax.set_title(title if subtitle is None else f"{title}\n{subtitle}",
                 fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return True


def write_comparison_plot(result, track_name, driver_name, output_path,
                          *, lap_time_label=None):
    """Sim-vs-AI speed overlay (thin wrapper over plot_speed_overlay)."""
    series = {"sim": result.speeds * 3.6}
    if result.ai_speeds is not None:
        series["ai"] = result.ai_speeds * 3.6
    subtitle = f"Lap time: {lap_time_label}" if lap_time_label else None
    return plot_speed_overlay(
        result.distances,
        series,
        title=f"{track_name} - {driver_name}",
        output_path=output_path,
        subtitle=subtitle,
    )


def build_output_stem(track_path_or_name, driver_name, *, is_csv_track):
    """Compute the file-stem prefix used for sim outputs.

    For CSV tracks: same directory as the input, name = `<csv_stem>__<driver_name>`.
    For built-in / JSON tracks: cwd, name = `<track_name>__<driver_name>`.
    """
    safe_driver = "".join(c if c.isalnum() or c in "-_" else "_" for c in driver_name)
    if is_csv_track and os.path.isfile(track_path_or_name):
        d = os.path.dirname(track_path_or_name) or "."
        stem = os.path.splitext(os.path.basename(track_path_or_name))[0]
        return os.path.join(d, f"{stem}__{safe_driver}")
    safe_track = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(track_path_or_name))
    return os.path.join(".", f"{safe_track}__{safe_driver}")
