"""Sim output artifacts: trace CSV (with `lap` column), comparison PNG, helper."""
from __future__ import annotations

import csv
import os
import warnings

import numpy as np


def write_trace_csv(result, output_path):
    """Write the per-point trace CSV used for downstream QA.

    v1.1 columns: lap, distance_m, sim_speed_ms, sim_speed_kmh, ai_speed_kmh, time_s.
    `ai_speed_kmh` is left blank when no AI reference is available.
    `lap` is `1` or `2` in two-lap mode; constant `1` in single-lap mode.
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    n = len(result.distances)
    ai = result.ai_speeds
    lap_id = result.lap_id if result.lap_id is not None else np.ones(n, dtype=int)
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "lap", "distance_m", "sim_speed_ms", "sim_speed_kmh",
            "ai_speed_kmh", "time_s",
        ])
        for i in range(n):
            d = float(result.distances[i])
            v = float(result.speeds[i])
            ai_val = "" if ai is None else f"{float(ai[i]) * 3.6:.4f}"
            t = float(result.times[i]) if len(result.times) else 0.0
            w.writerow([
                int(lap_id[i]),
                f"{d:.3f}", f"{v:.4f}", f"{v * 3.6:.4f}", ai_val, f"{t:.4f}",
            ])


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
    """Sim-vs-AI speed overlay.

    In two-lap mode plots lap 2 sim + AI (lap 2 is the headline flying lap).
    In single-lap mode plots the full single-lap sim.
    """
    if result.two_lap and result.lap_id is not None:
        mask = result.lap_id == 2
        sim_kmh = result.speeds[mask] * 3.6
        ai_kmh = result.ai_speeds[mask] * 3.6 if result.ai_speeds is not None else None
        distances = result.distances[mask]
    else:
        sim_kmh = result.speeds * 3.6
        ai_kmh = result.ai_speeds * 3.6 if result.ai_speeds is not None else None
        distances = result.distances

    series = {"sim": sim_kmh}
    if ai_kmh is not None:
        series["ai"] = ai_kmh
    subtitle = f"Lap time: {lap_time_label}" if lap_time_label else None
    return plot_speed_overlay(
        distances,
        series,
        title=f"{track_name} - {driver_name}",
        output_path=output_path,
        subtitle=subtitle,
    )


def write_stint_summary_csv(stint, output_path: str) -> None:
    """Per-lap stint summary (spec §7.11): one row per lap with end-of-lap state.

    Columns:
      lap, lap_time_s,
      tempFL_C, tempFR_C, tempRL_C, tempRR_C,
      wearFL_pct, wearFR_pct, wearRL_pct, wearRR_pct,
      pressureFL_psi, pressureFR_psi, pressureRL_psi, pressureRR_psi
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "lap", "lap_time_s",
            "tempFL_C", "tempFR_C", "tempRL_C", "tempRR_C",
            "wearFL_pct", "wearFR_pct", "wearRL_pct", "wearRR_pct",
            "pressureFL_psi", "pressureFR_psi", "pressureRL_psi", "pressureRR_psi",
        ])
        for k in range(stint.n_laps):
            end_state = stint.tyre_state_history[k + 1]
            lap_time = stint.lap_times_s[k]
            w.writerow([
                k + 1, f"{lap_time:.3f}",
                f"{end_state.temp_C['FL']:.2f}", f"{end_state.temp_C['FR']:.2f}",
                f"{end_state.temp_C['RL']:.2f}", f"{end_state.temp_C['RR']:.2f}",
                f"{end_state.wear_pct['FL']:.3f}", f"{end_state.wear_pct['FR']:.3f}",
                f"{end_state.wear_pct['RL']:.3f}", f"{end_state.wear_pct['RR']:.3f}",
                f"{end_state.pressure_psi['FL']:.3f}", f"{end_state.pressure_psi['FR']:.3f}",
                f"{end_state.pressure_psi['RL']:.3f}", f"{end_state.pressure_psi['RR']:.3f}",
            ])


def print_per_lap_block(stint, *, file=None) -> None:
    """Per-lap stdout block (spec §11.33).

    Format per line:
      Lap N: M:SS.sss | wear FL=..% FR=..% RL=..% RR=..% | temp avg ..°C | pressure avg .. psi
    """
    import sys
    out = file if file is not None else sys.stdout
    for k in range(stint.n_laps):
        lap_time = stint.lap_times_s[k]
        m = int(lap_time // 60)
        s = lap_time - m * 60
        t_str = f"{m}:{s:06.3f}"
        end = stint.tyre_state_history[k + 1]
        temps = list(end.temp_C.values())
        press = list(end.pressure_psi.values())
        print(
            f"Lap {k + 1}: {t_str} | "
            f"wear FL={end.wear_pct['FL']:.0f}% FR={end.wear_pct['FR']:.0f}% "
            f"RL={end.wear_pct['RL']:.0f}% RR={end.wear_pct['RR']:.0f}% | "
            f"temp avg {sum(temps) / 4:.0f}°C | "
            f"pressure avg {sum(press) / 4:.1f} psi",
            file=out,
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
