"""Post-flight telemetry report.

    python -m analysis.flight_report                  every session in reports/
    python -m analysis.flight_report reports/flight3  one session
    python -m analysis.flight_report --demo           fly all levels on autopilot first

For each level in a session's flight_log.csv this writes levelN_report.png
and a summary.txt with the key numbers.
"""

import csv
import glob
import math
import os
import sys

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle, Circle

from analysis import plotstyle as ps
from uavsim.mission import LEVELS, WORLD_W, WORLD_H, CHECKPOINT_R, ZONE_R, gate_walls

REPORTS_DIR = "reports"
NUMERIC_SKIP = {"mode"}


def load_log(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if rows and "theta_deg" not in rows[0]:
        raise ValueError(f"{path} uses the old pixel-based log format; fly a new session")
    out = []
    for r in rows:
        out.append({k: (v if k in NUMERIC_SKIP else float(v)) for k, v in r.items()})
    return out


def by_level(rows):
    seg = {}
    for r in rows:
        seg.setdefault(int(r["level"]), []).append(r)
    return seg


def summarize(rows):
    t = [r["t"] for r in rows]
    speed = [math.hypot(r["vx"], r["vz"]) for r in rows]
    # load factor n = |a + g| / g: what an accelerometer on the frame reads
    nz = [math.hypot(r["ax"], r["az"] + 9.81) / 9.81 for r in rows]
    track = [abs(r["theta_deg"] - r["theta_cmd_deg"]) for r in rows if r["mode"] != "RATE"]
    p = [r["P_elec"] for r in rows]
    return {
        "duration_s": t[-1] - t[0] if len(t) > 1 else 0.0,
        "max_speed_mps": max(speed),
        "mean_speed_mps": sum(speed) / len(speed),
        "max_altitude_m": max(r["z"] for r in rows),
        "max_tilt_deg": max(abs(r["theta_deg"]) for r in rows),
        "peak_load_factor_g": max(nz),
        "rms_tilt_tracking_deg": math.sqrt(sum(e * e for e in track) / len(track)) if track else float("nan"),
        "mean_power_W": sum(p) / len(p),
        "peak_power_W": max(p),
        "energy_Wh": rows[-1]["E_used_Wh"] - rows[0]["E_used_Wh"],
        "min_voltage_V": min(r["V_batt"] for r in rows if r["V_batt"] > 0) if any(r["V_batt"] > 0 for r in rows) else 0.0,
        "soc_end_pct": rows[-1]["soc"] * 100,
        "modes": sorted(set(r["mode"] for r in rows)),
    }


def draw_course(ax, level):
    cfg = LEVELS[level]
    for x0, z0, x1, z1 in gate_walls(cfg["gates"]):
        ax.add_patch(Rectangle((x0, max(z0, 0)), x1 - x0, min(z1, WORLD_H) - max(z0, 0),
                               color="#c9a9a6", lw=0))
    for i, (cx, cz) in enumerate(cfg["checkpoints"]):
        ax.add_patch(Circle((cx, cz), CHECKPOINT_R, fill=False, ec=ps.INK2, lw=1))
        ax.text(cx, cz, str(i + 1), ha="center", va="center", fontsize=7, color=ps.INK2)
    for key, label in (("pickup", "pickup"), ("dropzone", "drop")):
        if key in cfg:
            cx, cz = cfg[key]
            ax.add_patch(Circle((cx, cz), ZONE_R, fill=False, ec=ps.INK2, lw=1, ls="--"))
            ax.text(cx, cz - ZONE_R - 0.4, label, ha="center", fontsize=7, color=ps.INK2)


def plot_level(rows, level, title, out_path):
    t = [r["t"] for r in rows]
    s = summarize(rows)
    fig = plt.figure(figsize=(13, 8.2))
    gs = fig.add_gridspec(3, 3, hspace=0.55, wspace=0.3)
    fig.suptitle(f"{title}  |  level {level}: {LEVELS[level]['name']}", x=0.06, ha="left",
                 fontsize=13, fontweight="bold", color=ps.INK)

    ax = fig.add_subplot(gs[0:2, 0:2])
    draw_course(ax, level)
    xs, zs = [r["x"] for r in rows], [r["z"] for r in rows]
    sp = [math.hypot(r["vx"], r["vz"]) for r in rows]
    pts = list(zip(xs, zs))
    lc = LineCollection([[a, b] for a, b in zip(pts, pts[1:])], cmap="Blues", linewidth=2.2)
    lc.set_array(sp[:-1])
    lc.set_clim(0, max(sp) or 1)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, pad=0.01, fraction=0.03)
    cb.set_label("speed [m/s]", fontsize=8, color=ps.INK2)
    ax.plot(xs[0], zs[0], "o", color=ps.AQUA, ms=8, label="start")
    ax.plot(xs[-1], zs[-1], "s", color=ps.ORANGE, ms=8, label="end")
    ax.set_xlim(0, WORLD_W)
    ax.set_ylim(0, WORLD_H)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("altitude z [m]")
    ax.set_title("Trajectory (colour = speed)")
    ax.legend(loc="upper left")

    ax = fig.add_subplot(gs[0, 2])
    ax.plot(t, [r["theta_cmd_deg"] for r in rows], color=ps.INK2, lw=1, ls="--", label="command")
    ax.plot(t, [r["theta_deg"] for r in rows], color=ps.BLUE, lw=1.4, label="measured")
    ax.set_title("Tilt angle [deg]")
    ax.legend(loc="best")

    ax = fig.add_subplot(gs[1, 2])
    rpm = 60 / (2 * math.pi)
    ax.plot(t, [r["w_l"] * rpm for r in rows], color=ps.BLUE, lw=1.2, label="left")
    ax.plot(t, [r["w_r"] * rpm for r in rows], color=ps.ORANGE, lw=1.2, label="right")
    ax.set_title("Rotor speed [rpm]")
    ax.legend(loc="best")

    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, [r["P_elec"] for r in rows], color=ps.ORANGE, lw=1.3)
    ax.set_title("Electrical power [W]")
    ax.set_xlabel("time [s]")

    ax = fig.add_subplot(gs[2, 1])
    ax.plot(t, [r["V_batt"] for r in rows], color=ps.VIOLET, lw=1.3)
    ax.set_title("Battery terminal voltage [V]")
    ax.set_xlabel("time [s]")

    ax = fig.add_subplot(gs[2, 2])
    ax.axis("off")
    ax.set_title("Summary")
    lines = [
        ("duration", f"{s['duration_s']:.1f} s"),
        ("max / mean speed", f"{s['max_speed_mps']:.2f} / {s['mean_speed_mps']:.2f} m/s"),
        ("max tilt", f"{s['max_tilt_deg']:.1f} deg"),
        ("peak load factor", f"{s['peak_load_factor_g']:.2f} g"),
        ("RMS tilt tracking", f"{s['rms_tilt_tracking_deg']:.2f} deg"),
        ("mean / peak power", f"{s['mean_power_W']:.0f} / {s['peak_power_W']:.0f} W"),
        ("energy used", f"{s['energy_Wh']:.2f} Wh"),
        ("min voltage, end SoC", f"{s['min_voltage_V']:.2f} V, {s['soc_end_pct']:.1f} %"),
        ("modes flown", ", ".join(s["modes"])),
    ]
    for i, (k, v) in enumerate(lines):
        y = 0.95 - i * 0.115
        ax.text(0.0, y, k, transform=ax.transAxes, fontsize=8.5, color=ps.INK2, va="top")
        ax.text(0.52, y, v, transform=ax.transAxes, fontsize=8.5, color=ps.INK, va="top", fontweight="bold")

    ps.save(fig, out_path)
    return s


def process_session(session_dir):
    log_path = os.path.join(session_dir, "flight_log.csv")
    if not os.path.isfile(log_path):
        print(f"skip {session_dir}: no flight_log.csv")
        return
    try:
        rows = load_log(log_path)
    except ValueError as e:
        print(f"skip: {e}")
        return
    pilot_file = os.path.join(session_dir, "pilot.txt")
    pilot = open(pilot_file, encoding="utf-8").read().strip() if os.path.isfile(pilot_file) else "unknown pilot"
    title = f"{os.path.basename(session_dir)} ({pilot})"
    with open(os.path.join(session_dir, "summary.txt"), "w", encoding="utf-8") as f:
        for lvl, seg in sorted(by_level(rows).items()):
            s = plot_level(seg, lvl, title, os.path.join(session_dir, f"level{lvl}_report.png"))
            f.write(f"[level {lvl}]\n")
            for k, v in s.items():
                f.write(f"{k:24s} {v:.3f}\n" if isinstance(v, float) else f"{k:24s} {v}\n")
            f.write("\n")


def demo_flight(out_dir):
    """Fly all three levels on autopilot and save the log like the game does."""
    from uavsim import Mission, AUTO
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for lvl in (1, 2, 3):
        m = Mission(lvl)
        m.fc.set_mode(AUTO)
        while not (m.complete or m.crashed) and m.t < 120:
            m.step(1 / 60)
        print(f"level {lvl}: {'complete' if m.complete else m.crash_reason} in {m.t:.1f} s")
        rows += m.log
    with open(os.path.join(out_dir, "flight_log.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out_dir, "pilot.txt"), "w") as f:
        f.write("autopilot")


def main(argv):
    ps.apply()
    if argv and argv[0] == "--demo":
        d = os.path.join(REPORTS_DIR, "demo_autopilot")
        demo_flight(d)
        process_session(d)
        return
    if argv:
        process_session(argv[0])
        return
    sessions = sorted(d for d in glob.glob(os.path.join(REPORTS_DIR, "*")) if os.path.isdir(d))
    if not sessions:
        print("No sessions found in reports/. Fly the simulator first, or use --demo.")
    for d in sessions:
        process_session(d)


if __name__ == "__main__":
    main(sys.argv[1:])
