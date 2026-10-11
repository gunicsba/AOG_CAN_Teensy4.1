#!/usr/bin/env python3
"""Analyse a Claas steering diagnostics capture (Teensy firmware, serial key 'D').

Usage:
    python tools/steer_diag.py capture.txt --left-deg 38.5 --right-deg 40.0

The capture can be the raw serial monitor log: comment lines (#...), service-tool
output and other firmware prints are skipped, only rows matching the CSV header are used.

Requires numpy; matplotlib is needed for the plot only.
"""

import argparse
import math
import os
import sys

import numpy as np

STRAIGHT = 32128          # estCurve raw value for zero curvature
THEORY_CPD = 23.3         # counts/deg from kappa = tan(delta) / 3.0 m at 0.25 km^-1/bit
NUMERIC = [
    "t_ms", "estCurve", "valveState", "curv_kmInv", "angle_div10", "angle_div23_3",
    "r301_raw", "r301_cnt", "r301_delta", "r301_age_ms", "r105", "e2d2_delta",
    "cmd131b_curve", "cmd131b_status", "speed_kmh",
    "aog_setpoint_deg", "aog_actual_sent_deg", "steer_engaged", "pwm_or_cmd_out", "r301_status",
]


# ---------------------------------------------------------------- loading

def load(path):
    """Return dict of column -> np.array (NaN for empty) plus 'mark' as list of str."""
    header = None
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("t_ms,mark,"):
                header = line.split(",")
                continue
            if header is None or not line or not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) != len(header):
                continue                      # partial / interleaved line
            rows.append(dict(zip(header, parts)))

    if not rows:
        sys.exit(f"No CSV rows found in {path} (is the 't_ms,mark,...' header line in the capture?)")

    data = {"mark": [r.get("mark", "") for r in rows]}
    for col in NUMERIC:
        vals = []
        for r in rows:
            v = r.get(col, "")
            try:
                vals.append(float(v) if v != "" else math.nan)
            except ValueError:
                vals.append(math.nan)
        data[col] = np.array(vals)
    data["t"] = (data["t_ms"] - data["t_ms"][0]) / 1000.0
    return data


def windows(d, marks, hold_s, band=50.0):
    """Index arrays for each marker in `marks`: [mark time, min(mark + hold, next mark)],
    cut short as soon as the wheels move (5-sample rolling median of estCurve leaves the
    first second's median by more than `band` counts). Single-sample spikes do not cut it."""
    t, e = d["t"], d["estCurve"]
    mark_idx = [i for i, m in enumerate(d["mark"]) if m]
    out = []
    for k, i in enumerate(mark_idx):
        if d["mark"][i] not in marks:
            continue
        end = t[i] + hold_s
        if k + 1 < len(mark_idx):
            end = min(end, t[mark_idx[k + 1]] - 1e-6)
        sel = np.where((t >= t[i]) & (t <= end))[0]
        ref = med(e[sel[t[sel] <= t[i] + 1.0]])
        for j in range(len(sel)):
            if abs(med(e[sel[max(0, j - 2):j + 3]]) - ref) > band:
                sel = sel[:j]
                break
        out.append((d["mark"][i], t[i], sel))
    return out


def fin(a):
    return a[np.isfinite(a)]


def med(a):
    a = fin(a)
    return float(np.median(a)) if a.size else math.nan


def p5_95(a):
    return float(np.percentile(a, 95) - np.percentile(a, 5))


def fmt(v, nd=2):
    return "n/a" if v is None or not np.isfinite(v) else f"{v:.{nd}f}"


# ---------------------------------------------------------------- analysis

def section(title):
    print(f"\n=== {title} " + "=" * max(0, 60 - len(title)))


def summary(d):
    section("Capture")
    t, n = d["t"], len(d["t"])
    dt = np.diff(d["t_ms"])
    print(f"rows {n}, duration {t[-1]:.1f} s, 0x0CAC1E13 interval median {np.median(dt):.0f} ms")
    gaps = np.where(dt > 200)[0]
    print(f"gaps > 200 ms between 0x0CAC1E13 frames: {gaps.size}"
          + ("" if not gaps.size else " (first at t=" + ", ".join(f"{t[i]:.1f}s/{dt[i]:.0f}ms" for i in gaps[:8]) + ")"))
    vs, cnt = np.unique(fin(d["valveState"]), return_counts=True)
    print("valveState values: " + ", ".join(f"{int(v)} x{c}" for v, c in zip(vs, cnt)))
    for col, name in (("r301_raw", "0x301"), ("r105", "0x105"), ("e2d2_delta", "0x18FFE2D2"),
                      ("cmd131b_curve", "0x0CAD131B"), ("speed_kmh", "CCVS speed")):
        print(f"{name:11s} present in {np.isfinite(d[col]).sum()}/{n} rows")
    present = [m for m in d["mark"] if m]
    print("markers: " + (" ".join(present) if present else "none"))

    age = fin(d["r301_age_ms"])
    if age.size:
        print(f"0x301 age at sample: median {np.median(age):.0f} ms, max {age.max():.0f} ms, "
              f">50 ms in {(age > 50).sum()} rows")
    cnt = d["r301_cnt"]
    ok = np.isfinite(cnt[1:]) & np.isfinite(cnt[:-1])
    if ok.any():
        step = (cnt[1:][ok] - cnt[:-1][ok]) % 256
        vals, c = np.unique(step, return_counts=True)
        top = sorted(zip(c, vals), reverse=True)[:6]
        print("0x301 counter step per CSV line (mod 256, ~8 expected at 100 Hz): "
              + ", ".join(f"{int(v)} x{n_}" for n_, v in top))


def centre_and_locks(d, args):
    section("Centre and lock (markers 0 / L / R)")
    zero = windows(d, "0", args.hold_zero, args.still_band)
    left = windows(d, "L", args.hold_lock, args.still_band)
    right = windows(d, "R", args.hold_lock, args.still_band)
    res = {"zero": zero}
    if not zero:
        print("no '0' marker - centre unknown, using 32128")
    c_est = med(np.concatenate([d["estCurve"][s] for _, _, s in zero])) if zero else STRAIGHT
    c_301 = med(np.concatenate([d["r301_raw"][s] for _, _, s in zero])) if zero else math.nan
    res.update(c_est=c_est, c_301=c_301)
    print(f"centre estCurve {c_est:.1f}  (offset {c_est - STRAIGHT:+.1f} counts = "
          f"{(c_est - STRAIGHT) / THEORY_CPD:+.2f} deg at {THEORY_CPD}/deg)")
    print(f"centre r301_raw {fmt(c_301, 1)}")

    for name, wins in (("L", left), ("R", right)):
        for _, t0, s in wins:
            print(f"  {name} at {t0:6.1f}s: estCurve {fmt(med(d['estCurve'][s]), 1)}, "
                  f"r301_raw {fmt(med(d['r301_raw'][s]), 1)}")
    l_est = med(np.concatenate([d["estCurve"][s] for _, _, s in left])) if left else math.nan
    r_est = med(np.concatenate([d["estCurve"][s] for _, _, s in right])) if right else math.nan
    l_301 = med(np.concatenate([d["r301_raw"][s] for _, _, s in left])) if left else math.nan
    r_301 = med(np.concatenate([d["r301_raw"][s] for _, _, s in right])) if right else math.nan
    print(f"full left : estCurve {fmt(l_est, 1)} ({fmt(l_est - c_est, 1)} from centre), "
          f"r301_raw {fmt(l_301, 1)} ({fmt(l_301 - c_301, 1)})")
    print(f"full right: estCurve {fmt(r_est, 1)} ({fmt(r_est - c_est, 1)} from centre), "
          f"r301_raw {fmt(r_301, 1)} ({fmt(r_301 - c_301, 1)})")

    if args.left_deg is None or args.right_deg is None:
        print("pass --left-deg and --right-deg (measured wheel lock angles) for counts/deg")
        return res

    cpd = []
    for name, v_est, v_301, deg in (("left", l_est, l_301, args.left_deg), ("right", r_est, r_301, args.right_deg)):
        if not np.isfinite(v_est):
            continue
        k = abs(v_est - c_est) / deg
        cpd.append(k)
        kappa = abs(v_est - c_est) * 0.25 / 1000.0                 # 1/m
        l_eff = math.tan(math.radians(deg)) / kappa if kappa > 0 else math.nan
        print(f"{name:5s}: estCurve {k:6.2f} counts/deg, r301 {fmt(abs(v_301 - c_301) / deg)} counts/deg, "
              f"tan model -> effective length {fmt(l_eff)} m (PVED set to 3.00 m)")
    if cpd:
        avg = sum(cpd) / len(cpd)
        res["cpd"] = avg
        print(f"estCurve average {avg:.2f} counts/deg (firmware uses 10, theory {THEORY_CPD}); "
              f"AOG 'counts per degree' is an integer -> {round(avg)} "
              f"({(round(avg) - avg) / avg * 100:+.1f}% scale error)")
        lock = (args.left_deg + args.right_deg) / 2
        ratio = math.tan(math.radians(lock)) / math.radians(lock)
        print(f"note: curvature ~ tan(angle). Near centre the slope is {avg / ratio:.2f} counts/deg; with "
              f"{round(avg)} small angles read {(1 - 1 / ratio) * 100:.0f}% low in AOG, with "
              f"{round(avg / ratio)} full lock reads {(ratio - 1) * 100:.0f}% high")
    return res


def sweep_fit(d, args):
    section("Linear fit r301_raw vs estCurve (manual sweep)")
    t = d["t"]
    marks = [(t[i], m) for i, m in enumerate(d["mark"]) if m]
    t0 = args.sweep_start if args.sweep_start is not None else next((tm for tm, m in marks if m in "0LR"), t[0])
    t1 = args.sweep_end if args.sweep_end is not None else next((tm for tm, m in marks if m == "E" and tm > t0), t[-1])
    sel = (t >= t0) & (t <= t1) & (d["steer_engaged"] != 1)
    x, y = d["estCurve"][sel], d["r301_raw"][sel]
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    print(f"segment {t0:.1f}s .. {t1:.1f}s, not engaged, {x.size} rows")
    if x.size < 10 or np.ptp(x) < 1:
        print("not enough movement / 0x301 data for a fit")
        return None
    slope, off = np.polyfit(x, y, 1)
    pred = slope * x + off
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else math.nan
    print(f"r301_raw = {slope:.4f} * estCurve + {off:.1f}   R^2 = {r2:.4f}   "
          f"residual std {np.std(y - pred):.1f} counts")
    print(f"estCurve range {x.min():.0f}..{x.max():.0f}, r301_raw range {y.min():.0f}..{y.max():.0f}")
    if r2 < 0.9:
        print("R^2 < 0.9: 0x301 does not track estCurve linearly (hysteresis, different sensor, or lag)")
    return {"slope": slope, "offset": off, "r2": r2}


def noise(d, cr):
    section("Noise while straight ('0' windows)")
    cpd = cr.get("cpd", THEORY_CPD)
    out = []
    for _, t0, s in cr["zero"]:
        e, r = fin(d["estCurve"][s]), fin(d["r301_raw"][s])
        if e.size < 3:
            continue
        line = (f"0 at {t0:6.1f}s, {e.size} rows: estCurve std {e.std():.2f} p2p {np.ptp(e):.0f} "
                f"({np.ptp(e) / cpd:.2f} deg), {np.unique(e).size} distinct values")
        if r.size >= 3:
            line += f" | r301_raw std {r.std():.2f} p2p {np.ptp(r):.0f}, {np.unique(r).size} distinct"
        print(line)
        out.append((e.size, p5_95(e), p5_95(r) if r.size >= 3 else math.nan))
    if not out:
        print("no usable '0' window")
        return None
    longest = max(out)
    return {"est_rng": longest[1], "r301_rng": longest[2]}


def jumps(d, args):
    section(f"Jumps in estCurve > {args.jump} counts between consecutive samples")
    e, t = d["estCurve"], d["t"]
    de = np.diff(e)
    idx = np.where(np.abs(de) > args.jump)[0] + 1
    if not idx.size:
        print("none")
        return idx
    spikes = steps = 0
    spike_at = set()
    print("   t[s]   from     to   step  dt[ms]  r301 step  valve eng  kind")
    for n, i in enumerate(idx):
        if i - 1 in spike_at:
            kind = "spike return"
        elif i + 1 < len(e) and abs(e[i + 1] - e[i - 1]) <= args.jump:
            kind = "SPIKE (back next sample)"
            spike_at.add(i)
            spikes += 1
        else:
            kind = "step"
            steps += 1
        r_step = d["r301_raw"][i] - d["r301_raw"][i - 1]
        if n < args.max_list:
            print(f"{t[i]:7.2f} {e[i - 1]:6.0f} {e[i]:6.0f} {de[i - 1]:+6.0f} {d['t_ms'][i] - d['t_ms'][i - 1]:7.0f}"
                  f"  {fmt(r_step, 0):>9s}  {fmt(d['valveState'][i], 0):>5s} {fmt(d['steer_engaged'][i], 0):>3s}  {kind}")
    if idx.size > args.max_list:
        print(f"... {idx.size - args.max_list} more (raise --max-list)")
    print(f"{spikes} single-sample spikes, {steps} steps. Spikes without a matching r301 step are signal "
          "dropouts / glitches, not wheel movement; steps during a fast sweep are normal")
    return np.array(sorted(spike_at), dtype=int)


def sasa_check(d, fit, nz):
    section("Is 0x301 the road-wheel sensor or the steering-wheel SASA?")
    eng = d["steer_engaged"] == 1
    e, r = d["estCurve"][eng], d["r301_raw"][eng]
    ok = np.isfinite(e) & np.isfinite(r)
    e, r = e[ok], r[ok]
    if e.size < 20:
        print(f"only {e.size} engaged rows with 0x301 - drive step 4 of the procedure (AOG steering a curve)")
        return
    e_rng = np.percentile(e, 95) - np.percentile(e, 5)
    r_rng = np.percentile(r, 95) - np.percentile(r, 5)
    corr = np.corrcoef(e, r)[0, 1] if np.std(e) > 0 and np.std(r) > 0 else 0.0
    print(f"engaged rows {e.size}: estCurve moved {e_rng:.0f} counts (p5-p95), "
          f"r301_raw moved {r_rng:.0f} counts, correlation {corr:.2f}")
    e_noise = 3 * nz["est_rng"] if nz and np.isfinite(nz["est_rng"]) else 40
    if e_rng < max(e_noise, 40):
        print("wheels hardly moved while engaged -> inconclusive, let AOG steer a real curve")
        return
    r_noise = 3 * nz["r301_rng"] if nz and np.isfinite(nz["r301_rng"]) else 0
    expected = abs(fit["slope"]) * e_rng if fit else math.nan
    if np.isfinite(expected):
        print(f"from the manual sweep fit r301 should have moved ~{expected:.0f} counts")
    small = r_rng <= max(r_noise, 0.2 * expected if np.isfinite(expected) else 0, 2)
    if small:
        print("RESULT: 0x301 stays still while the valve turns the wheels -> steering-wheel sensor (SASA)")
    elif corr > 0.8:
        print("RESULT: 0x301 follows the wheels while the valve steers -> road-wheel angle sensor")
    else:
        print("RESULT: unclear - 0x301 moves but does not follow estCurve (check the plot)")

    # estCurve -> sent angle mapping learned while not engaged (works for /10 and atan firmware alike)
    free = ~eng & np.isfinite(d["estCurve"]) & np.isfinite(d["aog_actual_sent_deg"])
    if free.sum() > 20 and np.ptp(d["estCurve"][free]) > 0:
        poly = np.polyfit(d["estCurve"][free] - STRAIGHT, d["aog_actual_sent_deg"][free], 3)
        expect = np.polyval(poly, d["estCurve"][eng] - STRAIGHT)
        print(f"note: while engaged the firmware reports the commanded setCurve to AOG, not estCurve; "
              f"median |aog_actual_sent_deg - angle(estCurve)| = "
              f"{med(np.abs(d['aog_actual_sent_deg'][eng] - expect)):.2f} deg")


# ---------------------------------------------------------------- plot

def plot(d, jump_idx, out_png, show):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed - skipping plot (pip install matplotlib)")
        return

    blue, orange, ink, muted, grid = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
    t = d["t"]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(14, 9), facecolor="#fcfcfb")
    panels = [
        ("estCurve [counts]", [("estCurve", "estCurve (PVED)", blue, "-"),
                               ("pwm_or_cmd_out", "setCurve sent", orange, "--")]),
        ("r301_raw [counts]", [("r301_raw", "0x301 raw", blue, "-")]),
        ("AOG [deg]", [("aog_actual_sent_deg", "actual sent to AOG", blue, "-"),
                       ("aog_setpoint_deg", "AOG setpoint", orange, "--")]),
    ]
    eng = d["steer_engaged"] == 1
    for ax, (ylabel, series) in zip(axes, panels):
        ax.set_facecolor("#fcfcfb")
        if eng.any():
            ax.fill_between(t, 0, 1, where=eng, transform=ax.get_xaxis_transform(),
                            color=muted, alpha=0.08, linewidth=0, step="mid")
        for col, label, color, ls in series:
            ax.plot(t, d[col], color=color, linestyle=ls, linewidth=1.2 if ls == "-" else 1.0, label=label)
        ax.set_ylabel(ylabel, color=muted)
        ax.grid(True, color=grid, linewidth=0.6)
        ax.tick_params(colors=muted)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(grid)
        if len(series) > 1:
            ax.legend(loc="upper right", frameon=False, fontsize=9)
    if jump_idx is not None and len(jump_idx):
        axes[0].plot(t[jump_idx], d["estCurve"][jump_idx], "o", markersize=5, color=ink,
                     markerfacecolor="none", label="spike")
        axes[0].legend(loc="upper right", frameon=False, fontsize=9)
    for i, m in enumerate(d["mark"]):
        if not m:
            continue
        for ax in axes:
            ax.axvline(t[i], color=muted, linewidth=0.8, linestyle=":")
        axes[0].annotate(m, (t[i], 1.0), xycoords=("data", "axes fraction"), xytext=(2, 2),
                         textcoords="offset points", color=ink, fontsize=9, fontweight="bold")
    axes[0].set_title("Claas steering diagnostics (grey band = steer_engaged)", color=ink, loc="left", pad=18)
    axes[-1].set_xlabel("time [s]", color=muted)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120, facecolor=fig.get_facecolor())
    print(f"\nplot saved to {out_png}")
    if show:
        plt.show()


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="captured serial log / CSV")
    ap.add_argument("--left-deg", type=float, help="measured front wheel angle at full left lock [deg]")
    ap.add_argument("--right-deg", type=float, help="measured front wheel angle at full right lock [deg]")
    ap.add_argument("--hold-zero", type=float, default=10.0, help="seconds after a '0' marker used as straight window")
    ap.add_argument("--hold-lock", type=float, default=3.0, help="seconds after an L/R marker used as lock window")
    ap.add_argument("--still-band", type=float, default=50.0,
                    help="a marker window ends when estCurve moves more than this [counts]")
    ap.add_argument("--sweep-start", type=float, help="fit segment start [s], default first 0/L/R marker")
    ap.add_argument("--sweep-end", type=float, help="fit segment end [s], default E marker or end of log")
    ap.add_argument("--jump", type=float, default=20.0, help="estCurve jump threshold [counts]")
    ap.add_argument("--max-list", type=int, default=50, help="max jumps to list")
    ap.add_argument("--out", help="plot PNG path (default: <log>.png)")
    ap.add_argument("--show", action="store_true", help="also open the plot window")
    args = ap.parse_args()

    d = load(args.log)
    summary(d)
    cr = centre_and_locks(d, args)
    fit = sweep_fit(d, args)
    nz = noise(d, cr)
    jump_idx = jumps(d, args)
    sasa_check(d, fit, nz)
    plot(d, jump_idx, args.out or os.path.splitext(args.log)[0] + ".png", args.show)


if __name__ == "__main__":
    main()
