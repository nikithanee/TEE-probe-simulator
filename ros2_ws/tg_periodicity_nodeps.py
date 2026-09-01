#!/usr/bin/env python3
"""
Transgastric periodicity test -- NO external dependencies.

Uses only the standard library plus the container's own numpy.
No pandas, no scipy, no matplotlib. Do not pip install anything.

Usage:
    python3 tg_periodicity_nodeps.py results/eval_tol010_n100.csv
    python3 tg_periodicity_nodeps.py results/eval_tol010_n100.csv --view TG

Writes tg_trajectory.csv (two columns: step, distance) so the figure can
be plotted later on any machine, or directly with pgfplots in the paper.
"""

import argparse
import csv
import sys

import numpy as np


TRANSIENT_STEPS = 50
PERIODIC_CV_THRESHOLD = 0.10
FLAT_THRESHOLD = 0.01      # std below this => fixed point, not oscillation


def sniff(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("Empty CSV.")
    cols = list(rows[0].keys())
    print("=" * 68)
    print("COLUMNS:", cols)
    print("ROWS:", len(rows))
    print("=" * 68)
    return rows, cols


def pick(cols, keywords, exclude=()):
    for c in cols:
        lc = c.lower()
        if any(k in lc for k in keywords) and not any(x in lc for x in exclude):
            return c
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--view", default="TG")
    ap.add_argument("--view-col", default=None)
    ap.add_argument("--dist-col", default=None)
    ap.add_argument("--step-col", default=None)
    ap.add_argument("--ep-col", default=None)
    args = ap.parse_args()

    rows, cols = sniff(args.csv)

    view_col = args.view_col or pick(cols, ("view", "target", "goal_name"))
    dist_col = args.dist_col or pick(cols, ("joint_dist", "dist"),
                                     exclude=("goal_dist_x",))
    step_col = args.step_col or pick(cols, ("step",))
    ep_col = args.ep_col or pick(cols, ("episode", "repeat", "run", "trial"))

    print(f"\nview={view_col}  dist={dist_col}  step={step_col}  ep={ep_col}")
    if not (view_col and dist_col):
        sys.exit("Could not identify columns. Re-run with --view-col and "
                 "--dist-col set explicitly from the list above.")

    vals = sorted({r[view_col] for r in rows})
    print("View values:", vals)

    sel = [r for r in rows if args.view.lower() in r[view_col].lower()]
    if not sel:
        sel = [r for r in rows
               if "transgastric" in r[view_col].lower()
               or r[view_col].strip().lower().startswith("tg")]
    if not sel:
        sys.exit(f"No rows matched '{args.view}'.")
    print(f"Selected {len(sel)} transgastric rows.")

    # ---- group into episodes ----------------------------------------
    episodes = {}
    for r in sel:
        key = r[ep_col] if ep_col else "0"
        step = float(r[step_col]) if step_col else len(episodes.get(key, []))
        episodes.setdefault(key, []).append((step, float(r[dist_col])))

    series = []
    for k in sorted(episodes, key=lambda x: (len(x), x)):
        pts = sorted(episodes[k], key=lambda t: t[0])
        series.append(np.array([p[1] for p in pts]))

    # ---- 1. determinism ---------------------------------------------
    print("\n" + "=" * 68)
    print("1. ARE THE EPISODES IDENTICAL?")
    print("=" * 68)
    print(f"Episodes: {len(series)}  lengths: {sorted({len(s) for s in series})}")

    if len(series) > 1:
        n = min(len(s) for s in series)
        stack = np.vstack([s[:n] for s in series])
        spread = stack.std(axis=0)
        print(f"Max std across episodes at any step: {spread.max():.6f}")
        if spread.max() < 1e-6:
            print("--> IDENTICAL. One trajectory, repeated. Say so in the")
            print("    paper: it is direct evidence of a deterministic")
            print("    attractor rather than stochastic failure.")
        elif spread.max() < 1e-3:
            print("--> Near-identical; variation is simulator timing only.")
        else:
            print("--> Episodes differ. The policy is deterministic, so the")
            print("    source is the simulator. Worth one sentence.")

    d = series[0]

    # ---- 2. periodicity ---------------------------------------------
    print("\n" + "=" * 68)
    print("2. IS IT PERIODIC?")
    print("=" * 68)

    steady = d[TRANSIENT_STEPS:] if len(d) > TRANSIENT_STEPS else d
    print(f"Analysing {len(steady)} steps after a {TRANSIENT_STEPS}-step "
          f"transient.")
    print(f"Range {steady.min():.4f} to {steady.max():.4f}")
    print(f"Mean {steady.mean():.4f}, std {steady.std():.4f}")

    if steady.std() < FLAT_THRESHOLD:
        print("\n--> The signal is FLAT. This is a FIXED POINT, not an")
        print("    oscillation. The paper should say 'converges to a fixed")
        print("    point outside tolerance'. This is the cleaner result and")
        print("    matches the four-chamber spike at 0.1443.")
        write_traj(steady)
        return

    # local maxima with a prominence filter
    amp = steady.max() - steady.min()
    prom = 0.05 * amp
    peaks = [i for i in range(1, len(steady) - 1)
             if steady[i] > steady[i - 1] and steady[i] >= steady[i + 1]
             and steady[i] - min(steady[max(0, i - 20):i + 20].min(),
                                 steady[i]) > prom]
    peaks = np.array(peaks)
    print(f"\nPeaks found: {len(peaks)}")

    periodic = False
    if len(peaks) >= 3:
        gaps = np.diff(peaks)
        cv = gaps.std() / gaps.mean() if gaps.mean() else 9.9
        print(f"Inter-peak gaps: {gaps[:15]}{'...' if len(gaps) > 15 else ''}")
        print(f"Mean period {gaps.mean():.2f}, std {gaps.std():.2f}, CV {cv:.3f}")
        print(f"Peak amplitudes: mean {steady[peaks].mean():.4f}, "
              f"std {steady[peaks].std():.4f}")
        periodic = cv < PERIODIC_CV_THRESHOLD
    else:
        print("Fewer than 3 peaks -- not enough structure to call periodic.")

    # autocorrelation
    x = steady - steady.mean()
    ac = np.correlate(x, x, mode="full")[len(x) - 1:]
    ac = ac / ac[0]
    ac_peaks = [i for i in range(1, min(len(ac) - 1, 200))
                if ac[i] > ac[i - 1] and ac[i] >= ac[i + 1] and ac[i] > 0.3]
    if ac_peaks:
        print(f"\nAutocorrelation peaks at lags {ac_peaks[:6]} "
              f"(heights {np.round(ac[ac_peaks[:6]], 3)})")
        print(f"--> Period near {ac_peaks[0]} steps.")
    else:
        print("\nNo autocorrelation peak above 0.3 -- no repeating period.")
        periodic = False

    # ---- 3. verdict --------------------------------------------------
    print("\n" + "=" * 68)
    print("3. WHAT THE PAPER SHOULD SAY")
    print("=" * 68)
    if periodic:
        print('Use "limit cycle". Report the period in steps and the')
        print("amplitude range alongside it.")
    else:
        print('Use "bounded non-convergent oscillation". Do NOT write')
        print('"limit cycle": the spacing is irregular and a reviewer from')
        print("a dynamical-systems background will check. The weaker term")
        print("costs nothing -- your claim is non-convergence, which holds.")

    write_traj(steady)


def write_traj(steady, out="tg_trajectory.csv"):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "joint_dist"])
        for i, v in enumerate(steady, start=TRANSIENT_STEPS):
            w.writerow([i, f"{v:.6f}"])
    print(f"\nTrajectory written to {out}")
    print("Plot it later on any machine, or directly in LaTeX with pgfplots:")
    print(r"  \addplot table[x=step,y=joint_dist,col sep=comma]"
          r"{tg_trajectory.csv};")


if __name__ == "__main__":
    main()
