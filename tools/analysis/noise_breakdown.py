#!/usr/bin/env python3
"""Split load-cell noise during grinds into its sources, from exported session data.

Needs sessions recorded by firmware with log schema 3 or later, which carry the raw
HX711 sample on every measurement row. Older sessions are skipped.

Every weight grind already passes through three conditions, so no special test run is
needed - just grinds:

  load cell   motor off, portafilter on the scale (tare, final settling). What the
              HX711, its wiring and the load cell produce with nothing moving.
  vibration   motor running but no grounds in the cup yet - the start-up latency
              window between motor-on and the first grounds arriving.
  grinding    motor on with grounds landing: vibration plus impact of the grounds
              and irregular flow.

Each is measured as the scatter of individual HX711 samples around a local straight
line, so the steady rise while grinding and slow drift while settling do not count as
noise. Only fresh HX711 samples are used: rows are logged at 50Hz, samples arrive at
10 SPS, and the in-between rows repeat the previous sample.

At 10 SPS the fastest frequency visible is 5Hz. Motor vibration is far faster than
that and folds down into the samples as broadband noise, so this separates sources by
WHEN noise appears, not by frequency - which is what can be done at this sample rate.

Usage:
    python tools/analysis/noise_breakdown.py [path/to/grinder_data.db] [--csv DIR]

--csv writes each session's fresh samples (time, raw, grams, motor, phase) for
designing and replaying filters offline.
"""

import argparse
import csv
import math
import os
import sqlite3
import statistics
import sys

# GrindPhase enum values (src/controllers/grind_controller.h)
PHASE_TARE_CONFIRM = 4
PHASE_PREDICTIVE = 5
PHASE_FINAL_SETTLING = 9
PHASE_TIME_GRINDING = 10
PHASE_PRIME = 14

GRINDING_PHASES = {PHASE_PREDICTIVE, PHASE_TIME_GRINDING, PHASE_PRIME}
QUIET_PHASES = {PHASE_TARE_CONFIRM, PHASE_FINAL_SETTLING}

FLOW_ARRIVAL_G = 0.2      # Rise over the motor-on reading that marks grounds arriving

# Crossing FLOW_ARRIVAL_G happens a sample or two after grounds really start landing, so
# the samples just before it already carry impact noise and the start of the ramp. Left
# in, they inflated the vibration figure by ~30% on synthetic data with a known answer.
PRE_ARRIVAL_TRIM = 1

# Three points rather than five: the latency window before grounds arrive holds only a
# handful of samples at 10 SPS, and a wider fit would leave too few residuals per window
# and let the flat-to-ramp corner contaminate more of them.
LOCAL_FIT_POINTS = 3      # Samples in the local straight-line fit (odd, centred)
MIN_SEGMENT_SAMPLES = LOCAL_FIT_POINTS


def fresh_samples(rows, tare, cal):
    """One entry per HX711 sample: (time_ms, raw, grams, motor_on, phase).

    A row carries a new sample only when raw_sample_seq changes. Also counts samples
    that arrived without any row capturing them (sequence jumps of more than one).
    """
    samples, missed, prev_seq = [], 0, None
    for timestamp_ms, raw, seq, age_ms, motor_on, phase in rows:
        if raw is None or seq is None:
            continue
        if seq == prev_seq:
            continue
        if prev_seq is not None:
            gap = (seq - prev_seq) % 65536
            missed += max(0, gap - 1)
        prev_seq = seq
        sample_time = timestamp_ms - (age_ms if age_ms is not None else 0)
        samples.append((sample_time, raw, (raw - tare) / cal, motor_on, phase))
    return samples, missed


def local_residuals(points):
    """Residual of each interior sample from a straight line through its neighbours.

    Fitted at the true sample times, so jitter in the 10 SPS spacing does not leak in.
    Scaled so the result estimates the per-sample noise sigma: the centre point pulls
    the fit towards itself and would otherwise understate it.
    """
    half = LOCAL_FIT_POINTS // 2
    out = []
    for i in range(half, len(points) - half):
        window = points[i - half:i + half + 1]
        ts = [p[0] for p in window]
        ys = [p[1] for p in window]
        t_mean = sum(ts) / len(ts)
        y_mean = sum(ys) / len(ys)
        sxx = sum((t - t_mean) ** 2 for t in ts)
        slope = (sum((t - t_mean) * (y - y_mean) for t, y in zip(ts, ys)) / sxx) if sxx > 0 else 0.0
        fitted = y_mean + slope * (points[i][0] - t_mean)
        leverage = 1.0 / len(ts) + ((points[i][0] - t_mean) ** 2 / sxx if sxx > 0 else 0.0)
        if leverage >= 1.0:
            continue
        out.append((points[i][1] - fitted) / math.sqrt(1.0 - leverage))
    return out


def contiguous_runs(samples, predicate):
    """Split samples into runs where predicate holds, so fits never span a gap."""
    runs, current = [], []
    for s in samples:
        if predicate(s):
            current.append(s)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return runs


def segment_residuals(samples):
    """Residuals per noise source for one session."""
    quiet = []
    for run in contiguous_runs(samples, lambda s: not s[3] and s[4] in QUIET_PHASES):
        if len(run) >= MIN_SEGMENT_SAMPLES:
            quiet += local_residuals([(s[0], s[2]) for s in run])

    vibration, grinding = [], []
    # Each motor start opens a latency window, including the prime and every pulse
    for run in contiguous_runs(samples, lambda s: s[3]):
        start_grams = run[0][2]
        arrived = next((i for i, s in enumerate(run) if s[2] - start_grams > FLOW_ARRIVAL_G), None)
        # No arrival at all (empty hopper, or a pulse too short to deliver) leaves the
        # whole run as pure vibration, which is the cleanest measurement available
        before = run if arrived is None else run[:max(0, arrived - PRE_ARRIVAL_TRIM)]
        after = [] if arrived is None else [s for s in run[arrived:] if s[4] in GRINDING_PHASES]
        if len(before) >= MIN_SEGMENT_SAMPLES:
            vibration += local_residuals([(s[0], s[2]) for s in before])
        if len(after) >= MIN_SEGMENT_SAMPLES:
            grinding += local_residuals([(s[0], s[2]) for s in after])

    return {"load cell": quiet, "vibration": vibration, "grinding": grinding}


def sigma(values):
    return statistics.pstdev(values) if len(values) >= 2 else float("nan")


def fmt(value):
    return "   n/a" if math.isnan(value) else f"{value:6.4f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    default_db = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "database", "grinder_data.db")
    parser.add_argument("db", nargs="?", default=os.path.normpath(default_db),
                        help="database written by `tools/grinder.py export` (default: tools/database/grinder_data.db)")
    parser.add_argument("--csv", metavar="DIR", help="write each session's fresh samples as CSV")
    args = parser.parse_args()

    if not os.path.exists(args.db):
        sys.exit(f"Database not found: {args.db} - run `python tools/grinder.py export` first")

    conn = sqlite3.connect(args.db)
    columns = {row[1] for row in conn.execute("PRAGMA table_info(grind_measurements)")}
    if "raw_adc" not in columns:
        sys.exit("This database has no raw samples - export sessions recorded by schema 3 firmware")

    sessions = conn.execute(
        "SELECT session_id, tare_offset_raw, cal_factor, target_weight, final_weight "
        "FROM grind_sessions WHERE cal_factor IS NOT NULL AND cal_factor != 0 ORDER BY session_id").fetchall()
    if not sessions:
        sys.exit("No sessions with raw samples and a tare yet - grind a few times on schema 3 firmware")

    if args.csv:
        os.makedirs(args.csv, exist_ok=True)

    pooled = {"load cell": [], "vibration": [], "grinding": []}
    total_missed = total_samples = 0

    print(f"{'session':>7}  {'samples':>7}  {'missed':>6}  {'load cell':>9}  {'vibration':>9}  {'grinding':>9}   (sigma, g)")
    for session_id, tare, cal, target, final in sessions:
        rows = conn.execute(
            "SELECT timestamp_ms, raw_adc, raw_sample_seq, raw_sample_age_ms, motor_is_on, phase_id "
            "FROM grind_measurements WHERE session_id = ? ORDER BY sequence_id", (session_id,)).fetchall()
        samples, missed = fresh_samples(rows, tare, cal)
        if not samples:
            continue
        total_missed += missed
        total_samples += len(samples)

        parts = segment_residuals(samples)
        for key, values in parts.items():
            pooled[key] += values
        print(f"{session_id:>7}  {len(samples):>7}  {missed:>6}  "
              f"{fmt(sigma(parts['load cell'])):>9}  {fmt(sigma(parts['vibration'])):>9}  "
              f"{fmt(sigma(parts['grinding'])):>9}")

        if args.csv:
            path = os.path.join(args.csv, f"session_{session_id}.csv")
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_ms", "raw_adc", "grams", "motor_on", "phase_id"])
                writer.writerows(samples)

    print()
    print("Pooled over all sessions (sigma per HX711 sample):")
    for key in ("load cell", "vibration", "grinding"):
        values = pooled[key]
        print(f"  {key:<10} {fmt(sigma(values))} g   from {len(values)} samples")
    if total_samples:
        print(f"  missed samples: {total_missed} of {total_samples + total_missed} "
              f"({100.0 * total_missed / (total_samples + total_missed):.1f}%)")

    base, vib, grind = (sigma(pooled[k]) for k in ("load cell", "vibration", "grinding"))
    print()
    print("Reading it:")
    if not math.isnan(base):
        print(f"  Load cell / electrical floor is {base:.4f} g. The settling test requires 0.010 g;"
              f" {'this floor alone exceeds it' if base > 0.010 else 'the floor is below it'}.")
    if not math.isnan(base) and not math.isnan(vib) and base > 0:
        extra = math.sqrt(max(vib ** 2 - base ** 2, 0.0))
        print(f"  Vibration adds {extra:.4f} g on top of that ({vib / base:.1f}x the floor)."
              f" {'Mechanical coupling dominates - look at mounting and damping.' if vib > 2 * base else 'Vibration pickup is modest.'}")
    if not math.isnan(vib) and not math.isnan(grind) and vib > 0:
        extra = math.sqrt(max(grind ** 2 - vib ** 2, 0.0))
        print(f"  Grounds landing add {extra:.4f} g beyond vibration ({grind / vib:.1f}x vibration alone).")
    if total_samples and total_missed / max(total_samples, 1) > 0.02:
        print("  More than 2% of samples were never logged - the control loop is falling behind the HX711.")
    if len(pooled["vibration"]) < 30:
        print("  Fewer than 30 vibration samples so far - the latency window is short; more grinds will firm this up.")


if __name__ == "__main__":
    main()
