#!/usr/bin/env python3
"""Offline summary for LegBot bridge CSV logs."""

import argparse
import csv
import math
import os
import re
import statistics
from collections import defaultdict


JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

LEGS = ["FR", "FL", "RR", "RL"]
LEG_JOINTS = {
    "FR": (0, 1, 2),
    "FL": (3, 4, 5),
    "RR": (6, 7, 8),
    "RL": (9, 10, 11),
}
DIAGONAL_PAIRS = {
    (0, 3): ("FR", "RL"),
    (1, 2): ("FL", "RR"),
}
PAIR_LABELS = {
    (0, 1): "front_pair_FR_FL",
    (0, 2): "right_side_FR_RR",
    (0, 3): "diag_FR_RL",
    (1, 2): "diag_FL_RR",
    (1, 3): "left_side_FL_RL",
    (2, 3): "rear_pair_RR_RL",
}
FOCUS_JOINTS = [1, 4, 7, 10, 2, 5, 8, 11]
EPS = 1e-5
BRIDGE_REJECT_CODES = {
    0: "ok",
    1: "q_des_not_finite",
    2: "q_des_out_of_range",
    3: "q_des_jump",
    4: "qd_des_too_large",
    5: "rear_thigh_too_low",
    6: "pre_trot_timeout",
}


def f(row, key, default=math.nan):
    try:
        value = row.get(key, "")
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def finite(values):
    return [v for v in values if math.isfinite(v)]


def mean(values):
    values = finite(values)
    return statistics.fmean(values) if values else math.nan


def minmax(values):
    values = finite(values)
    if not values:
        return (math.nan, math.nan)
    return (min(values), max(values))


def fmt(value, width=8, precision=4):
    if not math.isfinite(value):
        return "nan".rjust(width)
    return f"{value:{width}.{precision}f}"


def infer_mode(row):
    mode = row.get("mode", "").strip()
    if mode:
        return mode
    control_mode = round(f(row, "control_mode", -1))
    requested_gait = round(f(row, "requested_gait", -1))
    cmpc_gait = round(f(row, "cmpc_gait", -1))
    if control_mode == 3:
        return "MIT_BALANCE_STAND_DIRECT"
    if control_mode == 4 and requested_gait == 4 and cmpc_gait == 4:
        return "MIT_FORWARD_PRE_TROT"
    if control_mode == 4:
        return "MIT_FORWARD_DIRECT"
    if control_mode == 1:
        return "Q_STAND_HOLD"
    return "UNKNOWN"


def load_rows(path):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    for row in rows:
        row["_phase"] = infer_mode(row)
        row["_t"] = f(row, "t")
    return [row for row in rows if math.isfinite(row["_t"])]


def parse_log_counts(csv_path):
    log_path = os.path.splitext(csv_path)[0] + ".log"
    if not os.path.exists(log_path):
        return {}

    counts = defaultdict(lambda: {"num_qdes_clamped": 0, "num_qdes_delta_clamped": 0})
    current = None
    mode_re = re.compile(r"^\[LEGBOT-DDS\]\[(MIT-[^\]]+)\]")
    q_re = re.compile(r"^num_qdes_clamped=(\d+)")
    dq_re = re.compile(r"^num_qdes_delta_clamped=(\d+)")
    tag_to_phase = {
        "MIT-BALANCE-STAND-DIRECT": "MIT_BALANCE_STAND_DIRECT",
        "MIT-FORWARD-DIRECT": "MIT_FORWARD_DIRECT",
    }

    with open(log_path, errors="replace") as handle:
        for raw in handle:
            line = raw.strip()
            match = mode_re.match(line)
            if match:
                current = tag_to_phase.get(match.group(1))
                continue
            if line.startswith("phase=PRE_TROT") or line.startswith("phase=LOCOMOTION_PRE_TROT"):
                current = "MIT_FORWARD_PRE_TROT"
                continue
            if line.startswith("phase=DIRECT"):
                current = "MIT_FORWARD_DIRECT"
                continue
            if not current:
                continue
            match = q_re.match(line)
            if match:
                counts[current]["num_qdes_clamped"] += int(match.group(1))
                continue
            match = dq_re.match(line)
            if match:
                counts[current]["num_qdes_delta_clamped"] += int(match.group(1))

    return dict(counts)


def phase_rows(rows):
    phases = []
    seen = set()
    for row in rows:
        phase = row["_phase"]
        if phase not in seen:
            phases.append((phase, [r for r in rows if r["_phase"] == phase]))
            seen.add(phase)

    if rows:
        end_t = rows[-1]["_t"]
        last = [r for r in rows if r["_t"] >= end_t - 1.0]
        if last:
            phases.append(("FAULT_LAST_1S", last))
    return phases


def scalar_summary(rows, key):
    vals = [f(r, key) for r in rows]
    mn, mx = minmax(vals)
    return mean(vals), mn, mx


def array_minmax(rows, prefix, count):
    out = []
    for i in range(count):
        out.append(minmax([f(r, f"{prefix}{i}") for r in rows]))
    return out


def array_mean(rows, prefix, count):
    return [mean([f(r, f"{prefix}{i}") for r in rows]) for i in range(count)]


def count_raw_pub_diff(rows, raw_prefix, pub_prefix, count):
    total = 0
    for row in rows:
        for i in range(count):
            raw = f(row, f"{raw_prefix}{i}")
            pub = f(row, f"{pub_prefix}{i}")
            if math.isfinite(raw) and math.isfinite(pub) and abs(raw - pub) > EPS:
                total += 1
    return total


def max_abs(rows, prefix, count):
    vals = []
    for row in rows:
        for i in range(count):
            vals.append(abs(f(row, f"{prefix}{i}")))
    vals = finite(vals)
    return max(vals) if vals else math.nan


def counts(values):
    out = defaultdict(int)
    for value in values:
        out[value] += 1
    return dict(sorted(out.items(), key=lambda item: str(item[0])))


def pair_label(pair):
    pair = tuple(sorted(pair))
    if not pair:
        return "no_swing"
    return PAIR_LABELS.get(pair, "+".join(LEGS[i] for i in pair))


def swing_pair_from_phase(row):
    pair = []
    for leg in range(4):
        if f(row, f"cmpc_swing_phase_{leg}", 0.) > EPS:
            pair.append(leg)
    return tuple(pair)


def swing_pair_from_mpc_table(row):
    pair = []
    for leg in range(4):
        table = f(row, f"cmpc_mpc_table_now_{leg}", math.nan)
        if math.isfinite(table) and round(table) == 0:
            pair.append(leg)
    return tuple(pair)


def leg_norm(row, prefix, leg_name):
    vals = [f(row, f"{prefix}{idx}") for idx in LEG_JOINTS[leg_name]]
    if not all(math.isfinite(v) for v in vals):
        return math.nan
    return math.sqrt(sum(v * v for v in vals))


def leg_delta_norm(row, prev_row, prefix, leg_name):
    if prev_row is None:
        return math.nan
    total = 0.
    for idx in LEG_JOINTS[leg_name]:
        cur = f(row, f"{prefix}{idx}")
        prev = f(prev_row, f"{prefix}{idx}")
        if not math.isfinite(cur) or not math.isfinite(prev):
            return math.nan
        total += (cur - prev) * (cur - prev)
    return math.sqrt(total)


def top_two_legs(scores, min_score=EPS):
    valid = [(i, v) for i, v in enumerate(scores) if math.isfinite(v) and v > min_score]
    if len(valid) < 2:
        return ()
    valid.sort(key=lambda item: (-item[1], item[0]))
    return tuple(sorted(i for i, _ in valid[:2]))


def print_pair_counter(title, counter):
    total = sum(counter.values())
    print(f"    {title} total={total}")
    for pair, count in sorted(counter.items(), key=lambda item: (-item[1], item[0])):
        pct = 100. * count / total if total else 0.
        print(f"      {pair_label(pair):17s} {count:4d} ({pct:5.1f}%)")


def print_leg_order_check(rows):
    direct = [
        r for r in rows
        if r["_phase"] == "MIT_FORWARD_DIRECT"
        and round(f(r, "requested_gait", -1)) == 0
    ]
    print("\n[leg order check]")
    print("  assumption: model_leg 0=FR 1=FL 2=RR 3=RL; joints grouped as FR 0..2, FL 3..5, RR 6..8, RL 9..11")
    if len(direct) < 2:
        print("  not enough MIT_FORWARD_DIRECT / gait=0 rows")
        return

    by_phase = defaultdict(lambda: {
        "rows": 0,
        "q_delta_top": defaultdict(int),
        "qd_top": defaultdict(int),
        "q_delta_match": 0,
        "qd_match": 0,
        "front_or_side_q": 0,
        "front_or_side_qd": 0,
        "q_delta_sum": [0., 0., 0., 0.],
        "qd_sum": [0., 0., 0., 0.],
    })
    by_table = defaultdict(int)

    prev = None
    for row in direct:
        phase_pair = swing_pair_from_phase(row)
        table_pair = swing_pair_from_mpc_table(row)
        by_table[table_pair] += 1
        if len(phase_pair) != 2:
            prev = row
            continue

        q_delta = [leg_delta_norm(row, prev, "q_des_raw_", leg) for leg in LEGS]
        qd_norm = [leg_norm(row, "qd_des_raw_", leg) for leg in LEGS]
        q_top = top_two_legs(q_delta)
        qd_top = top_two_legs(qd_norm)

        stat = by_phase[phase_pair]
        stat["rows"] += 1
        if len(q_top) == 2:
            stat["q_delta_top"][q_top] += 1
        if len(qd_top) == 2:
            stat["qd_top"][qd_top] += 1
        if q_top == phase_pair:
            stat["q_delta_match"] += 1
        if qd_top == phase_pair:
            stat["qd_match"] += 1
        if q_top in ((0, 1), (0, 2), (1, 3)):
            stat["front_or_side_q"] += 1
        if qd_top in ((0, 1), (0, 2), (1, 3)):
            stat["front_or_side_qd"] += 1
        for i in range(4):
            if math.isfinite(q_delta[i]):
                stat["q_delta_sum"][i] += q_delta[i]
            if math.isfinite(qd_norm[i]):
                stat["qd_sum"][i] += qd_norm[i]
        prev = row

    print_pair_counter("mpc_table_now swing(0) pairs", by_table)
    for phase_pair in sorted(by_phase):
        stat = by_phase[phase_pair]
        rows_n = stat["rows"]
        expected = DIAGONAL_PAIRS.get(phase_pair)
        expected_text = "+".join(expected) if expected else pair_label(phase_pair)
        print(f"  swing_phase {phase_pair} expects active {expected_text}: rows={rows_n}")
        print_pair_counter("q_des_raw delta top2", stat["q_delta_top"])
        print_pair_counter("qd_des_raw norm top2", stat["qd_top"])
        q_match_pct = 100. * stat["q_delta_match"] / rows_n if rows_n else 0.
        qd_match_pct = 100. * stat["qd_match"] / rows_n if rows_n else 0.
        q_bad_pct = 100. * stat["front_or_side_q"] / rows_n if rows_n else 0.
        qd_bad_pct = 100. * stat["front_or_side_qd"] / rows_n if rows_n else 0.
        print(f"    match swing by q_delta={stat['q_delta_match']}/{rows_n} ({q_match_pct:.1f}%)")
        print(f"    match swing by qd_norm={stat['qd_match']}/{rows_n} ({qd_match_pct:.1f}%)")
        print(f"    q_delta front/same-side suspicious={stat['front_or_side_q']}/{rows_n} ({q_bad_pct:.1f}%)")
        print(f"    qd_norm front/same-side suspicious={stat['front_or_side_qd']}/{rows_n} ({qd_bad_pct:.1f}%)")
        q_avg = [v / rows_n if rows_n else math.nan for v in stat["q_delta_sum"]]
        qd_avg = [v / rows_n if rows_n else math.nan for v in stat["qd_sum"]]
        print("    mean q_delta by leg:", " ".join(f"{leg}={fmt(q_avg[i], 7, 4)}" for i, leg in enumerate(LEGS)))
        print("    mean qd_norm by leg:", " ".join(f"{leg}={fmt(qd_avg[i], 7, 4)}" for i, leg in enumerate(LEGS)))


def print_bridge_gate_summary(rows):
    ready_values = [round(f(r, "bridge_ready", 1)) for r in rows]
    ready_counts = counts(v for v in ready_values if math.isfinite(v))
    stage_counts = counts(r.get("bridge_stage", "") or "UNKNOWN" for r in rows)
    code_counts = counts(round(f(r, "bridge_reject_code", 0)) for r in rows)

    ready_text = " ".join(f"{int(k)}:{v}" for k, v in ready_counts.items())
    stage_text = " ".join(f"{k}:{v}" for k, v in stage_counts.items())
    code_text = " ".join(
        f"{BRIDGE_REJECT_CODES.get(int(k), str(int(k)))}:{v}"
        for k, v in code_counts.items()
    )
    print(f"  bridge_ready counts     {ready_text}")
    print(f"  bridge_stage counts     {stage_text}")
    print(f"  bridge_reject counts    {code_text}")


def print_array_range(title, ranges, indexes=None):
    print(title)
    indexes = range(len(ranges)) if indexes is None else indexes
    for i in indexes:
        mn, mx = ranges[i]
        print(f"  {JOINTS[i]:9s} min={fmt(mn)} max={fmt(mx)}")


def print_leg_focus(rows):
    print("  thigh/calf focus q_des_raw / q_feedback ranges:")
    qdes = array_minmax(rows, "q_des_raw_", 12)
    qfb = array_minmax(rows, "q_feedback_", 12)
    qd = array_minmax(rows, "qd_des_raw_", 12)
    for i in FOCUS_JOINTS:
        qmn, qmx = qdes[i]
        fmn, fmx = qfb[i]
        dmn, dmx = qd[i]
        print(
            f"    {JOINTS[i]:9s} q_des=[{fmt(qmn)},{fmt(qmx)}] "
            f"q_fb=[{fmt(fmn)},{fmt(fmx)}] qd_des=[{fmt(dmn)},{fmt(dmx)}]"
        )


def print_phase(name, rows, log_counts):
    t0, t1 = rows[0]["_t"], rows[-1]["_t"]
    print(f"\n[{name}] rows={len(rows)} t={t0:.3f}..{t1:.3f}s")
    for key in ("state_pos_z", "tilt", "max_abs_dq_feedback", "max_qerr"):
        avg, mn, mx = scalar_summary(rows, key)
        label = "max_tilt" if key == "tilt" else key
        print(f"  {label:24s} mean={fmt(avg)} min={fmt(mn)} max={fmt(mx)}")
    print_bridge_gate_summary(rows)

    q_diff = count_raw_pub_diff(rows, "q_des_raw_", "q_des_pub_", 12)
    qd_diff = count_raw_pub_diff(rows, "qd_des_raw_", "qd_des_pub_", 12)
    exact = log_counts.get(name, {})
    if exact:
        print(
            "  num_qdes_clamped/log     "
            f"{exact.get('num_qdes_clamped', 0)}"
        )
        print(
            "  num_qdes_delta/log       "
            f"{exact.get('num_qdes_delta_clamped', 0)}"
        )
    print(f"  num_qdes_raw_pub_diff {q_diff}")
    print(f"  num_qd_raw_pub_diff   {qd_diff}")

    for prefix, title in (
        ("q_feedback_", "  q_feedback min/max:"),
        ("q_des_raw_", "  q_des_raw min/max:"),
        ("q_des_pub_", "  q_des_pub min/max:"),
        ("qd_des_raw_", "  qd_des_raw min/max:"),
        ("tau_raw_", "  tau_raw min/max:"),
        ("tau_pub_", "  tau_pub min/max:"),
    ):
        print_array_range(title, array_minmax(rows, prefix, 12))

    print("  contactEstimate mean:", " ".join(fmt(v, 7, 3) for v in array_mean(rows, "contact_", 4)))
    print("  cmpc_desired_contact mean:", " ".join(fmt(v, 7, 3) for v in array_mean(rows, "cmpc_desired_contact_", 4)))
    print("  cmpc_swing_phase max:", " ".join(fmt(max(finite([f(r, f"cmpc_swing_phase_{i}") for r in rows]) or [math.nan]), 7, 3) for i in range(4)))
    print("  cmpc_swing_time min:", " ".join(fmt(min(finite([f(r, f"cmpc_swing_time_{i}") for r in rows]) or [math.nan]), 7, 3) for i in range(4)))
    print("  cmpc_swing_time_remaining min:", " ".join(fmt(min(finite([f(r, f"cmpc_swing_time_remaining_{i}") for r in rows]) or [math.nan]), 7, 3) for i in range(4)))
    print_leg_focus(rows)


def phase_map(rows):
    out = defaultdict(list)
    for row in rows:
        out[row["_phase"]].append(row)
    return out


def answer_questions(rows):
    phases = phase_map(rows)
    bal = phases.get("MIT_BALANCE_STAND_DIRECT", [])
    pre = phases.get("MIT_FORWARD_PRE_TROT", [])
    direct = phases.get("MIT_FORWARD_DIRECT", [])
    last = [r for r in rows if rows and r["_t"] >= rows[-1]["_t"] - 1.0]

    print("\n[direct answers]")
    if bal and pre:
        bal_rear_thigh = [mean([f(r, "q_des_raw_7") for r in bal]), mean([f(r, "q_des_raw_10") for r in bal])]
        pre_rear_thigh = [minmax([f(r, "q_des_raw_7") for r in pre])[0], minmax([f(r, "q_des_raw_10") for r in pre])[0]]
        bal_rear_calf = [mean([f(r, "q_des_raw_8") for r in bal]), mean([f(r, "q_des_raw_11") for r in bal])]
        pre_rear_calf = [mean([f(r, "q_des_raw_8") for r in pre]), mean([f(r, "q_des_raw_11") for r in pre])]
        print(f"  PRE_TROT q_des_raw vs BalanceStand: rear thigh balance_mean={bal_rear_thigh}, pretrot_min={pre_rear_thigh}")
        print(f"  rear calf balance_mean={bal_rear_calf}, pretrot_mean={pre_rear_calf}")
        print(f"  rear thigh pulled to <=0.80: {any(v <= 0.80 for v in pre_rear_thigh)}")
        print(f"  rear calf pulled toward -1.72: {any(v > -1.75 for v in pre_rear_calf)}")
    if direct:
        print(f"  gait=0 max_abs_qd_des_raw={max_abs(direct, 'qd_des_raw_', 12):.4f} rad/s")
    if last:
        rl = [(r['_t'], f(r, "q_feedback_10"), f(r, "q_des_raw_10")) for r in last]
        print(
            "  fault last 1s RL_thigh q_feedback: "
            f"start={rl[0][1]:.4f} min={min(v[1] for v in rl):.4f} "
            f"end={rl[-1][1]:.4f}; q_des_raw end={rl[-1][2]:.4f}"
        )
    quiet_bad = [
        r for r in rows
        if f(r, "tilt") < 0.02
        and max(abs(f(r, f"gyro_{axis}")) for axis in ("x", "y", "z")) < 0.2
        and max(abs(f(r, f"q_des_raw_{i}")) for i in range(12)) > 3.0
    ]
    print(f"  impossible q_des while tilt/gyro quiet rows: {len(quiet_bad)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    if not rows:
        raise SystemExit("no rows found")

    log_counts = parse_log_counts(args.csv_path)
    print(f"file: {args.csv_path}")
    print(f"rows: {len(rows)} t={rows[0]['_t']:.3f}..{rows[-1]['_t']:.3f}s")
    if log_counts:
        print("log clamp counts: available from sibling .log")
    else:
        print("log clamp counts: unavailable; using raw/pub diff estimates only")

    for name, group in phase_rows(rows):
        print_phase(name, group, log_counts)
    print_leg_order_check(rows)
    answer_questions(rows)


if __name__ == "__main__":
    main()
