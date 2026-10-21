#!/usr/bin/env python3
"""Group summaries of mc_physical_scoring.py tables, historical vs current.

For each (study, campaign, group): outcome counts and, for every scored
constraint in each window, the number/fraction of trials with an excess above
the reporting tolerance, the median and 90th percentile of the per-trial
maximum excess, and the median violation duration. All trials are included;
non-landings keep their scored window. Also touchdown error (landed trials),
completion time, accepted/rejected solves and solve times.

Usage: mc_compare_summary.py --table NAME=study:campaign:path.csv ... --out DIR
"""
import argparse
import collections
import csv
import json
import math
import pathlib

import numpy as np

ORIG = ("glideslope_m", "speed_m_s", "tilt_deg", "angular_rate_rad_s")
ACT = ("altitude_floor_m", "speed_m_s", "tilt_deg", "angular_rate_rad_s",
       "glideslope_m", "los_deg", "los_cone_m", "staging_height_m")


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def stats(values):
    v = [x for x in values if x is not None]
    if not v:
        return dict(n=0)
    return dict(n=len(v), median=float(np.median(v)), p90=float(np.percentile(v, 90)),
                max=float(max(v)))


def group_summary(rows):
    out = dict(trials=len(rows), results=dict(collections.Counter(r["result"] for r in rows)))
    out["landed"] = out["results"].get("LANDED", 0)
    landed = [r for r in rows if r["result"] == "LANDED"]
    out["td_lateral_m_landed"] = stats(num(r.get("td_lateral_m")) for r in landed)
    out["completion_s_landed"] = stats(num(r.get("completion_s")) for r in landed)
    for key in ("accepted", "rejected", "solve_ms_median", "solve_ms_p95"):
        out[key] = stats(num(r.get(key)) for r in rows)
    for prefix, names in (("orig", ORIG), ("act", ACT)):
        scored = [r for r in rows if num(r.get(f"{prefix}_{names[0]}_max")) is not None]
        block = dict(scored=len(scored))
        for n in names:
            viol = [int(num(r.get(f"{prefix}_{n}_viol")) or 0) for r in scored]
            block[n] = dict(violated=sum(viol), fraction=(sum(viol)/len(scored) if scored else None),
                            max_excess=stats(num(r.get(f"{prefix}_{n}_max")) for r in scored),
                            duration_s=stats(num(r.get(f"{prefix}_{n}_dur_s")) for r in scored))
        anyv = [int(num(r.get(f"{prefix}_any_viol")) or 0) for r in scored]
        block["any"] = dict(violated=sum(anyv), fraction=(sum(anyv)/len(scored) if scored else None))
        out[prefix] = block
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", action="append", required=True,
                    help="study:campaign:path.csv, e.g. mc1:historical:mc1_scores.csv")
    ap.add_argument("--out", type=pathlib.Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    summary, flat = {}, []
    for spec in args.table:
        study, campaign, path = spec.split(":", 2)
        rows = list(csv.DictReader(open(path)))
        key = "motion" if study == "mc1" else "level"
        groups = collections.OrderedDict()
        for r in rows:
            groups.setdefault(r[key], []).append(r)
        groups["all"] = rows
        summary.setdefault(study, {})[campaign] = dict(source=str(path), truth=dict(collections.Counter(
            r.get("target_truth", "") for r in rows)), groups={g: group_summary(v) for g, v in groups.items()})
        for g, s in summary[study][campaign]["groups"].items():
            row = dict(study=study, campaign=campaign, group=g, trials=s["trials"], landed=s["landed"],
                       td_median=s["td_lateral_m_landed"].get("median"),
                       solve_ms_median=s["solve_ms_median"].get("median"),
                       rejected_median=s["rejected"].get("median"))
            for prefix, names in (("orig", ORIG), ("act", ACT)):
                for n in names + ("any",):
                    b = s[prefix][n]
                    row[f"{prefix}_{n}_viol"] = b["violated"]
                    if n != "any":
                        row[f"{prefix}_{n}_med"] = b["max_excess"].get("median")
                        row[f"{prefix}_{n}_dur_med"] = b["duration_s"].get("median")
            flat.append(row)
    (args.out/"mc_summary.json").write_text(json.dumps(summary, indent=1)+"\n")
    with open(args.out/"mc_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in flat for k in r)))
        w.writeheader()
        w.writerows(flat)
    print("wrote", args.out/"mc_summary.json", args.out/"mc_summary.csv")


if __name__ == "__main__":
    main()
