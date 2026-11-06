#!/usr/bin/env python3
"""Read-only wall-clock audit using the frozen regression's native C++ checks.

No optimization or simulation replay: format saved plans/plant samples for
`test_wall_clock_rh --audit-saved`, with the recorded parameter environment.
Return code 2 means the saved trajectory fails the constraint audit.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--binary", type=Path,
                    default=Path(__file__).resolve().parents[3] / "build/test_wall_clock_rh")
    ap.add_argument("--output", type=Path, help="optional saved audit JSON; otherwise print only")
    args = ap.parse_args()
    run = args.run.resolve()
    if (run / "profile.json").exists():
        profile = json.loads((run / "profile.json").read_text())
    else:
        metadata = json.loads((run.parent / "results.json").read_text())
        record = next((r for r in metadata if r["name"] == run.name), None)
        if record is None:
            raise SystemExit("Run's recorded parameter profile is missing")
        profile = record["profile"]
    summary = json.loads((run / "summary.json").read_text())
    plans = [json.loads(line) for line in (run / "plans.jsonl").read_text().splitlines()]
    with (run / "plant.csv").open() as f:
        rows = list(csv.DictReader(f))
    if not plans or not rows:
        raise SystemExit("Saved plans or plant samples are empty")
    columns = sorted((k for k in rows[0] if k.startswith("x")), key=lambda k: int(k[1:]))
    env = {k: v for k, v in os.environ.items() if not k.startswith(("SZMUK_", "STC_", "MPC_", "WALL_RH_"))}
    env.update(profile)
    with tempfile.TemporaryDirectory(prefix="comando-saved-constraint-audit-") as tmp:
        data, result = Path(tmp) / "saved.txt", Path(tmp) / "audit.json"
        with data.open("w") as f:
            f.write(f'{int(summary["model"] == "quad")} {int(summary["ctcs"])} {len(plans)}\n')
            for plan in plans:
                f.write(f'{plan["activation_epoch"]:.17g} {len(plan["states"])} {len(plan["controls"])}\n')
                for vector in plan["states"] + plan["controls"]:
                    f.write(str(len(vector)) + " " + " ".join(format(v, ".17g") for v in vector) + "\n")
            f.write(str(len(rows)) + "\n")
            for row in rows:
                vector = [float(row[k]) for k in columns]
                f.write(format(float(row["t"]), ".17g") + " " + str(len(vector)) + " "
                        + " ".join(format(v, ".17g") for v in vector) + "\n")
        process = subprocess.run([str(args.binary.resolve()), "--audit-saved", str(data), str(result)],
                                 env=env, capture_output=True, text=True)
        if not result.exists():
            raise SystemExit(process.stderr or "Native audit did not produce a result")
        audit = json.loads(result.read_text())
        audit["audit_returncode"] = process.returncode
        audit["run"] = str(run)
        text = json.dumps(audit, indent=2) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text)
        print(text, end="")
        return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
