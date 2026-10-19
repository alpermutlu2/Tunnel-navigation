"""Run the v0.4 error budget with many seeds (final comparisons: 1000) and write
reports/<version>/budget_<seeds>.json. The 200-seed development budget is part of
tools/snapshot.py.

Usage: python tools/budget.py [seeds]
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.budget import budget_report  # noqa: E402

if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    out = d / f"budget_{seeds}.json"
    out.write_text(json.dumps(budget_report(seeds), indent=2) + "\n")
    print(f"wrote {out}")
