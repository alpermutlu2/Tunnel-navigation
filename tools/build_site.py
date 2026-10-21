"""Pre-run simulator pages for viewing without Python (GitHub Pages: serve /docs).

    python tools/build_site.py      -> docs/sim/*.html + docs/sim/index.html
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "sim"
RUNS = [("S1", []), ("S2", []), ("S3", []), ("S4", []), ("S5", []), ("S6", []),
        ("S1-1km", []), ("S1-5km", []),
        ("S1-nomap", ["S1", "--eskf", "map_in_tunnel=False"]),
        ("S3-pressure30", ["S3", "--override", "baro_tunnel_30pa"]),
        ("S1-lowend", ["S1", "--override", "imu_lowend"])]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, args in RUNS:
        subprocess.run([sys.executable, str(ROOT / "tools" / "simulate.py"), *(args or [name]),
                        "--out", str(OUT / f"{name}.html")], check=True)
    body = (ROOT / "tools" / "site_index.html").read_text()
    (OUT / "index.html").write_text('<!doctype html><html lang="en"><head><meta charset="utf-8">'
                                    '<meta name="viewport" content="width=device-width, initial-scale=1">' + body + "</html>")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
