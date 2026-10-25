"""One self-contained simulator page with every scenario as a tab (no Python needed to view).

    python tools/build_site.py            -> docs/sim/index.html (GitHub Pages: serve /docs)

Each tab is a pre-run drive (seed 0) with the map layer on and off; the callout margin k and the
callout distance are sliders evaluated in the browser (ŝ and σ_s are in the page). Filter or sensor
changes beyond these need the local server (tools/serve.py).
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT))

# key, tab label, scenario, overrides, both map variants?, failure case?
TABS = [
    ("S1", "S1 straight 3 km", "S1", [], True, False),
    ("S2", "S2 S-bend", "S2", [], True, False),
    ("S3", "S3 V-profile", "S3", [], True, False),
    ("S4", "S4 stop-and-go", "S4", [], True, False),
    ("S5", "S5 braking", "S5", [], True, False),
    ("S6", "S6 speed wander", "S6", [], True, False),
    ("S1-1km", "1 km", "S1-1km", [], True, False),
    ("S1-5km", "5 km", "S1-5km", [], True, False),
    ("S3-pressure30", "pressure gradient 30 Pa/km", "S3", ["baro_tunnel_30pa"], False, True),
    ("S1-lowend", "low-end IMU", "S1", ["imu_lowend"], False, True),
    ("S4-lowend", "low-end stop-and-go", "S4", ["imu_lowend"], False, True),
]


def _run(args):
    import simulate
    scen, overrides, map_on = args
    return simulate.run(scen, 0, overrides, "detector", [f"map_in_tunnel={map_on}"])


def main():
    jobs, keys = [], []
    for key, _, scen, ov, both, _ in TABS:
        for v, on in (("map", True), ("nomap", False)) if both else (("map", True),):
            jobs.append((scen, ov, on)); keys.append((key, v))
    with ProcessPoolExecutor() as ex:
        data = list(ex.map(_run, jobs))
    runs = {key: {"label": label, "risk": risk, "scen": scen, "overrides": ov, "variants": {}} for key, label, scen, ov, _, risk in TABS}
    for (key, v), d in zip(keys, data):
        runs[key]["variants"][v] = d
    allj = {"order": [t[0] for t in TABS], "runs": runs}
    tpl = (ROOT / "tools" / "simulate_template.html").read_text()
    first = runs[TABS[0][0]]["variants"]["map"]
    page = tpl.replace("/*DATA*/null", json.dumps(first)).replace("/*ALLDATA*/null", json.dumps(allj))
    page = page.replace("<title>Through the Tunnel</title>", "<title>Tunnel Simulator</title>")
    out = ROOT / "docs" / "sim"
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.html"):
        old.unlink()   # browser.html is rewritten below
    (out / "index.html").write_text(page)
    print(f"wrote {out / 'index.html'} ({len(page) / 1e6:.1f} MB)")
    # experimental in-browser live mode (owner 2026-10-24): same page, the LIVE controls answered by
    # Pyodide in a Web Worker; index.html is unchanged
    import zipfile
    shim = (ROOT / "tools" / "browser" / "shim.html").read_text()
    bpage = page.replace("<title>Tunnel Simulator</title>", "<title>Tunnel Simulator (in-browser)</title>")
    i = bpage.index("<body") ; i = bpage.index(">", i) + 1
    bpage = bpage[:i] + "\n" + shim + bpage[i:]
    (out / "browser.html").write_text(bpage)
    (out / "py").mkdir(exist_ok=True)
    (out / "py" / "worker.js").write_text((ROOT / "tools" / "browser" / "worker.js").read_text())
    with zipfile.ZipFile(out / "py" / "tunnelsim.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted((ROOT / "tunnelsim").glob("*.py")):
            z.write(f, f"tunnelsim/{f.name}")
        for f in ("simulate.py", "serve.py", "simulate_template.html"):
            z.write(ROOT / "tools" / f, f"tools/{f}")
    print(f"wrote {out / 'browser.html'} + py/ (experimental Pyodide live mode)")


if __name__ == "__main__":
    main()
