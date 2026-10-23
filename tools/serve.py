"""Live simulator server.

    python tools/serve.py            # then open http://localhost:8000
    python tools/serve.py --port 8080

/            the all-in-one simulator page (docs/sim/index.html, built by tools/build_site.py). Served
             from here, it finds /api/ping and shows a LIVE bar: seed, sensors, mount error and filter
             switches re-run truth -> sensors -> ESKF in Python (~7 s) and load the result in place.
/controls    the older control panel with every switch (cruise model, NHC turn gate, re-acquisition).
/api/run     JSON of tools/simulate.run for one parameter set.

Standard library only (http.server); results are cached per parameter set.
"""
import argparse
import html
import json
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import simulate  # noqa: E402
from tunnelsim.scenario import build_scenarios  # noqa: E402

TEMPLATE = (ROOT / "tools" / "simulate_template.html").read_text()
SITE = ROOT / "docs" / "sim" / "index.html"
CACHE: dict[str, str] = {}
LOCK = threading.Lock()

# control -> named overrides (tunnelsim/profiles.py OVERRIDES)
CHOICES = {
    "imu": [("nominal", "Note10 (LSM6DSR) [MEAS]"), ("imu_fwd_bi_xy", "forward bias at x/y level"),
            ("imu_bi_gm1", "single-GM bias model"), ("imu_lowend", "low-end MPU9250 [MEAS]")],
    "vib": [("nominal", "calm"), ("vib_noisy", "noisy (×3)")],
    "baro_tunnel": [("nominal", "none"), ("baro_tunnel_10pa", "10 Pa/km"), ("baro_tunnel_30pa", "30 Pa/km")],
}
GNSS_FLAGS = [("gnss_coasting", "receiver coasts 5 s into the tunnel"), ("gnss_lagging", "lagging fixes at the exit"),
              ("gnss_multipath_10", "exit multipath 10 m"), ("gnss_multipath_50", "exit multipath 50 m"),
              ("gnss_optimistic", "optimistic reported accuracy"), ("gnss_pessimistic", "pessimistic reported accuracy"),
              ("baro_cabin", "cabin pressure disturbance")]


def run_data(q: dict) -> dict:
    key = q.get("scenario", "S1")
    seed = int(q.get("seed", 0))
    overrides = [q[k] for k in ("imu", "vib", "baro_tunnel") if q.get(k, "nominal") != "nominal"]
    overrides += [name for name, _ in GNSS_FLAGS if q.get(name) == "1"]
    overrides += [o for o in q.get("overrides", "").split(",") if o and o not in overrides]
    # live sliders (owner 2026-10-23): world and sensor values as tagged overrides, filter values as
    # EskfConfig fields, speed as a factor on the scenario, map errors on the filter's map
    sliders = {"mp": ("set:gnss.exit_multipath_m", 30.0), "pgrad": ("set:baro.tunnel_grad_pa_per_km", 0.0),
               "viblvl": ("set:vib.level", 1.0), "abi": ("scale:imu.accel_bias_instab", 1.0)}
    for k_, (path, dflt) in sliders.items():
        if k_ in q and float(q[k_]) != dflt:
            overrides.append(f"{path}={float(q[k_]):g}")
    eskf = [f"cruise={q.get('cruise', '1') == '1'}",
            f"cruise_mode={q.get('cruise_mode', 'gm')!r}",
            f"nhc_turn_gate={float(q.get('nhc_turn_gate', 0.3))}",
            f"baro_grad={q.get('baro_grad', '0') == '1'}",
            f"gate_recover={q.get('gate_recover', '1') == '1'}",
            f"adaptive_q={q.get('gate_recover', '1') == '1'}",
            f"reacq={q.get('reacq', '1') == '1'}",
            f"map_in_tunnel={q.get('map_in_tunnel', '1') == '1'}",
            f"cruise_gm_sigma={float(q.get('gm_sigma', 0.5))}",
            f"cruise_gm_tau={float(q.get('gm_tau', 60.0))}"]
    me = float(q.get("mount_err", 0.0))
    mount = (0.0, me, me) if me else None
    map_err = (float(q.get("map_lat", 0.0)), 1.0 + float(q.get("map_len", 0.0)) / 100.0)
    return simulate.run(key, seed, overrides, q.get("zupt", "detector"), eskf, mount, q.get("no_sideslip") == "1",
                        float(q.get("spd", 1.0)), map_err)


def run_page(q: dict) -> str:
    return TEMPLATE.replace("/*DATA*/null", json.dumps(run_data(q)))


def control_page() -> str:
    keys = list(build_scenarios())
    opt = lambda pairs, sel: "".join(f'<option value="{v}"{" selected" if v == sel else ""}>{html.escape(t)}</option>' for v, t in pairs)
    gnss = "".join(f'<label><input type="checkbox" name="{n}" value="1"> {html.escape(t)}</label>' for n, t in GNSS_FLAGS)
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Tunnel simulator</title>
<style>
:root {{ --bg:#f6f5f1; --panel:#fff; --ink:#1d232b; --muted:#6b7380; --line:#d9d6cf; --acc:#c2410c; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#12151a; --panel:#1a1f26; --ink:#e8e6e1; --muted:#98a0ab; --line:#2c333d; --acc:#fb923c; }} }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:13px/1.4 system-ui, sans-serif; display:grid; grid-template-columns:290px 1fr; height:100vh; }}
form {{ padding:14px; overflow:auto; border-right:1px solid var(--line); background:var(--panel); }}
h1 {{ font-size:15px; margin:0 0 4px; }} h2 {{ font-size:11px; text-transform:uppercase; letter-spacing:.06em; color:var(--muted); margin:14px 0 6px; }}
label {{ display:block; margin:5px 0; }} select, input[type=number] {{ width:100%; font:inherit; padding:3px; }}
input[type=range] {{ width:100%; }} .row {{ display:flex; justify-content:space-between; }}
#status {{ margin-top:12px; color:var(--acc); font-weight:600; min-height:18px; }}
iframe {{ width:100%; height:100vh; border:0; background:var(--bg); }}
@media (max-width: 800px) {{ body {{ grid-template-columns:1fr; height:auto; }} iframe {{ height:1600px; }} }}
</style></head><body>
<form id="f">
<h1>Through the Tunnel</h1><div style="color:var(--muted)">Each change re-runs truth → sensors → ESKF (~10 s).</div>
<h2>Scenario</h2>
<label>scenario <select name="scenario">{opt([(k, k) for k in keys], "S1")}</select></label>
<label><div class="row"><span>seed (noise realisation)</span><span id="seed_v">0</span></div><input type="range" name="seed" min="0" max="49" value="0"></label>
<label><input type="checkbox" name="no_sideslip" value="1"> car without sideslip</label>
<h2>Phone sensors</h2>
<label>IMU <select name="imu">{opt(CHOICES["imu"], "nominal")}</select></label>
<label>vibration <select name="vib">{opt(CHOICES["vib"], "nominal")}</select></label>
<label>tunnel pressure gradient <select name="baro_tunnel">{opt(CHOICES["baro_tunnel"], "nominal")}</select></label>
<label><div class="row"><span>filter's mount error (pitch & yaw)</span><span id="mount_err_v">0°</span></div><input type="range" name="mount_err" min="0" max="5" step="0.5" value="0"></label>
{gnss}
<h2>Filter</h2>
<label><input type="checkbox" name="map_in_tunnel" value="1" checked> map in the tunnel (heading + cross-track)</label>
<label><input type="checkbox" name="cruise" value="1" checked> cruise prior</label>
<label>cruise model <select name="cruise_mode">{opt([("gm", "GM speed wander (default)"), ("increment", "speed increment (driver holds speed)")], "gm")}</select></label>
<label><div class="row"><span>lateral NHC turn gate</span><span id="nhc_turn_gate_v">0.3 m/s²</span></div><input type="range" name="nhc_turn_gate" min="0" max="1" step="0.1" value="0.3"></label>
<label><input type="checkbox" name="baro_grad" value="1"> pressure-gradient state (experimental)</label>
<label><input type="checkbox" name="gate_recover" value="1" checked> gate-lock recovery + adaptive noise</label>
<label><input type="checkbox" name="reacq" value="1" checked> exit re-acquisition phase</label>
<label>stops <select name="zupt">{opt([("detector", "IMU stop detector"), ("oracle", "oracle (truth)")], "detector")}</select></label>
<div id="status"></div>
</form>
<iframe id="sim" title="simulator"></iframe>
<script>
const f = document.getElementById("f"), sim = document.getElementById("sim"), st = document.getElementById("status");
const unit = {{ seed: "", mount_err: "°", nhc_turn_gate: " m/s²" }};
function q() {{
  const p = new URLSearchParams();
  for (const el of f.elements) {{
    if (!el.name) continue;
    if (el.type === "checkbox") p.set(el.name, el.checked ? "1" : "0"); else p.set(el.name, el.value);
    const v = document.getElementById(el.name + "_v"); if (v) v.textContent = el.value + (unit[el.name] || "");
  }}
  return p.toString();
}}
let t0 = 0;
function go() {{ st.textContent = "running…"; t0 = performance.now(); sim.src = "/run?" + q(); }}
sim.onload = () => {{ if (sim.src) st.textContent = `done in ${{((performance.now() - t0) / 1000).toFixed(1)}} s`; }};
f.addEventListener("change", go);
f.addEventListener("input", e => {{ if (e.target.type === "range") q(); }});
go();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, body: str, code=200, ctype="text/html; charset=utf-8"):
        b = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            if SITE.exists():
                return self._send(SITE.read_text())
            return self._send("docs/sim/index.html is missing: run python tools/build_site.py, or open /controls", 404, "text/plain")
        if u.path == "/controls":
            return self._send(control_page())
        if u.path == "/api/ping":
            return self._send(json.dumps({"live": True}), ctype="application/json")
        if u.path in ("/run", "/api/run"):
            q = {k: v[-1] for k, v in parse_qs(u.query).items()}
            key = u.path + json.dumps(q, sort_keys=True)
            try:
                with LOCK:
                    if key not in CACHE:
                        t = time.time()
                        CACHE[key] = run_page(q) if u.path == "/run" else json.dumps(run_data(q))
                        print(f"run {q.get('scenario')} seed {q.get('seed')} {q.get('overrides', '')}: {time.time() - t:.1f} s", flush=True)
                return self._send(CACHE[key], ctype="text/html; charset=utf-8" if u.path == "/run" else "application/json")
            except Exception:
                print(traceback.format_exc(), flush=True)
                return self._send(f"<pre>{html.escape(traceback.format_exc())}</pre>", 500)
        self._send("not found", 404, "text/plain")

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"open http://localhost:{a.port}  (Ctrl+C to stop)")
    srv.serve_forever()


if __name__ == "__main__":
    main()
