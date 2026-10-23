// Experimental in-browser live mode (docs/sim/browser.html): runs the same Python package as
// tools/serve.py inside a Web Worker with Pyodide (CPython compiled to WebAssembly), so the LIVE
// controls work on a static host such as GitHub Pages. First load downloads ~30 MB.
const PYODIDE = "https://cdn.jsdelivr.net/pyodide/v0.26.4/full/";
let ready = null;
async function boot() {
  postMessage({ type: "status", text: "loading Python (Pyodide)…" });
  importScripts(PYODIDE + "pyodide.js");
  const py = await loadPyodide({ indexURL: PYODIDE });
  postMessage({ type: "status", text: "loading numpy and scipy…" });
  await py.loadPackage(["numpy", "scipy"]);
  postMessage({ type: "status", text: "loading the simulator…" });
  const buf = await (await fetch("tunnelsim.zip")).arrayBuffer();
  py.unpackArchive(buf, "zip", { extractDir: "/tn" });
  py.runPython(`
import sys, json
sys.path.insert(0, "/tn"); sys.path.insert(0, "/tn/tools")
import serve
def _run(q):
    return json.dumps(serve.run_data(dict(q)))
`);
  postMessage({ type: "status", text: "" });
  return py;
}
onmessage = async (e) => {
  const { id, kind, query } = e.data;
  try {
    if (!ready) ready = boot();
    const py = await ready;
    if (kind === "ping") { postMessage({ id, ok: true, body: JSON.stringify({ live: true, engine: "pyodide" }) }); return; }
    const q = py.toPy(Object.fromEntries(new URLSearchParams(query)));
    const t0 = performance.now();
    const body = py.globals.get("_run")(q);
    q.destroy();
    postMessage({ id, ok: true, body, ms: performance.now() - t0 });
  } catch (err) {
    postMessage({ id, ok: false, body: String(err && err.message || err) });
  }
};
