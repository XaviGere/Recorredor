// Worker de Pyodide para Recorredor (GitHub Pages).
// Carga Python en WebAssembly, monta el paquete sim/ desde los .py servidos
// estáticos junto a index.html, y responde RPC: {id, op, params} -> {id, json|error}.
const PYODIDE_URL = "https://cdn.jsdelivr.net/pyodide/v0.26.4/full/";

const SIM_FILES = [
  "sim/__init__.py",
  "sim/config.py",
  "sim/world.py",
  "sim/web_api.py",
  "sim/core/__init__.py",
  "sim/core/geometry.py",
  "sim/core/estimators.py",
  "sim/core/navigator.py",
  "sim/core/odometry.py",
];

let pyodide = null;

async function boot() {
  postMessage({ type: "progress", msg: "Cargando Pyodide (WebAssembly)..." });
  importScripts(PYODIDE_URL + "pyodide.js");
  pyodide = await loadPyodide({ indexURL: PYODIDE_URL });

  postMessage({ type: "progress", msg: "Cargando simulador Python..." });
  for (const f of SIM_FILES) {
    const resp = await fetch(f);
    if (!resp.ok) throw new Error("No se pudo cargar " + f + " (" + resp.status + ")");
    const text = await resp.text();
    const parts = f.split("/");
    parts.pop();
    let dir = "";
    for (const d of parts) {
      dir += "/" + d;
      try { pyodide.FS.mkdir(dir); } catch (e) { /* ya existe */ }
    }
    pyodide.FS.writeFile("/" + f, text);
  }

  await pyodide.runPythonAsync(
    "import sys; sys.path.insert(0, '/'); import sim.web_api"
  );
  postMessage({ type: "ready" });
}

self.onmessage = async (e) => {
  const { id, op, params } = e.data;
  try {
    const jsonOut = await pyodide.runPythonAsync(
      "import json, sim.web_api as _w\n" +
      "_w.handle(" + JSON.stringify(op) + ", " + JSON.stringify(JSON.stringify(params || {})) + ")"
    );
    postMessage({ id, json: jsonOut });
  } catch (err) {
    postMessage({ id, error: String(err) });
  }
};

boot().catch((err) => postMessage({ type: "boot_error", error: String(err) }));
