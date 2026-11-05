// Shim de "backend" para Recorredor en GitHub Pages (Pyodide).
// Reemplaza window.fetch: las llamadas /api/* se resuelven con un Worker
// Pyodide (simulación real en Python/WASM) o con localStorage (shapes/runs).
// Instalado ANTES del <script> del dashboard: ver web/index.html.
(function () {
  const SHAPES_KEY = "recorredor_shapes";
  const RUNS_KEY = "recorredor_runs";

  // ------------------- Worker Pyodide (RPC con correlación) ----------------
  const worker = new Worker("pyworker.js");
  let nextId = 1;
  const pending = {};
  let readyResolve;
  const readyP = new Promise((res) => (readyResolve = res));
  let bootMsg = "Iniciando Pyodide...";

  worker.onmessage = (e) => {
    const m = e.data;
    if (m.type === "ready") {
      readyResolve();
      const b = document.getElementById("fsmBadge");
      if (b && b.textContent.includes("WASM")) b.textContent = "LISTO";
    } else if (m.type === "progress") {
      bootMsg = m.msg;
      const b = document.getElementById("fsmBadge");
      if (b) b.textContent = "WASM: " + m.msg;
    } else if (m.type === "boot_error") {
      const b = document.getElementById("fsmBadge");
      if (b) b.textContent = "ERROR Pyodide: " + m.error;
      console.error("Pyodide boot error:", m.error);
    } else if (pending[m.id]) {
      pending[m.id](m);
      delete pending[m.id];
    }
  };

  function rpc(op, params) {
    return new Promise((resolve, reject) => {
      const id = nextId++;
      pending[id] = (m) => {
        if (m.error) reject(new Error(m.error));
        else {
          const parsed = JSON.parse(m.json);
          if (parsed && parsed.error) reject(new Error(parsed.error));
          else resolve(parsed);
        }
      };
      worker.postMessage({ id, op, params });
    });
  }

  // --------------------------- localStorage --------------------------------
  function loadShapes() {
    try { return JSON.parse(localStorage.getItem(SHAPES_KEY)) || {}; }
    catch (e) { return {}; }
  }
  function saveShapes(s) { localStorage.setItem(SHAPES_KEY, JSON.stringify(s)); }
  function loadRuns() {
    try { return JSON.parse(localStorage.getItem(RUNS_KEY)) || []; }
    catch (e) { return []; }
  }
  function saveRuns(r) { localStorage.setItem(RUNS_KEY, JSON.stringify(r)); }

  function jsonResp(obj, status) {
    return new Response(JSON.stringify(obj), {
      status: status || 200,
      headers: { "Content-Type": "application/json; charset=utf-8" },
    });
  }

  // ------------------------------ fetch shim --------------------------------
  const realFetch = window.fetch.bind(window);

  window.fetch = async function (url, opts) {
    const u = new URL(url, location.href);
    const method = (opts && opts.method) || "GET";

    if (!u.pathname.startsWith("/api/")) return realFetch(url, opts);

    // ---- Persistencia (no requiere Pyodide) ----
    if (u.pathname === "/api/shapes" && method === "GET") {
      return jsonResp({ shapes: loadShapes() });
    }
    if (u.pathname === "/api/runs" && method === "GET") {
      return jsonResp({ runs: loadRuns() });
    }
    if (u.pathname === "/api/runs" && method === "POST") {
      const rec = JSON.parse(opts.body || "{}");
      const runs = loadRuns();
      if (!runs.some((r) => r.seed === rec.seed)) {
        rec.run_number = runs.length + 1;
        rec.saved_at = new Date().toLocaleTimeString();
        runs.push(rec);
        if (runs.length > 120) runs.splice(0, runs.length - 120);
        saveRuns(runs);
      }
      return jsonResp({ ok: true, runs });
    }
    if (u.pathname === "/api/runs_clear" && method === "POST") {
      saveRuns([]);
      return jsonResp({ ok: true, runs: [] });
    }
    if (u.pathname === "/api/runs_delete" && method === "POST") {
      const data = JSON.parse(opts.body || "{}");
      const runs = loadRuns().filter((r) => r.run_number !== Number(data.run_number));
      saveRuns(runs);
      return jsonResp({ ok: true, runs });
    }
    if (u.pathname === "/api/esp32_live") {
      return jsonResp({ packets: [] });   // sin hardware en la versión web
    }

    // ---- Operaciones que requieren el simulador Python/WASM ----
    await readyP;

    if (u.pathname === "/api/shapes" && method === "POST") {
      const data = JSON.parse(opts.body || "{}");
      const name = (data.name || "Figura_Personalizada").trim() || "Figura_Personalizada";
      const norm = await rpc("normalize_shape", {
        vertices: data.vertices || [],
        mode: data.mode || "classic",
        name: name,
      });
      const shapes = loadShapes();
      shapes[name] = norm.normalized_vertices;
      saveShapes(shapes);
      return jsonResp({ ...norm, shapes });
    }

    if (u.pathname === "/api/sim" && method === "GET") {
      const q = u.searchParams;
      const roomType = q.get("room") || "rect";
      const params = {
        room_type: roomType,
        mode: q.get("mode") || "classic",
        max_chords: Number(q.get("max_chords") || 32),
        max_relearns: Number(q.get("max_relearns") || 15),
        turn_mode: q.get("turn_mode") || "random",
        fixed_turn_deg: Number(q.get("fixed_turn_deg") || 135.0),
        turn_seq_str: q.get("turn_seq_str") || "105, 135, 155",
        test_stride_m: Number(q.get("test_stride_m") || 0.42),
        test_corner_pass_m: Number(q.get("test_corner_pass_m") || 0.12),
        early_stop_perimeter: ["1", "true", "True"].includes(q.get("early_stop")),
      };
      const seed = q.get("seed");
      if (seed !== null && seed !== "") params.seed = Number(seed);
      // Figuras guardadas: resolver vértices desde localStorage
      if (roomType.startsWith("saved:")) {
        const nm = roomType.slice(6);
        const verts = loadShapes()[nm];
        if (verts && verts.length >= 3) {
          params.custom_vertices = verts;
          params.custom_name = nm;
        } else {
          params.room_type = "rect";
        }
      }
      try {
        return jsonResp(await rpc("sim", params));
      } catch (e) {
        return jsonResp({ error: String(e.message || e) }, 500);
      }
    }

    if (u.pathname === "/api/sim_custom" && method === "POST") {
      const data = JSON.parse(opts.body || "{}");
      const params = {
        room_type: "custom",
        mode: data.mode || "classic",
        max_chords: Number(data.max_chords || 32),
        max_relearns: Number(data.max_relearns || 15),
        turn_mode: data.turn_mode || "random",
        fixed_turn_deg: Number(data.fixed_turn_deg || 135.0),
        turn_seq_str: data.turn_seq_str || "105, 135, 155",
        test_stride_m: Number(data.test_stride_m || 0.42),
        test_corner_pass_m: Number(data.test_corner_pass_m || 0.12),
        early_stop_perimeter: !!data.early_stop,
        custom_vertices: data.vertices || [],
      };
      if (data.seed !== undefined && data.seed !== null) params.seed = Number(data.seed);
      try {
        return jsonResp(await rpc("sim", params));
      } catch (e) {
        return jsonResp({ error: String(e.message || e) }, 500);
      }
    }

    return jsonResp({ error: "Endpoint no soportado en la versión web: " + u.pathname }, 404);
  };

  console.log("[Recorredor WASM] fetch shim instalado — " + bootMsg);
})();
