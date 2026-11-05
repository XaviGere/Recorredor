"""Build de la versión GitHub Pages (Pyodide/WASM) de Recorredor.

Genera/actualiza web/:
  - web/sim/*.py       copias de los módulos stdlib-only del simulador
  - web/index.html     HTML_DASHBOARD extraído de sim/live_server.py con el
                       shim local_api.js inyectado antes de </head>

Los archivos web/local_api.js, web/pyworker.js y web/sim/web_api.py son
mantenidos a mano (no se sobrescriben).

Uso:  python tools/build_web.py
Después: publicar web/ en GitHub Pages (Settings > Pages > carpeta /docs o
rama gh-pages renombrada a web).
"""
import os
import re
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIM_SRC = os.path.join(ROOT, "sim")
WEB = os.path.join(ROOT, "web")
WEB_SIM = os.path.join(WEB, "sim")

# Solo los módulos que Pyodide puede correr (stdlib puro: math/random/dataclasses)
SIM_FILES = [
    "__init__.py",
    "config.py",
    "world.py",
    "core/__init__.py",
    "core/geometry.py",
    "core/estimators.py",
    "core/navigator.py",
    "core/odometry.py",
]

INJECT = '<script src="local_api.js"></script>'


def copy_sim_files():
    for rel in SIM_FILES:
        src = os.path.join(SIM_SRC, rel)
        dst = os.path.join(WEB_SIM, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src, dst)
        print(f"  copiado sim/{rel}")


def build_index():
    src = os.path.join(SIM_SRC, "live_server.py")
    code = open(src, encoding="utf-8").read()
    m = re.search(r'HTML_DASHBOARD = """(.*?)"""', code, re.DOTALL)
    if not m:
        sys.exit("No se encontró HTML_DASHBOARD en live_server.py")
    html = m.group(1)
    if INJECT in html:
        print("  index.html ya contiene el shim")
    else:
        html = html.replace("</head>", "  " + INJECT + "\n</head>", 1)
    # Aviso visual de que corre 100% en el navegador (WASM), sin backend
    badge = (
        '<div style="position:fixed;bottom:8px;right:10px;font-size:0.68rem;'
        'color:#64748b;background:#0f172acc;padding:3px 8px;border-radius:6px;'
        'border:1px solid #334155;z-index:50">'
        "GitHub Pages &middot; Python vía Pyodide/WASM &middot; sin servidor</div>"
    )
    if "Pyodide/WASM" not in html:
        html = html.replace("</body>", badge + "\n</body>", 1)
    out = os.path.join(WEB, "index.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"  escrito web/index.html ({len(html)} bytes)")


def main():
    os.makedirs(os.path.join(WEB_SIM, "core"), exist_ok=True)
    copy_sim_files()
    build_index()
    print("Listo — publicar el contenido de web/ en GitHub Pages.")


if __name__ == "__main__":
    main()
