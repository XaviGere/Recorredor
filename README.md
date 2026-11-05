# Recorredor — Robot Autónomo de Mapeo Perimetral y Estimación Estocástica

Proyecto de robótica móvil basado en **ESP32 (C++/Arduino)** y **Simulador Físico 2D (Python)** que explora una habitación cerrada mediante trayectorias rectilíneas (cuerdas), reconstruye el mapa perimetral combinando **Geometría Computacional** y el **Método de las Hormigas (Mallock / Buffon / Cauchy-Crofton)**, y finalmente recorre todo el perímetro de forma autónoma sin colisionar.

## Estructura del Repositorio

```text
Recorredor/
├── AGENTS.md                                  # Guía rápida para agentes e ingenieros
├── AgentRules.md                              # Reglas de arquitectura, física y código
├── README.md                                  # Este archivo
├── docs/
│   ├── Recorredor-spec.md                     # Especificación funcional, cinemática y FSM
│   ├── Recorredor-matematicas-perimetro.md    # Fórmulas: Odometría, Hormigas (Mallock/Crofton), Hull y Offset
│   ├── Recorredor-electronica-y-circuito.md   # Esquema de conexiones ESP32, BOM y protecciones
│   ├── Recorredor-circuito-tinkercad-arduino.md # Wiring Arduino UNO + L293D + SG90 + 2×HC-SR04 (Tinkercad)
│   ├── Recorredor-simulador-guia-y-modos.md   # Guía del simulador web, los 7 modos y comparativa
│   ├── Recorredor-plan-implementacion.md      # Plan maestro de implementación por fases
│   └── decisions/
│       ├── ADR-0001-cinematica-y-direccion.md
│       ├── ADR-0002-estimacion-hibrida-perimetro.md
│       └── ADR-0003-protocolo-telemetria-serial.md
├── sim/                                       # Simulador en Python (Core + Mundo 2D + Visualización)
│   ├── __init__.py
│   ├── config.py                              # Parámetros físicos y de calibración
│   ├── world.py                               # Entorno poligonal 2D, raycasting y colisiones
│   ├── visualizer.py                          # Renderizado en Matplotlib / SVG / ASCII
│   ├── run_demo.py                            # Script principal de demostración fin a fin
│   ├── core/
│   │   ├── __init__.py
│   │   ├── geometry.py                        # Vectores, intersección de segmentos, Convex Hull, Inward Offset
│   │   ├── estimators.py                      # Método de las Hormigas (Mallock/Buffon/Crofton) + Shoelace
│   │   ├── odometry.py                        # Integración cinemática (t, v, theta -> x, y)
│   │   └── navigator.py                       # Máquina de estados (FSM) y seguidor Pure Pursuit
│   └── tests/
│       └── test_recorredor.py                 # Suite de tests matemáticos y de misión completa
└── firmware/
    └── recorredor_esp32/                      # Firmware Arduino/ESP32 listo para compilar
        ├── recorredor_esp32.ino               # Punto de entrada setup() y loop() no bloqueante
        ├── config.h                           # Pinout ESP32 y constantes de calibración
        ├── hal_esp32.h                        # Abstracción de motores, servo, ultrasonido, bumpers e IMU
        ├── geometry_fixed.h                   # Geometría 2D, cruces de cuerdas, Hull y Offset en C++
        └── navigator_fsm.h                    # Máquina de estados de exploración y recorrido perimetral
    └── recorredor_tinkercad_uno/              # Sketch de UN SOLO archivo para Arduino UNO en Tinkercad
        └── recorredor_tinkercad_uno.ino       # Solo modo Recto + Reaprendizaje (L293D + SG90 + 2×HC-SR04)
    └── recorredor_tinkercad_nano/             # Misma lógica para Arduino Nano (componentes básicos Tinkercad)
        └── recorredor_tinkercad_nano.ino      # Solo modo Recto + Reaprendizaje
    └── recorredor_thin_client/                # Firmware "tonto": solo motores + sensores
        └── recorredor_thin_client.ino         # Serial (UNO/Nano) o WiFi UDP (ESP32); el cerebro corre en PC
├── tools/
│   ├── recorredor_brain.py                    # Cerebro PC: FSM recto+reaprend. por UDP/Serial (thin client)
│   ├── draw_tinkercad_nano_circuit.py         # Genera el PNG del circuito Nano en docs/
│   └── build_web.py                           # Build de la versión GitHub Pages (Pyodide/WASM)
├── web/                                       # Versión estática del simulador para GitHub Pages
│   ├── index.html                             # Dashboard completo (generado por tools/build_web.py)
│   ├── local_api.js                           # Shim: fetch /api/* → Worker Pyodide o localStorage
│   ├── pyworker.js                            # Web Worker: Pyodide/WASM carga sim/ y ejecuta runs
│   └── sim/                                   # Copia stdlib-only de sim/ + web_api.py (RPC del worker)
```

## Inicio Rápido (Simulación en Python)

Desde la carpeta `Recorredor`:

```powershell
# 1. Ejecutar todos los tests unitarios y de integración
python -m unittest discover -s sim/tests -p "test_*.py" -v

# 2. Ejecutar demostración completa y generar reporte + mapa visual
python -m sim.run_demo --headless --save-plot mapa_perimetro.png
```
