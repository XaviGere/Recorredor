# ADR-0003: Memoria Estática Acotada y Telemetría JSONL en ESP32

- **Estado:** Aceptado
- **Fecha:** 2026-10-25

## Contexto y Problema
El firmware en C++ corre sobre un microcontrolador ESP32 sin sistema operativo de escritorio. El uso de `std::vector` con realocaciones frecuentes o concatenación de `String` dinámicos durante meses/horas de operación puede fragmentar el heap. Además, necesitamos que la PC pueda graficar exactamente lo mismo tanto desde la simulación como desde el robot físico.

## Decisión (Confirmada por el Usuario)
1. **Buffers estáticos de tamaño fijo en el ESP32:** Se reservan en tiempo de compilación arreglos de `MAX_CHORDS = 128` elementos (`Point2D wall_points[128]`, `Chord chords[128]`, `Point2D hull_vertices[32]`, `Point2D safe_waypoints[32]`). Esto ocupa menos de $4\,\text{KB}$ de los $520\,\text{KB}$ de RAM del ESP32 y elimina por completo la fragmentación de memoria.
2. **Telemetría en Tiempo Real vía WiFi UDP (`Puerto 5005`) + Serial (`115200 bps`):** El ESP32 se conecta a la red WiFi local y transmite datagramas UDP en formato JSON (`broadcastTelemetry()` cada $100\,\text{ms}$ y en cada impacto de pared). El servidor `sim/live_server.py` en la PC escucha ese puerto UDP para dibujar en vivo el movimiento del robot físico exactamente en el mismo dashboard web que la simulación.

## Consecuencias
- **Positivas:** Ejecución en tiempo real determinista ($\Delta t = 20\,\text{ms}$) sin pausas de recolección de basura ni desbordes de pila.
- **Negativas:** Si una exploración supera las 128 cuerdas, el buffer opera en ventana deslizante o finaliza la fase de exploración para pasar al recorrido perimetral (en la práctica, un recinto simple converge en $18$ a $35$ cuerdas).
