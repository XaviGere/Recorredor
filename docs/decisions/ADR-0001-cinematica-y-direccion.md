# ADR-0001: Cinemática Híbrida (Tracción Diferencial + Servomotor de Dirección/Escaneo)

- **Estado:** Aceptado
- **Fecha:** 2026-10-25

## Contexto y Problema
El robot necesita avanzar en línea recta con precisión, frenar ante una pared, girar un ángulo exacto $\Delta\theta$ y volver a arrancar. Un chasis puramente tipo auto (Ackermann con servo delantero) tiene un radio mínimo de giro $R_{\min} = L_{\text{wb}} / \tan(\phi_{\max}) > 0$, lo que lo obliga a realizar múltiples maniobras de marcha atrás en las esquinas y complica mantener el centro de rotación fijo al medir las cuerdas. Por otro lado, el requerimiento contempla el uso de servomotores para asistir el cambio de dirección o la orientación de sensado.

## Decisión (Confirmada por el Usuario)
Se adopta definitivamente la configuración **Chasis de 2 ruedas motrices + rueda loca (*caster*) + servomotor** y **estimación cinemática directa de distancia ($d = v \cdot t$)**:
1. **Dos motores DC independientes en el eje principal** (controlados por el puente H TB6612FNG) que permiten tanto avance recto como giro puro sobre el propio eje en las esquinas sin dar marcha atrás, apoyados en una **rueda loca** de baja fricción.
2. **Un servomotor MG90S/SG90** que orienta el cabezal direccional / sensor de proximidad frontal.
3. **Odometría por tiempo y velocidad ($d = (k_{\text{cal}} \cdot v_{\text{cmd}}) \cdot \Delta t$):** En lugar de depender obligatoriamente de encoders ópticos/magnéticos en las ruedas, el desplazamiento $\Delta s$ se calcula integrando la velocidad lineal calibrada por el intervalo de tiempo $\Delta t$ (`USE_OPEN_LOOP_VT_ODOM = true` por defecto en `config.h`).

## Consecuencias
- **Positivas:** El mismo código y circuito soportan ambos armados mecánicos cambiando una constante en `config.h` / `config.py`. En modo híbrido, las cuerdas $S_k$ son líneas rectas perfectas de pared a pared sin arcos parásitos de maniobra.
- **Negativas:** Requiere un canal PWM adicional de $50\,\text{Hz}$ en el ESP32 (`GPIO 18`) y alimentación regulada de $5\,\text{V}$ para el servo.
