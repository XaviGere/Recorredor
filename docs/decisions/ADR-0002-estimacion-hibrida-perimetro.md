# ADR-0002: Estimación Híbrida de Perímetro (Geometría Computacional + Método de las Hormigas / Cauchy-Crofton)

- **Estado:** Aceptado
- **Fecha:** 2026-10-25

## Contexto y Problema
Si el robot solo registra coordenadas $(x_k, y_k)$ de impacto en las paredes y calcula un polígono envolvente, no tiene una medida independiente para saber si ya exploró el interior de forma homogénea o si quedó atrapado rebotando en una sola mitad del salón. Por el contrario, si solo usa el **Método de las Hormigas (Mallock / Buffon)** y la fórmula de **Cauchy-Crofton**, obtiene escalares excelentes para el Área ($\hat{A}_{\text{ants}}$) y el Perímetro ($\hat{P}_{\text{crofton}}$), pero no los vértices cartesianos $(X_m, Y_m)$ necesarios para navegar el perímetro al final sin chocar.

## Decisión
Combinar **ambos métodos en paralelo** como validación cruzada:
1. **Nivel Topológico / Espacial:** El conjunto de impactos $\{P_{\text{wall}, k}\}$ genera el polígono envolvente simplificado $\mathcal{H}_k$, del cual se extraen los vértices de las esquinas, el área geométrica $A_{\text{poly}}$ y el perímetro $L_{\text{poly}}$ (fórmula de Shoelace).
2. **Nivel Estocástico / Integral (Hormigas + Crofton):** Las cuerdas $\{S_k\}$ y sus intersecciones interiores $C_{\text{non-adj}}$ alimentan el estimador de Buffon-Mallock ($\hat{A}_{\text{ants}}$) y el cociente de Cauchy-Crofton ($\bar{l}/\pi \approx A/P$).
3. **Condición de Transición a Navegación:** El robot pasa a recorrer el perímetro cuando el polígono se estabiliza, todos los sectores angulares fueron visitados y el estimador de las hormigas concuerda con el área geométrica.

## Consecuencias
- **Positivas:** Alta robustez matemática; permite demostrar visual y numéricamente cómo el método bio-inspirado de las hormigas converge al área y perímetro reales de la habitación mientras el polígono provee la ruta exacta libre de colisiones.
- **Negativas:** Requiere almacenar el historial de cuerdas en memoria ($\mathcal{O}(N)$ en espacio y $\mathcal{O}(N^2)$ en detección de cruces, lo cual para $N \le 128$ cuerdas toma menos de $0.5\,\text{ms}$ en un ESP32 a $240\,\text{MHz}$).
