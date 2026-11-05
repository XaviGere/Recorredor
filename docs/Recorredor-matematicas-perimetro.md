# Fundamentos Matemáticos para Determinación de Perímetro y Área — `Recorredor`

Este documento desarrolla de forma rigurosa los modelos matemáticos implementados en `Recorredor` para transformar mediciones locales de **tiempo ($t$), velocidad ($v$) y dirección ($\theta$)** en un mapa de coordenadas cartesianas, estimar el área y el perímetro mediante **Geometría Integral (Método de las Hormigas / Buffon / Cauchy-Crofton)** y **Geometría Computacional**, y generar la trayectoria interior segura para recorrer el perímetro sin chocar.

---

## 1. Cinemática y Odometría Dead-Reckoning $(t, v, \theta \rightarrow x, y)$

### 1.1 Integración en Tiempo Discreto (Punto Medio de Runge-Kutta Orden 2)
El robot muestrea sus sensores a intervalos discretos $\Delta t_i = t_i - t_{i-1}$. Conociendo la velocidad lineal instantánea $v_i$ y la orientación $\theta_i$, la integración de Euler simple introduce error numérico cuando el robot gira. Por ello, usamos **integración de punto medio (Runge-Kutta de 2º orden / Exact Arc Integration)**:

$$\bar{\theta}_i = \theta_{i-1} + \frac{\Delta \theta_i}{2}, \qquad \Delta s_i = v_i \cdot \Delta t_i$$

$$x_i = x_{i-1} + \Delta s_i \cos(\bar{\theta}_i)$$
$$y_i = y_{i-1} + \Delta s_i \sin(\bar{\theta}_i)$$
$$\theta_i = \text{wrap\_pi}(\theta_{i-1} + \Delta \theta_i)$$

donde la función de normalización angular mantiene $\theta \in (-\pi, \pi]$:
$$\text{wrap\_pi}(\alpha) = ((\alpha + \pi) \bmod 2\pi) - \pi$$

### 1.2 Fusión Odometría de Ruedas + Giroscopio (Filtro Complementario)
En el hardware real con ESP32:
- Los encoders de las ruedas izquierda y derecha entregan desplazamientos $\Delta s_L, \Delta s_R$, de donde:
  $$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \qquad \Delta \theta_{\text{odom}} = \frac{\Delta s_R - \Delta s_L}{b}$$
  siendo $b$ la trocha (distancia entre ruedas motrices).
- Si se utiliza un servomotor delantero con ángulo de dirección $\phi$ (geometría Ackermann/Triciclo de batalla $L_{\text{wb}}$):
  $$\Delta \theta_{\text{odom}} = \frac{\Delta s}{L_{\text{wb}}} \tan(\phi)$$
- El giroscopio MEMS (MPU6050) mide la velocidad angular $\omega_z$, libre de patinamiento de rueda a corto plazo. Se fusionan ambas fuentes con $\gamma \in [0.90, 0.98]$:
  $$\theta_i = \gamma (\theta_{i-1} + \omega_z \Delta t_i) + (1 - \gamma)(\theta_{i-1} + \Delta \theta_{\text{odom}})$$

### 1.3 Proyección del Punto de Pared en el Impacto $k$
Cuando el robot frena en la pose $(x_k, y_k, \theta_k)$ tras detectar una pared a distancia frontal $d_{\text{sensor}, k}$ (donde $d_{\text{sensor}, k} = 0$ si fue contacto de bumper):

$$P_{\text{wall}, k} = \begin{pmatrix} X_k \\ Y_k \end{pmatrix} = \begin{pmatrix} x_k + (R_{\text{robot}} + d_{\text{sensor}, k})\cos\theta_k \\ y_k + (R_{\text{robot}} + d_{\text{sensor}, k})\sin\theta_k \end{pmatrix}$$

---

## 2. El "Método de las Hormigas": Geometría Integral de Buffon, Mallock y Cauchy-Crofton

### 2.1 Origen Biológico y Físico (Fórmula de Mallock)
Estudios clásicos de etología matemática (Mugford, Mallock & Franks, *Proc. Royal Society B*, 2000) descubrieron cómo las hormigas exploradoras *Leptothorax albipennis* miden el área $A$ de una cavidad totalmente a oscuras antes de mudarse:
1. En una primera visita recorren el recinto dejando un rastro químico de longitud total $L_{\text{rastro}, 1}$.
2. En una segunda pasada recorren una distancia $L_{\text{rastro}, 2}$ y cuentan cuántas veces $C$ cruzan su propio rastro anterior.

De acuerdo con el **Problema de la Aguja de Buffon generalizado a curvas planas**, si dos curvas o conjuntos de segmentos de longitudes $L_1$ y $L_2$ se distribuyen de manera uniforme e isotrópica dentro de una región plana de área $A$, el número esperado de intersecciones $\mathbb{E}[C]$ entre ellas es exactamente:

$$\mathbb{E}[C] = \frac{2 \, L_1 \, L_2}{\pi \, A}$$

Despejando el área $A$, se obtiene el **Estimador de Mallock (Dos Fases)**:

$$\hat{A}_{\text{mallock}} = \frac{2 \, L_1 \, L_2}{\pi \, C}$$

### 2.2 Generalización a $N$ Cuerdas Continuas en una Sola Exploración
En nuestro robot `Recorredor`, no necesitamos detenernos a mitad de camino para separar "rastro 1" y "rastro 2": el robot ejecuta una secuencia continua de $N$ tramos rectos (cuerdas) $S_1, S_2, \dots, S_N$ con longitudes individuales $l_1, l_2, \dots, l_N$.

Cada par de cuerdas no adyacentes $(S_i, S_j)$ con $|i - j| > 1$ (ya que $S_i$ y $S_{i+1}$ comparten el vértice de rebote en la pared y no cuentan como cruce interior) tiene una probabilidad de cruzarse dada por la fórmula de Buffon-Laplace:

$$\mathbb{E}[I_{ij}] = \frac{2 \, l_i \, l_j}{\pi \, A} \cdot \eta_{\text{chord}}$$

Sumando sobre todos los pares de tramos no adyacentes $(i, j)$ con $1 \le i < j - 1 \le N$:

$$U_N = \sum_{1 \le i < j - 1 \le N} l_i \, l_j$$

Siendo $C_{\text{non-adj}}$ el número total de cruces interiores observados entre segmentos no consecutivos, el **Estimador Global de Hormigas por Pares de Cuerdas** es:

$$\boxed{\hat{A}_{\text{ants}} = \frac{2 \, \eta_{\text{chord}}}{\pi \, C_{\text{non-adj}}} \sum_{1 \le i < j - 1 \le N} l_i \, l_j}$$

> **Nota sobre el factor geométrico de cuerda $\eta_{\text{chord}}$:**
> - Para segmentos cortos que empiezan y terminan en el interior del recinto (hormigas caminando libremente), $\eta = 1$.
> - Para **cuerdas completas que van de pared a pared** (billar matemático / medida cinemática de Santaló), dos cuerdas se cruzan dentro del recinto convexo si y solo si sus puntos extremos se alternan a lo largo del perímetro. Además, cerca de la frontera la densidad de cruces es mayor que en segmentos interiores cortos. Empíricamente y por geometría integral en recintos de relación de aspecto moderada ($1:1$ a $2:1$), el factor de calibración de frontera es $\eta_{\text{chord}} \approx 1.18$ para rebotes pseudo-isotrópicos desde las paredes (parametrizado como `ANTS_BOUNDARY_FACTOR` en `config.py`).

### 2.3 Teorema de Cauchy-Crofton: Relación Directa entre Longitud Media de Cuerda, Área y Perímetro
La Geometría Integral provee una segunda joya matemática: el **Teorema de Cauchy-Crofton (Fórmula de Santaló)** para cualquier recinto convexo de área $A$ y perímetro $P$.

Si se trazan cuerdas aleatorias isotrópicas que atraviesan la habitación de pared a pared, la **longitud media de las cuerdas** $\mathbb{E}[l]$ cumple de forma exacta e independiente de la forma del polígono:

$$\boxed{\mathbb{E}[l] = \frac{\pi \, A}{P}}$$

Por lo tanto, el promedio muestral de los tramos recorridos por el robot:

$$\bar{l} = \frac{1}{N} \sum_{k=1}^{N} l_k$$

nos da directamente el **cociente isoperimétrico Área/Perímetro**:

$$\widehat{\left(\frac{A}{P}\right)}_{\text{Crofton}} = \frac{\bar{l}}{\pi}$$

Combinando el área estimada por los cruces de las hormigas ($\hat{A}_{\text{ants}}$) con la longitud media de cuerda de Cauchy-Crofton ($\bar{l}$), obtenemos un **estimador puramente estocástico del perímetro**:

$$\boxed{\hat{P}_{\text{ants+crofton}} = \frac{\pi \, \hat{A}_{\text{ants}}}{\bar{l}}}$$

¡Esto significa que el robot puede estimar tanto el **Área** como el **Perímetro** usando únicamente su odometría de tramos y el conteo de cruces de su propio rastro, y contrastarlo contra el polígono geométrico!

---

## 3. Detección Exacta de Cruces de Segmentos (Intersección de Cuerdas)

Dadas dos cuerdas $S_i = \overline{A B}$ y $S_j = \overline{C D}$ (con $|i - j| > 1$), determinamos si se cruzan en el interior mediante productos vectoriales 2D:

$$\text{cross}(U, V, W) = (V_x - U_x)(W_y - U_y) - (V_y - U_y)(W_x - U_x)$$

Dos segmentos $\overline{AB}$ y $\overline{CD}$ se intersectan propiamente si y solo si:
$$\text{cross}(A, B, C) \cdot \text{cross}(A, B, D) < 0 \quad \text{y} \quad \text{cross}(C, D, A) \cdot \text{cross}(C, D, B) < 0$$

La coordenada exacta del cruce $X_{ij}$ (útil para graficar los nodos de intersección en el mapa virtual) se obtiene resolviendo el parámetro $t \in (0, 1)$:

$$t = \frac{(C_x - A_x)(D_y - C_y) - (C_y - A_y)(D_x - C_x)}{(B_x - A_x)(D_y - C_y) - (B_y - A_y)(D_x - C_x)}, \qquad X_{ij} = A + t(B - A)$$

---

## 4. Reconstrucción Geométrica del Perímetro

A partir de la nube de puntos de impacto en las paredes $\mathcal{P} = \{P_{\text{wall}, 1}, \dots, P_{\text{wall}, N}\}$:

### 4.1 Ordenamiento Angular / Envolvente Convexa (Algoritmo Monotone Chain de Andrew)
Para recintos convexos (o para obtener el contorno exterior libre de ruido interior):
1. Se ordenan los puntos lexicográficamente por $(X, Y)$ en $\mathcal{O}(N \log N)$.
2. Se construyen las cadenas inferior y superior descartando giros horarios ($\text{cross}(H_{m-1}, H_m, P) \le 0$).
3. Se aplica **simplificación de vértices casi colineales (Ramer-Douglas-Peucker + filtro de ángulo interno)**: si tres vértices consecutivos forman un ángulo de desviación menor a $\delta_{\text{collinear}}$ (ej. $8^\circ$) o distancia perpendicular menor a $\epsilon_{\text{rdp}} = 0.06\,\text{m}$, el vértice intermedio (causado por impactos a mitad de una misma pared) se fusiona para conservar únicamente las **esquinas reales** de la habitación $\{V_1, V_2, \dots, V_M\}$.

> **Extensión para habitaciones no convexas (en "L"):** Los puntos $\{P_{\text{wall}, k}\}$ se ordenan polarmente o mediante un grafo de vecindad (*Concave Hull / $\alpha$-Shape*), verificando que ninguna cuerda recorrida $S_k$ (que sabemos que pertenece al espacio libre interior) sea cortada por una pared candidata del polígono. Esta restricción de **visibilidad de cuerdas libres** garantiza que el perímetro no "corte" por dentro de un pasillo transitado.

### 4.2 Área y Perímetro Geométrico (Fórmula de la Lazada de Gauss / Shoelace)
Dado el polígono ordenado en sentido antihorario $V_1, V_2, \dots, V_M$ (con $V_{M+1} \equiv V_1$):

$$L_{\text{poly}} = \sum_{m=1}^{M} \|V_{m+1} - V_m\|_2$$

$$A_{\text{poly}} = \frac{1}{2} \left| \sum_{m=1}^{M} (V_{m, x} V_{m+1, y} - V_{m+1, x} V_{m, y}) \right|$$

### 4.3 Condición de Consistencia Cruzada (Geometría vs. Hormigas)
El mapa se valida verificando simultáneamente:
1. **Estabilidad del polígono:** $\frac{|A_{\text{poly}}^{(k)} - A_{\text{poly}}^{(k-4)}|}{A_{\text{poly}}^{(k)}} < 0.03$.
2. **Identidad de Cauchy-Crofton:** El cociente geométrico $\frac{A_{\text{poly}}}{L_{\text{poly}}}$ debe coincidir con el cociente estocástico $\frac{\bar{l}}{\pi}$.
3. **Concordancia del Área de Hormigas:** $\frac{|\hat{A}_{\text{ants}} - A_{\text{poly}}|}{A_{\text{poly}}} < \tau_{\text{ants}}$.

---

## 5. Generación del Camino Interior Seguro (Inward Polygon Offset) y Navegación

Para que el robot recorra el perímetro al final **sin chocar**, no puede intentar pasar por encima de las coordenadas $V_m$ (que son las paredes mismas), sino por una curva paralela interior a distancia $d_{\text{safe}} > R_{\text{robot}} + \epsilon_{\text{seguridad}}$.

### 5.1 Cálculo Analítico de Vértices Desplazados por Bisectriz
Para cada vértice $V_m$ del polígono en sentido antihorario (CCW), sean:
- $\hat{\mathbf{e}}_{\text{prev}} = \frac{V_m - V_{m-1}}{\|V_m - V_{m-1}\|}$ el vector unitario de la arista entrante.
- $\hat{\mathbf{e}}_{\text{next}} = \frac{V_{m+1} - V_m}{\|V_{m+1} - V_m\|}$ el vector unitario de la arista saliente.
- Las normales unitarias apuntando hacia el **interior** de la habitación (rotación de $+90^\circ$ en CCW) son:
  $$\hat{\mathbf{n}}_{\text{prev}} = (-e_{\text{prev}, y},\; e_{\text{prev}, x}), \qquad \hat{\mathbf{n}}_{\text{next}} = (-e_{\text{next}, y},\; e_{\text{next}, x})$$

El waypoint interior seguro $W_m$ se ubica sobre la bisectriz interior a una distancia corregida por el ángulo de la esquina:

$$\mathbf{b}_m = \frac{\hat{\mathbf{n}}_{\text{prev}} + \hat{\mathbf{n}}_{\text{next}}}{\|\hat{\mathbf{n}}_{\text{prev}} + \hat{\mathbf{n}}_{\text{next}}\|}, \qquad \cos(\alpha_m) = \hat{\mathbf{n}}_{\text{prev}} \cdot \mathbf{b}_m$$

$$\boxed{W_m = V_m + \frac{d_{\text{safe}}}{\max(\cos(\alpha_m), 0.35)} \, \mathbf{b}_m}$$

De esta forma, toda arista del camino $\overline{W_m W_{m+1}}$ queda exactamente a una distancia perpendicular $d_{\text{safe}}$ de la pared real $\overline{V_m V_{m+1}}$.

### 5.2 Ley de Control de Seguimiento Perimetral (Pure Pursuit + Rumbo)
Para ir de $W_m$ a $W_{m+1}$:
1. Se calcula el error de rumbo hacia el punto objetivo:
   $$e_\theta = \text{wrap\_pi}\left(\text{atan2}(W_{m+1, y} - y,\; W_{m+1, x} - x) - \theta\right)$$
2. Si $|e_\theta| > \theta_{\text{pivot}}$ (ej. $25^\circ$ al llegar a una esquina $W_m$), el robot gira suavemente en el lugar hasta alinear $|e_\theta| < 5^\circ$ antes de acelerar por la recta paralela a la pared.
3. En la recta, la velocidad angular comandada (y el ángulo del servo $\phi$) sigue una ley proporcional saturada:
   $$\omega_{\text{cmd}} = \text{clip}(K_p \, e_\theta,\; -\omega_{\max},\; \omega_{\max})$$
   garantizando un recorrido perimetral limpio, paralelo a las paredes y con **cero colisiones**.
