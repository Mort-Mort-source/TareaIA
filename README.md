# 🧠 Proyecto: Búsqueda Informada (A*)

Este repositorio contiene las implementaciones y reportes correspondientes a la **Tarea 2 – Búsqueda Informada (A\*)**, incluyendo:
- Simulación interactiva con distintos mapas de costo (Parte A).
- Aplicación del algoritmo A* al robot aspiradora siguiendo el modelo PEAS (Parte B).

---

## 📂 Estructura del proyecto

```
├── parteA_interactivo/
│   ├── astar_interactivo.py
│   ├── costmap_generator.py
│   ├── output/
│   │   ├── map.png
│   │   ├── time_map.png
│   │   ├── danger_map.png
│   │   ├── scenic_map.png
│   │   ├── route_*.png
│   └── informe_astar.tex
│
├── parteB_robot/
│   ├── robot_aspiradora_astar_big.py
│   ├── informe_astar_robot.tex
│   └── capturas/
│       ├── mapa_inicial.png
│       ├── ruta_rapida.png
│       └── ruta_eficiente.png
│
└── README.md
```

---

## 🧭 Parte A – Simulación Interactiva A*

**Archivo principal:** `astar_interactivo.py`

**Descripción:**
Simula el algoritmo A* sobre un mapa con distintos tipos de costo:
- Tiempo (azul)
- Peligro (rojo)
- Escénico (verde)

El usuario puede ajustar los pesos de cada costo en tiempo real y observar cómo cambia la ruta óptima.

**Controles:**
| Tecla | Función |
|-------|----------|
| T / G | Aumentar / disminuir peso del tiempo |
| Y / H | Aumentar / disminuir peso del peligro |
| U / J | Aumentar / disminuir peso del escénico |
| Click izq. / der. | Cambiar inicio / destino |
| R | Regenerar mapa |
| ESC | Salir |

**Ejecución:**
```bash
cd parteA_interactivo
python astar_interactivo.py
```

**Informe:** `informe_astar.tex`  
Contiene portada, explicación del algoritmo, interpretación de colores, controles y ejemplos de rutas.

---

## 🤖 Parte B – Robot Aspiradora (PEAS)

**Archivo principal:** `robot_aspiradora_astar_big.py`

**Descripción:**
Simula un robot aspiradora que limpia celdas sucias en un entorno 8×8 siguiendo el modelo PEAS:
- Obstáculos (muebles) → celdas negras  
- Suciedad → celdas verdes  
- Robot → celda amarilla (inicio)

El algoritmo A* se aplica para dos modos de limpieza:
1. **Modo rápido:** Minimiza tiempo (distancia total).
2. **Modo eficiente:** Minimiza energía (penaliza giros).

**Ejecución:**
```bash
cd parteB_robot
python robot_aspiradora_astar_big.py
```

**Informe:** `informe_astar_robot.tex`  
Incluye modelo PEAS, explicación de ambos modos, capturas de simulación y conclusiones.

---

## 🧩 Requisitos generales

Instalar dependencias:
```bash
pip install pygame numpy pillow
```

Recomendado para ejecutar en:
- Python 3.10 o superior  
- Sistema operativo Windows / Linux / macOS

---

## 🏁 Créditos

**Autor:** Aaron

**Materia:** Inteligencia Artificial  

**Fecha:** Octubre 2025



