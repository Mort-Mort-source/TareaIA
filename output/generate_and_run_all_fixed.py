"""
generate_and_run_all_fixed.py

Genera un mapa de prueba y 3 cost maps (time, danger, scenic), corre A* con combinación
de costos y guarda los archivos en la carpeta output/. Versión corregida que garantiza
conectividad en los mapas.

Archivos generados (en output/):
 - map.png
 - time_map.png
 - danger_map.png
 - scenic_map.png
 - route_with_costmaps.png

Dependencias:
pip install pillow numpy

Ejecutar:
python generate_and_run_all_fixed.py
"""
import os
import random
import math
import heapq
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# ---------------- Configuración ----------------
OUT_DIR = Path("output")
OUT_DIR.mkdir(exist_ok=True)
W, H = 300, 200              # tamaño del mapa (ancho, alto) en "píxeles"
MAP_TYPE = "rects"           # opciones: "rects", "noise", "maze"
RANDOM_SEED = 42
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

# ---------------- Generadores de mapa MEJORADOS ----------------
def generate_empty(w, h, fill=1):
    return np.ones((h, w), dtype=np.uint8) * fill

def add_random_rectangles_fixed(arr, n=15, min_size=4, max_size=25):
    """Versión mejorada que evita crear áreas inaccesibles"""
    h, w = arr.shape
    
    # Primero asegurar pasillos principales
    corridor_width = 15
    arr[h//2-corridor_width//2:h//2+corridor_width//2, :] = 1  # Pasillo horizontal
    arr[:, w//2-corridor_width//2:w//2+corridor_width//2] = 1  # Pasillo vertical
    
    for _ in range(n):
        rw = random.randint(min_size, max_size)
        rh = random.randint(min_size, max_size)
        x = random.randint(0, max(0, w-rw-1))
        y = random.randint(0, max(0, h-rh-1))
        
        # Verificar que no bloquee completamente los pasillos principales
        blocks_horizontal = (y < h//2 < y+rh) and (x < w//4 or x+rw > 3*w//4)
        blocks_vertical = (x < w//2 < x+rw) and (y < h//4 or y+rh > 3*h//4)
        
        if not (blocks_horizontal or blocks_vertical):
            arr[y:y+rh, x:x+rw] = 0
    
    return arr

def generate_noise_map_fixed(w, h, density=0.25):
    """Ruido más disperso para mantener conectividad"""
    noise = (np.random.rand(h, w) > density).astype(np.uint8)
    
    # Asegurar conectividad básica - crear pasillos principales
    noise[h//3:2*h//3, :] = 1  # Pasillo horizontal
    noise[:, w//3:2*w//3] = 1  # Pasillo vertical
    
    return noise

def generate_maze_fixed(w, h):
    """Laberinto que garantiza conectividad"""
    # Asegurar dimensiones impares
    Wm = w if w % 2 == 1 else w - 1
    Hm = h if h % 2 == 1 else h - 1
    
    maze = np.zeros((Hm, Wm), dtype=np.uint8)
    
    def carve(x, y):
        maze[y, x] = 1
        dirs = [(2,0), (-2,0), (0,2), (0,-2)]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 < nx < Wm and 0 < ny < Hm and maze[ny, nx] == 0:
                maze[y + dy//2, x + dx//2] = 1
                carve(nx, ny)
    
    # Comenzar desde múltiples puntos para mejor conectividad
    carve(1, 1)
    
    # Crear matriz del tamaño original
    final_maze = np.ones((h, w), dtype=np.uint8)
    final_maze[:Hm, :Wm] = maze
    
    return final_maze

def verify_connectivity(occupancy, start, goal):
    """Verifica que start y goal estén conectados usando BFS"""
    h, w = occupancy.shape
    visited = np.zeros((h, w), dtype=bool)
    
    sx, sy = start
    if not occupancy[sy, sx]:
        return False
    
    queue = deque([start])
    visited[sy, sx] = True
    
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
            
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx] and occupancy[ny, nx]:
                visited[ny, nx] = True
                queue.append((nx, ny))
    
    return False

def find_free_near(occupancy, cand):
    """Versión mejorada que busca celdas libres"""
    x0, y0 = cand
    h, w = occupancy.shape
    
    # Buscar en espiral desde el punto candidato
    for radius in range(0, max(w, h)):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) == radius or abs(dy) == radius:  # Solo el perímetro
                    x, y = x0 + dx, y0 + dy
                    if 0 <= x < w and 0 <= y < h and occupancy[y, x]:
                        return (x, y)
    return None

# ---------------- Helpers para normalizar y guardar costmaps ----------------
def normalize_map(arr):
    a = np.array(arr, dtype=float)
    mn, mx = a.min(), a.max()
    if mx - mn < 1e-9:
        return np.zeros_like(a, dtype=float)
    return (a - mn) / (mx - mn)

def save_binary_map(arr, path):
    # arr: 2D array 1=free,0=obstacle
    h, w = arr.shape
    im = Image.new("L", (w, h))
    pixels = (arr * 255).astype(np.uint8)
    im.putdata(pixels.flatten())
    im = im.convert("RGB")
    im.save(path)
    print(f"Saved map: {path}")

def save_cost_map_as_image(cost_map, path):
    # cost_map assumed in [0,1] - map 0->black, 1->white for visualization
    a = (np.clip(cost_map, 0.0, 1.0) * 255).astype(np.uint8)
    im = Image.new("L", (a.shape[1], a.shape[0]))
    im.putdata(a.flatten())
    im = im.convert("RGB")
    im.save(path)
    print(f"Saved cost map image: {path}")

# ---------------- A* con cost maps ----------------
def euclidean(a,b):
    (x1,y1),(x2,y2)=a,b
    return math.hypot(x1-x2, y1-y2)

def neighbors_8(x, y, H, W):
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= ny < H:
                yield nx, ny

def a_star_with_costmaps(start, goal, occupancy,
                         neighbors_fn,
                         base_heuristic_fn,
                         cost_maps=None,   # dict name -> 2D arrays (float)
                         weights=None,     # dict name -> weight
                         base_cost_weight=1.0):
    H, W = occupancy.shape
    sx, sy = start; gx, gy = goal
    if not occupancy[sy, sx] or not occupancy[gy, gx]:
        raise ValueError("start o goal sobre obstáculo")

    if cost_maps is None: cost_maps = {}
    if weights is None: weights = {}

    # Calcular multiplicador mínimo optimista para heurística admisible
    min_sum = base_cost_weight
    for k, cmap in cost_maps.items():
        w = float(weights.get(k, 0.0))
        if w == 0.0: continue
        min_val = float(np.min(cmap))
        min_sum += w * min_val
    min_multiplier = min_sum

    def cost_fn(a,b):
        (x1,y1),(x2,y2) = a,b
        base = math.hypot(x1-x2, y1-y2)
        factor = base_cost_weight
        for k, cmap in cost_maps.items():
            w = float(weights.get(k, 0.0))
            if w == 0.0: continue
            factor += w * float(cmap[y2, x2])   # usamos celda destino
        return base * factor

    def heuristic_fn(a,b):
        return base_heuristic_fn(a,b) * min_multiplier

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    f_score = {start: heuristic_fn(start, goal)}
    visited = set()

    while open_heap:
        current_f, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            # reconstruir ruta
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, g_score[goal]
        cx, cy = current
        for nx, ny in neighbors_fn(cx, cy, H, W):
            if not occupancy[ny, nx]:
                continue
            neighbor = (nx, ny)
            tentative_g = g_score[current] + cost_fn(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic_fn(neighbor, goal)
                f_score[neighbor] = f
                heapq.heappush(open_heap, (f, neighbor))
    return None, math.inf

# ---------------- Dibujo de la ruta ----------------
def draw_route_on_map(occupancy, path, out_path):
    h, w = occupancy.shape
    im = Image.new("RGB", (w, h))
    pix = im.load()
    for y in range(h):
        for x in range(w):
            pix[x,y] = (255,255,255) if occupancy[y,x] else (0,0,0)
    draw = ImageDraw.Draw(im)
    if path:
        draw.line(path, fill=(255,0,0), width=2)
        for (x,y) in path:
            r = 2
            draw.ellipse((x-r,y-r,x+r,y+r), fill=(0,255,0))
    im.save(out_path)
    print(f"Saved route image: {out_path}")

# ---------------- Pipeline principal MEJORADO ----------------
def main():
    # 1) Generar mapa base MEJORADO
    print(f"Generating {MAP_TYPE} map...")
    if MAP_TYPE == "rects":
        occ = generate_empty(W, H, fill=1)
        occ = add_random_rectangles_fixed(occ, n=20, min_size=4, max_size=30)
    elif MAP_TYPE == "noise":
        occ = generate_noise_map_fixed(W, H, density=0.25)
    elif MAP_TYPE == "maze":
        occ = generate_maze_fixed(W, H)
    else:
        raise ValueError("MAP_TYPE inválido")

    # 2) Verificar y ajustar start/goal hasta que estén conectados
    print("Finding start and goal positions...")
    max_attempts = 10
    start = None
    goal = None
    
    for attempt in range(max_attempts):
        # Elegir start y goal en esquinas opuestas
        start_candidates = [(5, 5), (5, H-6), (W-6, 5), (W-6, H-6)]
        goal_candidates = [(W-6, H-6), (W-6, 5), (5, H-6), (5, 5)]
        
        start = find_free_near(occ, start_candidates[attempt % 4])
        goal = find_free_near(occ, goal_candidates[(attempt + 2) % 4])  # Esquina opuesta
        
        if start is None or goal is None:
            print(f"Attempt {attempt}: Could not find free start/goal")
            continue
            
        if verify_connectivity(occ, start, goal):
            print(f"Found connected start and goal after {attempt + 1} attempts")
            break
        else:
            print(f"Attempt {attempt}: Start and goal not connected")
    else:
        # Si no se encontró conexión después de varios intentos, forzar un camino
        print("Warning: Forcing connectivity in the map")
        if start and goal:
            # Algoritmo simple para conectar start y goal
            x1, y1 = start
            x2, y2 = goal
            # Camino en L
            for x in range(min(x1, x2), max(x1, x2) + 1):
                if 0 <= x < W:
                    occ[y1, x] = 1
            for y in range(min(y1, y2), max(y1, y2) + 1):
                if 0 <= y < H:
                    occ[y, x2] = 1

    if start is None or goal is None:
        print("ERROR: Could not find valid start and goal positions")
        return

    print("start =", start, "goal =", goal)
    
    if not verify_connectivity(occ, start, goal):
        print("ERROR: No se pudo garantizar conectividad entre start y goal")
        return

    save_binary_map(occ, OUT_DIR / "map.png")

    # 3) Crear cost maps de ejemplo (time, danger, scenic)
    print("Generating cost maps...")
    # time_map: zonas lentas (valores > 1), luego normalizamos
    time_map = np.ones((H, W), dtype=float)
    # pintar una franja lenta
    time_map[40:90, 30:120] = 2.0
    time_map[120:170, 160:200] = 1.8
    time_map = normalize_map(time_map)
    save_cost_map_as_image(time_map, OUT_DIR / "time_map.png")

    # danger_map: ruido + algunos hotspots
    danger_map = np.random.rand(H, W) * 0.6  # base de riesgo medio
    # hotspots peligrosos
    danger_map[20:40, 200:230] += 0.6
    danger_map[100:130, 50:80] += 0.7
    danger_map = normalize_map(danger_map)
    save_cost_map_as_image(danger_map, OUT_DIR / "danger_map.png")

    # scenic_map: zonas escénicas (1 = muy bonito), pero para coste queremos penalidad = 1 - scenic
    scenic = np.zeros((H, W), dtype=float)
    scenic[150:190, 220:280] = 1.0  # Corregido para estar dentro de los límites
    scenic[60:90, 180:230] = 0.8
    scenic = normalize_map(scenic)
    scenic_penalty = 1.0 - scenic
    scenic_penalty = normalize_map(scenic_penalty)
    save_cost_map_as_image(scenic_penalty, OUT_DIR / "scenic_map.png")

    # 4) Definir pesos (ajustables)
    weights = {
        'time': 1.4,     # importancia de minimizar tiempo
        'danger': 1.0,   # importancia de evitar peligro
        'scenic': 0.6    # importancia de preferir zonas con scenic (hemos convertido scenic->penalty)
    }
    cost_maps = {'time': time_map, 'danger': danger_map, 'scenic': scenic_penalty}
    base_cost_weight = 1.0  # peso para distancia pura

    # 5) Ejecutar A*
    print("Running A* algorithm...")
    path, cost = a_star_with_costmaps(start, goal, occ,
                                      neighbors_fn=neighbors_8,
                                      base_heuristic_fn=euclidean,
                                      cost_maps=cost_maps,
                                      weights=weights,
                                      base_cost_weight=base_cost_weight)
    if path:
        print(f"Ruta encontrada: {len(path)} nodos, costo total ≈ {cost:.3f}")
    else:
        print("No se encontró ruta")

    # 6) Guardar ruta dibujada
    draw_route_on_map(occ, path, OUT_DIR / "route_with_costmaps.png")
    print("Done! Check the 'output' folder for generated files.")

if __name__ == "__main__":
    main()