"""
astar_interactivo.py

Versión interactiva de A* con pygame.
Permite mover start/goal y cambiar pesos (tiempo, peligro, escénico) en tiempo real.
"""

import pygame
import numpy as np
import math, heapq, random

# ---------------- Parámetros globales ----------------
W, H = 100, 70      # tamaño del mapa en celdas
CELL = 8            # tamaño en pixeles de cada celda
FPS = 30
RANDOM_SEED = 42
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

# ---------------- Generadores de mapas ----------------
def generate_map():
    """Genera mapa binario (1 libre, 0 obstáculo)"""
    occ = np.ones((H, W), dtype=np.uint8)
    for _ in range(35):
        x = random.randint(0, W-10)
        y = random.randint(0, H-10)
        w = random.randint(3, 10)
        h = random.randint(3, 8)
        occ[y:y+h, x:x+w] = 0
    return occ

def normalize_map(arr):
    a = np.array(arr, dtype=float)
    mn, mx = a.min(), a.max()
    if mx - mn < 1e-9:
        return np.zeros_like(a)
    return (a - mn) / (mx - mn)

def generate_cost_maps():
    """Genera mapas de tiempo, peligro, escénico normalizados [0,1]"""
    time_map = np.ones((H, W))
    time_map[20:40, 10:40] = 2.0
    time_map[45:55, 60:80] = 1.8
    time_map = normalize_map(time_map)

    danger_map = np.random.rand(H, W) * 0.6
    danger_map[5:15, 60:80] += 0.6
    danger_map[40:55, 20:30] += 0.7
    danger_map = normalize_map(danger_map)

    scenic = np.zeros((H, W))
    scenic[50:70, 70:95] = 1.0
    scenic[20:30, 5:25] = 0.6
    scenic = normalize_map(scenic)
    scenic_penalty = 1.0 - scenic

    return {'time': time_map, 'danger': danger_map, 'scenic': scenic_penalty}

# ---------------- A* con cost maps ----------------
def euclid(a,b):
    (x1,y1),(x2,y2)=a,b
    return math.hypot(x1-x2, y1-y2)

def neighbors_8(x,y,H_,W_):
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if dx==0 and dy==0: continue
            nx, ny = x+dx, y+dy
            if 0<=nx<W_ and 0<=ny<H_:
                yield nx, ny

def a_star_with_costmaps(start, goal, occupancy, cost_maps, weights, base_weight=1.0):
    H_, W_ = occupancy.shape
    if not occupancy[start[1], start[0]] or not occupancy[goal[1], goal[0]]:
        return None
    def cost_fn(a,b):
        (x1,y1),(x2,y2)=a,b
        base = math.hypot(x1-x2, y1-y2)
        f = base_weight
        for k in cost_maps:
            f += weights[k]*cost_maps[k][b[1], b[0]]
        return base*f
    def heuristic(a,b):
        return euclid(a,b)*base_weight
    open_heap=[]
    g={start:0.0}
    came={}
    heapq.heappush(open_heap,(heuristic(start,goal),start))
    visited=set()
    while open_heap:
        f,cur=heapq.heappop(open_heap)
        if cur in visited: continue
        visited.add(cur)
        if cur==goal:
            path=[cur]
            while cur in came:
                cur=came[cur]
                path.append(cur)
            path.reverse()
            return path
        for nx,ny in neighbors_8(cur[0],cur[1],H_,W_):
            if not occupancy[ny,nx]: continue
            n=(nx,ny)
            tg=g[cur]+cost_fn(cur,n)
            if n not in g or tg<g[n]:
                g[n]=tg
                came[n]=cur
                heapq.heappush(open_heap,(tg+heuristic(n,goal),n))
    return None

# ---------------- Visualización Pygame ----------------
def draw_grid(screen, occ, cost_maps, weights, path, start, goal):
    for y in range(H):
        for x in range(W):
            if not occ[y,x]:
                color=(20,20,20)
            else:
                # mezcla de colores según mapas y pesos
                r = int(cost_maps['danger'][y,x]*255*weights['danger'])
                g = int(cost_maps['scenic'][y,x]*255*weights['scenic'])
                b = int(cost_maps['time'][y,x]*255*weights['time'])
                color=(min(r,255),min(g,255),min(b,255))
            pygame.draw.rect(screen, color, (x*CELL, y*CELL, CELL, CELL))
    # ruta
    if path:
        for (x,y) in path:
            pygame.draw.rect(screen, (255,0,0), (x*CELL, y*CELL, CELL, CELL))
    # start y goal
    pygame.draw.rect(screen, (0,255,0), (start[0]*CELL, start[1]*CELL, CELL, CELL))
    pygame.draw.rect(screen, (255,255,0), (goal[0]*CELL, goal[1]*CELL, CELL, CELL))

    # texto info
    font = pygame.font.SysFont(None, 20)
    txt = f"Tiempo:{weights['time']:.1f}  Peligro:{weights['danger']:.1f}  Escenico:{weights['scenic']:.1f}"
    img = font.render(txt, True, (255,255,255))
    screen.blit(img, (5, H*CELL + 5))

# ---------------- Main loop ----------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((W*CELL, H*CELL+30))
    clock = pygame.time.Clock()
    occ = generate_map()
    cost_maps = generate_cost_maps()
    weights = {'time':1.0, 'danger':1.0, 'scenic':1.0}
    start, goal = (2,2), (W-3,H-3)
    path = a_star_with_costmaps(start, goal, occ, cost_maps, weights)
    running=True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running=False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running=False
                elif ev.key == pygame.K_r:
                    occ = generate_map()
                    cost_maps = generate_cost_maps()
                elif ev.key == pygame.K_t:
                    weights['time'] += 0.2
                elif ev.key == pygame.K_g:
                    weights['time'] = max(0, weights['time'] - 0.2)
                elif ev.key == pygame.K_y:
                    weights['danger'] += 0.2
                elif ev.key == pygame.K_h:
                    weights['danger'] = max(0, weights['danger'] - 0.2)
                elif ev.key == pygame.K_u:
                    weights['scenic'] += 0.2
                elif ev.key == pygame.K_j:
                    weights['scenic'] = max(0, weights['scenic'] - 0.2)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                mx,my = pygame.mouse.get_pos()
                gx,gy = mx//CELL, my//CELL
                if ev.button==1:
                    start=(gx,gy)
                elif ev.button==3:
                    goal=(gx,gy)
        # recalcular ruta cada frame
        path = a_star_with_costmaps(start, goal, occ, cost_maps, weights)
        screen.fill((0,0,0))
        draw_grid(screen, occ, cost_maps, weights, path, start, goal)
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()

if __name__ == "__main__":
    main()
