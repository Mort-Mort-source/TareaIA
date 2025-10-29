"""
robot_aspiradora_astar_big.py
Versión ampliada del punto B (A* aplicado a PEAS - Robot Aspiradora)
Autor: Aaron [tu apellido]
Fecha: 29 Octubre 2025

Mapa 8x8 con obstáculos (muebles) y zonas sucias (celdas verdes).
El robot limpia todas las zonas sucias en:
 1) Modo rápido (tiempo mínimo)
 2) Modo eficiente (energía mínima, penaliza giros)
"""

import pygame, random, math, heapq

# ---------------- CONFIGURACIÓN ----------------
GRID_SIZE = 8
CELL_SIZE = 80
MUEBLES = 10
SUCIAS = 6
FPS = 2

# Colores
WHITE = (255,255,255)
BLACK = (30,30,30)
GREEN = (0,180,0)
GRAY = (180,180,180)
BLUE = (80,80,255)
RED = (255,50,50)
YELLOW = (255,255,0)

# ---------------- FUNCIONES AUXILIARES ----------------
def manhattan(a,b):
    (x1,y1),(x2,y2)=a,b
    return abs(x1-x2)+abs(y1-y2)

def neighbors_4(x,y,H,W):
    for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
        nx,ny=x+dx,y+dy
        if 0<=nx<W and 0<=ny<H:
            yield nx,ny

def a_star(start, goal, grid, cost_fn=None):
    H=W=len(grid)
    if cost_fn is None:
        cost_fn=lambda a,b:1
    openh=[]
    heapq.heappush(openh,(0,start))
    g={start:0}
    came={}
    while openh:
        f,cur=heapq.heappop(openh)
        if cur==goal:
            path=[cur]
            while cur in came:
                cur=came[cur]
                path.append(cur)
            return list(reversed(path)), g[goal]
        for nx,ny in neighbors_4(cur[0],cur[1],H,W):
            if grid[ny][nx]==1: # obstáculo
                continue
            cost=g[cur]+cost_fn(cur,(nx,ny))
            if (nx,ny) not in g or cost<g[(nx,ny)]:
                g[(nx,ny)]=cost
                f=cost+manhattan((nx,ny),goal)
                heapq.heappush(openh,(f,(nx,ny)))
                came[(nx,ny)]=cur
    return None,math.inf

# ---------------- MAPA ----------------
def generar_mapa():
    grid=[[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    libres=[(x,y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]
    random.shuffle(libres)
    # muebles
    for _ in range(MUEBLES):
        x,y=libres.pop()
        grid[y][x]=1
    # sucias
    for _ in range(SUCIAS):
        while libres:
            x,y=libres.pop()
            if grid[y][x]==0:
                grid[y][x]=2
                break
    # inicio
    libres2=[(x,y) for x in range(GRID_SIZE) for y in range(GRID_SIZE) if grid[y][x]==0]
    start=random.choice(libres2)
    return grid,start

# ---------------- PLANIFICADOR ----------------
def ruta_para_limpieza(grid, start, modo="rapido"):
    sucias=[(x,y) for y in range(GRID_SIZE) for x in range(GRID_SIZE) if grid[y][x]==2]
    ruta_total=[]
    total_cost=0
    actual=start
    dir_anterior=None

    while sucias:
        mejor_ruta=None
        mejor_cost=math.inf
        mejor_dest=None
        for destino in sucias:
            def cost_fn(a,b):
                base=1
                if modo=="eficiente" and dir_anterior is not None:
                    dx=b[0]-a[0]; dy=b[1]-a[1]
                    nueva_dir=(dx,dy)
                    if nueva_dir!=dir_anterior:
                        return base+1.5  # penaliza giro
                return base
            path,cost=a_star(actual,destino,grid,cost_fn)
            if path and cost<mejor_cost:
                mejor_cost=cost
                mejor_ruta=path
                mejor_dest=destino
        if not mejor_ruta:
            break
        ruta_total+=mejor_ruta[1:]
        total_cost+=mejor_cost
        actual=mejor_dest
        grid[actual[1]][actual[0]]=0
        sucias.remove(actual)
        if len(mejor_ruta)>=2:
            dx=mejor_ruta[-1][0]-mejor_ruta[-2][0]
            dy=mejor_ruta[-1][1]-mejor_ruta[-2][1]
            dir_anterior=(dx,dy)
    return ruta_total,total_cost

# ---------------- VISUALIZACIÓN ----------------
def dibujar_grid(screen, grid, start, ruta, step):
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            rect=pygame.Rect(x*CELL_SIZE,y*CELL_SIZE,CELL_SIZE,CELL_SIZE)
            val=grid[y][x]
            if val==1:
                color=BLACK
            elif val==2:
                color=GREEN
            else:
                color=GRAY
            pygame.draw.rect(screen,color,rect)
            pygame.draw.rect(screen,WHITE,rect,1)
    # ruta recorrida
    for (x,y) in ruta[:step]:
        rect=pygame.Rect(x*CELL_SIZE+CELL_SIZE//4,y*CELL_SIZE+CELL_SIZE//4,CELL_SIZE//2,CELL_SIZE//2)
        pygame.draw.rect(screen,BLUE,rect)
    # posición actual
    if step<len(ruta):
        x,y=ruta[step]
        rect=pygame.Rect(x*CELL_SIZE+CELL_SIZE//3,y*CELL_SIZE+CELL_SIZE//3,CELL_SIZE//3,CELL_SIZE//3)
        pygame.draw.rect(screen,RED,rect)
    # inicio
    sx,sy=start
    pygame.draw.rect(screen,YELLOW,(sx*CELL_SIZE+CELL_SIZE//3,sy*CELL_SIZE+CELL_SIZE//3,CELL_SIZE//3,CELL_SIZE//3))

def ejecutar_modo(nombre, grid_inicial, start):
    pygame.init()
    screen=pygame.display.set_mode((GRID_SIZE*CELL_SIZE,GRID_SIZE*CELL_SIZE))
    pygame.display.set_caption(f"Modo {nombre}")
    clock=pygame.time.Clock()

    grid=[row[:] for row in grid_inicial]
    ruta,costo=ruta_para_limpieza(grid,start,modo=nombre)
    print(f"Modo {nombre}: costo total={costo:.2f}, pasos={len(ruta)}")

    running=True
    step=0
    while running:
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT:
                running=False
        screen.fill(WHITE)
        dibujar_grid(screen,grid_inicial,start,ruta,step)
        pygame.display.flip()
        step=min(step+1,len(ruta))
        clock.tick(FPS)
        if step>=len(ruta):
            pygame.time.wait(1000)
            running=False
    pygame.quit()

# ---------------- MAIN ----------------
if __name__=="__main__":
    grid,start=generar_mapa()
    print("Mapa generado (0=libre,1=mueble,2=sucia):")
    for fila in grid:
        print(fila)
    print("Inicio:",start)
    ejecutar_modo("rapido",grid,start)
    ejecutar_modo("eficiente",grid,start)
