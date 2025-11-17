#Algoritmo de dijkstra
import heapq

def dijkstra(grafo, inicio):
    dist = {n: float('inf') for n in grafo}
    dist[inicio] = 0
    visitados = set()
    pq = [(0, inicio)]

    while pq:
        distancia_actual, nodo = heapq.heappop(pq)

        if nodo in visitados:
            continue

        visitados.add(nodo)

        for vecino, peso in grafo[nodo]:
            nueva_dist = distancia_actual + peso

            if nueva_dist < dist[vecino]:
                dist[vecino] = nueva_dist
                heapq.heappush(pq, (nueva_dist, vecino))

    return dist
grafo = {
    'A': [('B', 4), ('C', 2)],
    'B': [('C', 5), ('D', 10)],
    'C': [('E', 3)],
    'D': [('F', 11)],
    'E': [('D', 4)],
    'F': []
}

resultado = dijkstra(grafo, 'A')
print("Distancias más cortas desde A:", resultado)




#Algoritmo de Floyd
def floyd_warshall(grafo):
    dist = [fila[:] for fila in grafo]  

    n = len(dist)

    for k in range(n):            
        for i in range(n):        
            for j in range(n):    
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]

    return dist


# Grafo de ejemplo (matriz de adyacencia)
# 0 significa un nodo hacia sí mismo
# inf significa que no hay conexión directa
inf = float('inf')

grafo = [
    [0,   3,   inf, 5],
    [2,   0,   inf, 4],
    [inf, 1,   0,   inf],
    [inf, inf, 2,   0]
]

resultado = floyd_warshall(grafo)

for fila in resultado:
    print(fila)


#Algoritmo de Warshall 
def warshall(grafo):
    n = len(grafo)
    alcance = [fila[:] for fila in grafo]  

    for k in range(n):            
        for i in range(n):        
            for j in range(n):    
                if alcance[i][j] == 1 or (alcance[i][k] == 1 and alcance[k][j] == 1):
                    alcance[i][j] = 1

    return alcance



grafo = [
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 0]
]

resultado = warshall(grafo)

for fila in resultado:
    print(fila)


#Algoritmo de Warshall 
def warshall(grafo):
    n = len(grafo)
    alcance = [fila[:] for fila in grafo]  

    for k in range(n):            
        for i in range(n):        
            for j in range(n):    
                if alcance[i][j] == 1 or (alcance[i][k] == 1 and alcance[k][j] == 1):
                    alcance[i][j] = 1

    return alcance



grafo = [
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 0]
]

resultado = warshall(grafo)

for fila in resultado:
    print(fila)



#Algoritmo de Kruskal 
class UnionFind:
    def __init__(self, size):
        self.padre = list(range(size))
        self.rango = [0] * size

    def find(self, x):
        if self.padre[x] != x:
            self.padre[x] = self.find(self.padre[x])
        return self.padre[x]

    def union(self, x, y):
        raizX = self.find(x)
        raizY = self.find(y)

        if raizX != raizY:
            if self.rango[raizX] < self.rango[raizY]:
                self.padre[raizX] = raizY
            elif self.rango[raizX] > self.rango[raizY]:
                self.padre[raizY] = raizX
            else:
                self.padre[raizY] = raizX
                self.rango[raizX] += 1
            return True
        return False


def kruskal(vertices, aristas):
    uf = UnionFind(vertices)
    aristas.sort(key=lambda x: x[2])  
    mst = []

    for u, v, peso in aristas:
        if uf.union(u, v):
            mst.append((u, v, peso))

    return mst


# Ejemplo:
aristas = [
    (0, 1, 4),
    (0, 2, 3),
    (1, 2, 1),
    (1, 3, 2),
    (2, 3, 4),
    (3, 4, 2),
    (4, 5, 6)
]

resultado = kruskal(6, aristas)
print("Árbol de Expansión Mínima:", resultado)
