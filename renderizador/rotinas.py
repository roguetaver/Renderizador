#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
Rotinas de operação de nós X3D.
Desenvolvido por:
Disciplina: Computação Gráfica
Data:
"""

import gpu          # Simula os recursos de uma GPU

#################################################################################
# NÃO USAR MAIS ESSE ARQUIVO. AS ROTINAS DEVEM SER IMPLEMENTADAS AGORA NO gl.GL #
#################################################################################

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polypoint2D

def polypoint2D(point, colors):
    """Função usada para renderizar Polypoint2D."""
    # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
    # de pontos x, y sempre na ordem. Assim point[0] é o valor da coordenada x do
    # primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é a
    # coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista e assuma que sempre vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polypoint2D
    # você pode assumir o desenho dos pontos com a cor emissiva (emissiveColor).

    # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
    print("Polypoint2D : pontos = {0}".format(point)) # imprime no terminal pontos
    print("Polypoint2D : colors = {0}".format(colors)) # imprime no terminal as cores
    # Exemplo:

    nPontos = int(len(point)/2)
    
    intpointsX = []
    intpointsY = []
    for i in range(0,len(point) ,2):
        intpointsX.append(int(point[i]))
        intpointsY.append(int(point[i + 1]))


    for p in range(nPontos):
        gpu.GPU.set_pixel(intpointsX[p], intpointsY[p], int(colors["emissiveColor"][0] * 255), int(colors["emissiveColor"][1] * 255), int(colors["emissiveColor"][2] * 255)) # altera um pixel da imagem (u, v, r, g, b)

    
    # cuidado com as cores, o X3D especifica de (0,1) e o Framebuffer de (0,255)

#EXTRAIDO DE -> https://newbedev.com/python-bresenham-s-line-drawing-algorithm-python-code-example
def get_line(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polyline2D
def polyline2D(lineSegments, colors):
    """Função usada para renderizar Polyline2D."""
    # Nessa função você receberá os pontos de uma linha no parâmetro lineSegments, esses
    # pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o valor da
    # coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é
    # a coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista. A quantidade mínima de pontos são 2 (4 valores), porém a
    # função pode receber mais pontos para desenhar vários segmentos. Assuma que sempre
    # vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polyline2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).

    print("Polyline2D : lineSegments = {0}".format(lineSegments)) # imprime no terminal
    print("Polyline2D : colors = {0}".format(colors)) # imprime no terminal as cores
    # Exemplo:
    #pos_x = gpu.GPU.width//2
    #pos_y = gpu.GPU.height//2
    #gpu.GPU.set_pixel(pos_x, pos_y, 255, 0, 0) # altera um pixel da imagem (u, v, r, g, b)

    nPontos = int(len(lineSegments)/2)
    
    intpointsX = []
    intpointsY = []
    points = []
    for i in range(0,len(lineSegments) ,2):
        intpointsX.append(int(lineSegments[i]))
        intpointsY.append(int(lineSegments[i + 1]))

    for p in range(0,nPontos,2):
        points = get_line((intpointsX[p],intpointsY[p]),(intpointsX[p+1],intpointsY[p+1]))
        for x in range(len(points)):
            gpu.GPU.set_pixel(points[x][0], points[x][1], int(colors["emissiveColor"][0] * 255), int(colors["emissiveColor"][1] * 255), int(colors["emissiveColor"][2] * 255)) # altera um pixel da imagem (u, v, r, g, b)
        


def inside(x1,y1,x2,y2,x3,y3,x,y):
    L0 = False
    L1 = False
    L2 = False
    x = x + 0.5
    y = y + 0.5

    if  (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1) >= 0:
        L0 = True
    
    if (x - x2) * (y3 - y2) - (y - y2) * (x3 - x2) >= 0:
        L1 = True
    
    if  (x - x3) * (y1 - y3) - (y - y3) * (x1 - x3) >= 0:
        L2 = True

    if(L1 and L2 and L0):
        return True
    else:
        return False

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#TriangleSet2D
def triangleSet2D(vertices, colors):
    """Função usada para renderizar TriangleSet2D."""
    # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
    # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
    # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
    # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
    # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
    print("TriangleSet2D : vertices = {0}".format(vertices)) # imprime no terminal
    print("TriangleSet2D : colors = {0}".format(colors)) # imprime no terminal as cores

    for e in range(0,len(vertices),6):
        x1 = vertices[e]
        y1 = vertices[e+1]
        x2 = vertices[e+2]
        y2 = vertices[e+3]
        x3 = vertices[e+4]
        y3 = vertices[e+5]

        for x in range(gpu.GPU.width):
            for y in range(gpu.GPU.height):
                if(inside(x1,y1,x2,y2,x3,y3,x,y)):
                    gpu.GPU.set_pixel(x, y, int(colors["emissiveColor"][0] * 255), int(colors["emissiveColor"][1] * 255), int(colors["emissiveColor"][2] * 255))


