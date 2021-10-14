#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
Biblioteca Gráfica / Graphics Library.

Desenvolvido por: André Tavernaro e André Rocco
Disciplina: Computação Gráfica
Data: 14/10/2021
"""

import gpu          # Simula os recursos de uma GPU
import numpy as np
import math


class GL:
    """Classe que representa a biblioteca gráfica (Graphics Library)."""

    width = 800   # largura da tela
    height = 600  # altura da tela
    near = 0.01   # plano de corte próximo
    far = 1000    # plano de corte distante

    @staticmethod
    def setup(width, height, near=0.01, far=1000):
        """Definr parametros para câmera de razão de aspecto, plano próximo e distante."""
        GL.width = width
        GL.height = height
        GL.near = near
        GL.far = far
        GL.stack = []

    @staticmethod
    def triangleSet2D(vertices, ponto, colors):
        x, y = ponto[0], ponto[1]

        x1, x2, x3 = vertices[0][0][0], vertices[1][0][0], vertices[2][0][0]
        y1, y2, y3 = vertices[0][1][0], vertices[1][1][0], vertices[2][1][0]

        L0 = False
        L1 = False
        L2 = False
        x = x + 0.5
        y = y + 0.5

        if (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1) >= 0:
            L0 = True

        if (x - x2) * (y3 - y2) - (y - y2) * (x3 - x2) >= 0:
            L1 = True

        if (x - x3) * (y1 - y3) - (y - y3) * (x1 - x3) >= 0:
            L2 = True

        if(L1 and L2 and L0):
            gpu.GPU.set_pixel(int(x), int(y), int(colors["diffuseColor"][0]*255), int(
                colors["diffuseColor"][1]*255), int(colors["diffuseColor"][2]*255))

    @staticmethod
    def triangleSet(point, colors):
        """Função usada para renderizar TriangleSet."""
        # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
        # de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x do
        # primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e
        # assim por diante.
        # No TriangleSet os triângulos são informados individualmente, assim os três
        # primeiros pontos definem um triângulo, os três próximos pontos definem um novo
        # triângulo, e assim por diante.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).

        GL.matriz_M = np.matmul(
            np.matmul(GL.matriz_trans_in, GL.matriz_rotacao), GL.matriz_escala_in)

        GL.matriz_tela = np.array([[GL.width/2, 0, 0, GL.width/2],
                                   [0, -GL.height/2, 0, GL.height/2],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        coords = []

        GL.matriz_look_at = np.matmul(np.linalg.inv(
            GL.matriz_orien_cam), np.linalg.inv(GL.matriz_trans_cam))

        for i in range(0, len(point), 3):
            ponto = np.array([[point[i]],
                              [point[i+1]],
                              [point[i+2]],
                              [1]])

            ponto = np.matmul(GL.matriz_M, ponto)
            ponto = np.matmul(GL.matriz_look_at, ponto)
            ponto = np.matmul(GL.matriz_perspectiva, ponto)
            ponto /= ponto[3][0]
            ponto = np.matmul(GL.matriz_tela, ponto)
            coords.append(ponto)

        for triangulo in range(0, len(coords), 3):
            for i in range(GL.width):
                for j in range(GL.height):
                    GL.triangleSet2D(
                        coords[triangulo:triangulo+3], [i, j], colors)

    @staticmethod
    def viewpoint(position, orientation, fieldOfView):
        """Função usada para renderizar (na verdade coletar os dados) de Viewpoint."""
        # Na função de viewpoint você receberá a posição, orientação e campo de visão da
        # câmera virtual. Use esses dados para poder calcular e criar a matriz de projeção
        # perspectiva para poder aplicar nos pontos dos objetos geométricos.

        fovy = 2*math.atan(math.tan(fieldOfView/2)*GL.height /
                           math.sqrt(GL.height**2 + GL.width**2))
        cima = GL.near*math.tan(fovy)
        direita = cima*(GL.width/GL.height)

        if orientation:
            if (orientation[0] > 0):
                GL.matriz_orien_cam = np.array([[1, 0, 0, 0],
                                                [0, math.cos(
                                                    orientation[3]), -math.sin(orientation[3]), 0],
                                                [0, math.sin(orientation[3]), math.cos(
                                                    orientation[3]), 0],
                                                [0, 0, 0, 1]])
            elif (orientation[1] > 0):
                GL.matriz_orien_cam = np.array([[math.cos(orientation[3]), 0, math.sin(orientation[3]), 0],
                                                [0, 1, 0, 0],
                                                [-math.sin(orientation[3]), 0,
                                                 math.cos(orientation[3]), 0],
                                                [0, 0, 0, 1]])
            else:
                GL.matriz_orien_cam = np.array([[math.cos(orientation[3]), -math.sin(orientation[3]), 0, 0],
                                                [math.sin(orientation[3]), math.cos(
                                                    orientation[3]), 0, 0],
                                                [0, 0, 1, 0],
                                                [0, 0, 0, 1]])

        GL.matriz_perspectiva = np.array([[GL.near/direita, 0, 0, 0],
                                          [0, GL.near/cima, 0, 0],
                                          [0, 0, -((GL.far+GL.near)/(GL.far-GL.near)), -
                                           ((2*GL.far*GL.near)/(GL.far-GL.near))],
                                          [0, 0, -1, 0]])

        GL.matriz_trans_cam = np.array([[1, 0, 0, position[0]],
                                        [0, 1, 0, position[1]],
                                        [0, 0, 1, position[2]],
                                        [0, 0, 0, 1]])

    @staticmethod
    def transform_in(translation, scale, rotation):
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_in será chamada quando se entrar em um nó X3D do tipo Transform
        # do grafo de cena. Os valores passados são a escala em um vetor [x, y, z]
        # indicando a escala em cada direção, a translação [x, y, z] nas respectivas
        # coordenadas e finalmente a rotação por [x, y, z, t] sendo definida pela rotação
        # do objeto ao redor do eixo x, y, z por t radianos, seguindo a regra da mão direita.
        # Quando se entrar em um nó transform se deverá salvar a matriz de transformação dos
        # modelos do mundo em alguma estrutura de pilha.

        if rotation:
            if (rotation[0] > 0):
                GL.matriz_rotacao = np.array([[1, 0, 0, 0],
                                              [0, math.cos(
                                                  rotation[3]), -math.sin(rotation[3]), 0],
                                              [0, math.sin(rotation[3]), math.cos(
                                                  rotation[3]), 0],
                                              [0, 0, 0, 1]])
            elif (rotation[1] > 0):
                GL.matriz_rotacao = np.array([[math.cos(rotation[3]), 0, math.sin(rotation[3]), 0],
                                              [0, 1, 0, 0],
                                              [-math.sin(rotation[3]), 0,
                                               math.cos(rotation[3]), 0],
                                              [0, 0, 0, 1]])
            else:
                GL.matriz_rotacao = np.array([[math.cos(rotation[3]), -math.sin(rotation[3]), 0, 0],
                                              [math.sin(rotation[3]), math.cos(
                                                  rotation[3]), 0, 0],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]])

        if scale:
            GL.matriz_escala_in = np.array([[scale[0], 0, 0, 0],
                                            [0, scale[1], 0, 0],
                                            [0, 0, scale[2], 0],
                                            [0, 0, 0, 1]])

        if translation:
            GL.matriz_trans_in = np.array([[1, 0, 0, translation[0]],
                                           [0, 1, 0, translation[1]],
                                           [0, 0, 1, translation[2]],
                                           [0, 0, 0, 1]])

    @staticmethod
    def transform_out():
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_out será chamada quando se sair em um nó X3D do tipo Transform do
        # grafo de cena. Não são passados valores, porém quando se sai de um nó transform se
        # deverá recuperar a matriz de transformação dos modelos do mundo da estrutura de
        # pilha implementada.

        if (len(GL.stack) > 0):
            GL.stack.pop()

    @staticmethod
    def triangleStripSet(point, stripCount, colors):
        """Função usada para renderizar TriangleStripSet."""
        # A função triangleStripSet é usada para desenhar tiras de triângulos interconectados,
        # você receberá as coordenadas dos pontos no parâmetro point, esses pontos são uma
        # lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x
        # do primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e assim
        # por diante. No TriangleStripSet a quantidade de vértices a serem usados é informado
        # em uma lista chamada stripCount (perceba que é uma lista). Ligue os vértices na ordem,
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.

        coords = []
        GL.matriz_M = np.matmul(
            np.matmul(GL.matriz_trans_in, GL.matriz_rotacao), GL.matriz_escala_in)
        GL.matriz_look_at = np.matmul(np.linalg.inv(
            GL.matriz_orien_cam), np.linalg.inv(GL.matriz_trans_cam))

        for i in range(0, len(point), 3):
            ponto = np.array([[point[i]],
                              [point[i+1]],
                              [point[i+2]],
                              [1]])

            ponto = np.matmul(GL.matriz_M, ponto)
            ponto = np.matmul(GL.matriz_look_at, ponto)
            ponto = np.matmul(GL.matriz_perspectiva, ponto)
            ponto /= ponto[3][0]
            ponto = np.matmul(GL.matriz_tela, ponto)
            coords.append(ponto)

        for s in range(stripCount[0]):
            for x in range(GL.width):
                for y in range(GL.height):
                    if (s+3 < len(coords)):
                        GL.triangleSet2D(coords[s:s+3], [x, y], colors)

    @staticmethod
    def indexedTriangleStripSet(point, index, colors):
        """Função usada para renderizar IndexedTriangleStripSet."""
        # A função indexedTriangleStripSet é usada para desenhar tiras de triângulos
        # interconectados, você receberá as coordenadas dos pontos no parâmetro point, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor
        # da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto, point[2]
        # o valor z da coordenada z do primeiro ponto. Já point[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedTriangleStripSet uma lista informando
        # como conectar os vértices é informada em index, o valor -1 indica que a lista
        # acabou. A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.

        GL.matriz_M = np.matmul(
            np.matmul(GL.matriz_trans_in, GL.matriz_rotacao), GL.matriz_escala_in)

        GL.matriz_tela = np.array([[GL.width/2, 0, 0, GL.width/2],
                                   [0, -GL.height/2, 0, GL.height/2],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])

        coords = []

        GL.matriz_look_at = np.matmul(np.linalg.inv(
            GL.matriz_orien_cam), np.linalg.inv(GL.matriz_trans_cam))

        for i in range(0, len(point), 3):
            ponto = np.array([[point[i]],
                              [point[i+1]],
                              [point[i+2]],
                              [1]])

            ponto = np.matmul(GL.matriz_M, ponto)
            ponto = np.matmul(GL.matriz_look_at, ponto)
            ponto = np.matmul(GL.matriz_perspectiva, ponto)
            ponto /= ponto[3][0]
            ponto = np.matmul(GL.matriz_tela, ponto)
            coords.append(ponto)

        for ind in index:
            if (index[ind+2] == -1):
                break
            for x in range(GL.width):
                for y in range(GL.height):
                    GL.triangleSet2D(coords[ind:ind+3], [x, y], colors)

    @staticmethod
    def box(size, colors):
        """Função usada para renderizar Boxes."""
        # A função box é usada para desenhar paralelepípedos na cena. O Box é centrada no
        # (0, 0, 0) no sistema de coordenadas local e alinhado com os eixos de coordenadas
        # locais. O argumento size especifica as extensões da caixa ao longo dos eixos X, Y
        # e Z, respectivamente, e cada valor do tamanho deve ser maior que zero. Para desenhar
        # essa caixa você vai provavelmente querer tesselar ela em triângulos, para isso
        # encontre os vértices e defina os triângulos.

        x, y, z = size[0], size[1], size[2]

        ponto1 = (-x, -y, z)
        ponto2 = (x, -y, z)
        ponto3 = (x, y, z)
        ponto4 = (-x, y, z)
        ponto5 = (-x, y, -z)
        ponto6 = (x, y, -z)
        ponto7 = (-x, -y, -z)
        ponto8 = (x, -y, -z)

        pontos = [
            ponto1, ponto2, ponto3, ponto3, ponto4, ponto1, ponto7, ponto1, ponto4,
            ponto4, ponto5, ponto7, ponto8, ponto7, ponto5, ponto5, ponto6, ponto8,
            ponto8, ponto6, ponto2, ponto2, ponto6, ponto3, ponto6, ponto5, ponto4,
            ponto4, ponto3, ponto6, ponto8, ponto7, ponto1, ponto1, ponto2, ponto8
        ]

        pontos = list(sum(pontos, ()))
        GL.triangleSet(pontos, colors)

    @staticmethod
    def pintaDentro(vertices, ponto, colors, color, colorPerVertex, texture=False, texVertex=None, img=None):

        x, y = ponto[0], ponto[1]

        x1, x2, x3 = vertices[0][0][0], vertices[1][0][0], vertices[2][0][0]
        y1, y2, y3 = vertices[0][1][0], vertices[1][1][0], vertices[2][1][0]

        L0 = False
        L1 = False
        L2 = False
        x = x + 0.5
        y = y + 0.5

        if (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1) >= 0:
            L0 = True

        if (x - x2) * (y3 - y2) - (y - y2) * (x3 - x2) >= 0:
            L1 = True

        if (x - x3) * (y1 - y3) - (y - y3) * (x1 - x3) >= 0:
            L2 = True

        r = int(colors["diffuseColor"][0]*255)
        g = int(colors["diffuseColor"][1]*255)
        b = int(colors["diffuseColor"][2]*255)

        if(L1 and L2 and L0):
            alpha = (-(x - x2)*(y3 - y2) + (y - y2)*(x3 - x2)) / \
                (-(x1 - x2)*(y3 - y2) + (y1 - y2)*(x3 - x2))
            betha = (-(x - x3)*(y1 - y3) + (y - y3)*(x1 - x3)) / \
                (-(x2 - x3)*(y1 - y3) + (y2 - y3)*(x1 - x3))
            gamma = 1 - (alpha + betha)

            if colorPerVertex and color:
                r = (color[0][0]*alpha + color[1][0]*betha + color[2][0]*gamma)
                g = (color[0][1]*alpha + color[1][1]*betha + color[2][1]*gamma)
                b = (color[0][2]*alpha + color[1][2]*betha + color[2][2]*gamma)
                r = r * 255
                g = g * 255
                b = b * 255

            if texture:
                tex_x = (texVertex[0][0]*alpha + texVertex[1][0]
                         * betha + texVertex[2][0]*gamma)*img.shape[0]
                tex_y = (texVertex[0][1]*alpha + texVertex[1][1]
                         * betha + texVertex[2][1]*gamma)*img.shape[1]

                r, g, b, f = img[int(-tex_y)][int(tex_x)]

            gpu.GPU.draw_pixels([int(x), int(y)], gpu.GPU.RGB8, [r, g, b])

    @staticmethod
    def indexedFaceSet(coord, coordIndex, colorPerVertex, color, colorIndex,
                       texCoord, texCoordIndex, colors, current_texture):
        """Função usada para renderizar IndexedFaceSet."""
        # A função indexedFaceSet é usada para desenhar malhas de triângulos. Ela funciona de
        # forma muito simular a IndexedTriangleStripSet porém com mais recursos.
        # Você receberá as coordenadas dos pontos no parâmetro cord, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim coord[0] é o valor
        # da coordenada x do primeiro ponto, coord[1] o valor y do primeiro ponto, coord[2]
        # o valor z da coordenada z do primeiro ponto. Já coord[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedFaceSet uma lista de vértices é informada
        # em coordIndex, o valor -1 indica que a lista acabou.
        # A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante.
        # Adicionalmente essa implementação do IndexedFace aceita cores por vértices, assim
        # se a flag colorPerVertex estiver habilitada, os vértices também possuirão cores
        # que servem para definir a cor interna dos poligonos, para isso faça um cálculo
        # baricêntrico de que cor deverá ter aquela posição. Da mesma forma se pode definir uma
        # textura para o poligono, para isso, use as coordenadas de textura e depois aplique a
        # cor da textura conforme a posição do mapeamento. Dentro da classe GPU já está
        # implementadado um método para a leitura de imagens

        tri = []
        colorAbs = []
        rgb = []
        text = []

        GL.matriz_M = np.matmul(
            np.matmul(GL.matriz_trans_in, GL.matriz_rotacao), GL.matriz_escala_in)

        if (len(GL.stack) > 1):
            GL.matriz_M = np.matmul(GL.stack[-1], GL.matriz_M)

        if current_texture:
            image = gpu.GPU.load_texture(current_texture[0])

        GL.stack.append(GL.matriz_M)

        GL.matriz_tela = np.array(
            [[GL.width/2, 0, 0, GL.width/2], [0, -GL.height/2, 0, GL.height/2], [0, 0, 1, 0], [0, 0, 0, 1]])
        vertex = []

        GL.matriz_look_at = np.matmul(np.linalg.inv(
            GL.matriz_orien_cam), np.linalg.inv(GL.matriz_trans_cam))

        for i in range(0, len(coord), 3):
            ponto = np.array([[coord[i]], [coord[i+1]], [coord[i+2]], [1]])

            ponto = np.matmul(GL.matriz_M, ponto)
            ponto = np.matmul(GL.matriz_look_at, ponto)
            ponto = np.matmul(GL.matriz_perspectiva, ponto)
            ponto /= ponto[3][0]
            ponto = np.matmul(GL.matriz_tela, ponto)
            vertex.append(ponto)

        if coord and coordIndex:
            for v in coordIndex:
                if v < 0:
                    for x in range(GL.width):
                        for y in range(GL.height):
                            GL.pintaDentro(
                                tri, [x, y], colors, colorAbs, colorPerVertex)

                    colorAbs = []
                    tri = []
                    continue
                tri.append(vertex[v])

        if colorPerVertex and colorIndex:

            for c in range(0, len(color), 3):
                rgb.append([color[c], color[c+1], color[c+2]])

            for v, c in zip(coordIndex, colorIndex):
                if v < 0 or c < 0:
                    for x in range(GL.width):
                        for y in range(GL.height):
                            GL.pintaDentro(
                                tri, [x, y], colors, colorAbs, colorPerVertex)

                    colorAbs = []
                    tri = []
                    continue
                colorAbs.append(rgb[c])
                tri.append(vertex[v])

        if (texCoord and texCoordIndex):
            textura = []
            for p in range(0, len(texCoord), 2):
                textura.append([texCoord[p], texCoord[p+1]])
            text = []
            for v, t in zip(coordIndex, texCoordIndex):
                if v < 0:
                    for x in range(GL.width):
                        for y in range(GL.height):
                            GL.pintaDentro(tri, [
                                           x, y], colors, colorAbs, colorPerVertex, texture=True, texVertex=text, img=image)
                    tri = []
                    continue
                tri.append(vertex[v])
                text.append(textura[t])
