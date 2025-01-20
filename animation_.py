import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def carregar_dados(arquivo):
    # Carregar os dados do arquivo
    dados = np.genfromtxt(arquivo, delimiter=';', invalid_raise=False)
    tempos = dados[:, 0]
    giroscopio = dados[:, -3:]  # Últimas 3 colunas
    return tempos, giroscopio

def calcular_angulos(tempos, giroscopio):
    # Inicializar os ângulos
    angulos = np.zeros((len(tempos), 3))  # pitch, roll, yaw
    for i in range(1, len(tempos)):
        dt = tempos[i] - tempos[i - 1]
        angulos[i] = angulos[i - 1] + giroscopio[i] * dt
    return angulos

def criar_animacao(angulos):
    # Configuração da figura 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Animação de Rotação")
    
    # Vetores de referência para os eixos
    vetor_x = np.array([1, 0, 0])  # Eixo X
    vetor_y = np.array([0, 1, 0])  # Eixo Y
    vetor_z = np.array([0, 0, 1])  # Eixo Z
    
    # Linhas para os eixos rotacionados
    linha_x, = ax.plot([0, vetor_x[0]], [0, vetor_x[1]], [0, vetor_x[2]], color='r', lw=2, label="Eixo X")
    linha_y, = ax.plot([0, vetor_y[0]], [0, vetor_y[1]], [0, vetor_y[2]], color='g', lw=2, label="Eixo Y")
    linha_z, = ax.plot([0, vetor_z[0]], [0, vetor_z[1]], [0, vetor_z[2]], color='b', lw=2, label="Eixo Z")
    ax.legend()

    def atualizar(frame):
        # Atualizar os vetores com base nos ângulos de rotação
        pitch, roll, yaw = angulos[frame]
        rotacao = calcular_matriz_rotacao(pitch, roll, yaw)
        
        # Rotacionar os vetores de cada eixo
        vetor_x_rotado = rotacao @ vetor_x
        vetor_y_rotado = rotacao @ vetor_y
        vetor_z_rotado = rotacao @ vetor_z
        
        # Atualizar as linhas na animação
        linha_x.set_data([0, vetor_x_rotado[0]], [0, vetor_x_rotado[1]])
        linha_x.set_3d_properties([0, vetor_x_rotado[2]])
        
        linha_y.set_data([0, vetor_y_rotado[0]], [0, vetor_y_rotado[1]])
        linha_y.set_3d_properties([0, vetor_y_rotado[2]])
        
        linha_z.set_data([0, vetor_z_rotado[0]], [0, vetor_z_rotado[1]])
        linha_z.set_3d_properties([0, vetor_z_rotado[2]])
        
        return linha_x, linha_y, linha_z

    # Criar a animação
    anim = FuncAnimation(fig, atualizar, frames=len(angulos), interval=50, blit=True)
    plt.show()

def calcular_matriz_rotacao(pitch, roll, yaw):
    # Matrizes de rotação para cada eixo
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])
    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx  # Combinação das rotações

# Caminho para o arquivo contendo os dados
arquivo = "teste no skate.csv"
tempos, giroscopio = carregar_dados(arquivo)
angulos = calcular_angulos(tempos, giroscopio)
criar_animacao(angulos)
