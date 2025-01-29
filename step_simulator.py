import numpy as np
import matplotlib.pyplot as plt
import re


"""#Fazendo tudo ao contrário agora

Entrada do programa: Ax, Ay, Az e T
"""
def aplicar_media_movel(data, window_size):
    if window_size > len(data):
        raise ValueError("O tamanho da janela não pode ser maior que o array.")
    
    suavizado = data[:]  # Faz uma cópia dos dados originais
    metade_janela = window_size // 2

    # Calcula a média móvel para cada ponto com janela completa
    for i in range(metade_janela, len(data) - metade_janela):
        window = data[i - metade_janela:i + metade_janela + 1]
        suavizado[i] = sum(window) / window_size
    
    return suavizado

def filtro_passa_baixa(data, alpha):
    if not (0 < alpha <= 1):
        raise ValueError("O valor de alpha deve estar entre 0 e 1.")
    
    filtrado = [data[0]]  # Inicia com o primeiro valor do array
    for i in range(1, len(data)):
        y = alpha * data[i] + (1 - alpha) * filtrado[-1]
        filtrado.append(y)
    
    return filtrado

filename = "./testes/parado_mexendo_cadeira.txt"
with open(filename, 'r') as f:
    linhas = f.readlines()  # Lê todas as linhas do arquivo
    
    dados = []
    for linha in linhas:
        linha = linha.strip()
        partes = re.split(r'\t|,|;', linha)
        dados.append(partes)


dt = [float(linha[0])/1000 for linha in dados[1:]]
media = sum(dt) / len(dt)
variancia = sum((x - media) ** 2 for x in dt) / (len(dt) - 1)  # Amostral
desvio_padrao = variancia ** 0.5
print(f"Média Dt: {media} +- {desvio_padrao}")

# Criando vetor "T"
t = [0]
for i in dt[1:]:
   t.append(t[-1] + i)

world = 3
g=1 #9.80665/16384.0
ax = [float(linha[1])*g for linha in dados[1:]]
ay = [float(linha[2])*g for linha in dados[1:]]
az = [float(linha[3])*g for linha in dados[1:]]


# # !Filtros
# axqn = aplicar_media_movel(ax, 3)
# ayqn = aplicar_media_movel(ay, 3)
# azqn = aplicar_media_movel(az, 3)

# axn = filtro_passa_baixa(axqn, 0.01)
# ayn = filtro_passa_baixa(ayqn, 0.01)
# azn = filtro_passa_baixa(azqn, 0.01)


plt.plot(t, ax, label="x")
plt.plot(t, ay, label="y")
plt.plot(t, az, label="z")
# plt.ylim([-25, 25])
plt.legend()
plt.title("Gráfico de Acelerações (m/s²)")
plt.show()




# plt.scatter(ax, ay)
# plt.title("Diagrama GG")
# plt.show()
#quit()
"""Objetivo 1: Calcular as velocidades em cada ponto \\

v(ti) = v(t0) + ∫a(t)*dt
"""

#TODO: tornar os indices consistentes para o vetor v que retorna
def rectangular_integration(v0, a, t):
  dt = t[1]-t[0]
  v = []
  v.append(v0)

  for i in range(len(a)):
    v.append(v[-1] + a[i]*dt)

  return v[1:]

def trapezoid_integration(v0, a, t):
  dt = t[1]-t[0]
  v = []
  v.append(v0)

  for i in range(len(a)-1):
    v.append(v[-1] + (a[i] + a[i+1])*dt/2)

  return v


def simpson_(v0, a, t):
    dt = t[1] - t[0]  # Supõe-se espaçamento uniforme
    n = len(a)
    v = [v0]  # Inicializa a lista de velocidades com o valor inicial

    # Calcula a integral acumulativa usando Simpson
    for i in range(1, n-1):
        if i % 2 == 1:  # Aplica o método de Simpson apenas nos pontos adequados
            v_next = v[-1] + (a[i-1] + 4*a[i] + a[i+1]) * dt / 6
        else:  # Usa o método do trapézio para os outros pontos
            v_next = v[-1] + (a[i] + a[i+1]) * dt / 2
        v.append(v_next)

    return v

def runge_kutta_integration(v0, a, t):
    v = [v0]

    for i in range(1, len(a)):  # Começa em 1 para evitar problema de índice
        dt = float(t[i] - t[i-1])
        ai = float(a[i])
        k1 = ai
        k2 = ai + dt * k1 / 2
        k3 = ai + dt * k2 / 2
        k4 = ai + dt * k3
        v_next = v[-1] + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
        v.append(v_next)

    return v


vx = runge_kutta_integration(0, ax, t)
vy = runge_kutta_integration(0, ay, t)
vz = runge_kutta_integration(0, az, t)

# plt.plot(t, runge_kutta_integration(0, axqn, t), label="vx_quase_novo")
# plt.plot(t, runge_kutta_integration(0, ayqn, t), label="vy_quase_novo")
# # plt.plot(t, runge_kutta_integration(0, azqn, t), label="vz_quase_novo")

#plt.plot(t, runge_kutta_integration(0, axn, t), label="vx_novo")
# plt.plot(t, runge_kutta_integration(0, ayn, t), label="vy_novo")
# # plt.plot(t, runge_kutta_integration(0, azn, t), label="vz_novo")

plt.plot(t, vx, "-o", label="vx")
plt.plot(t, vy, label="vy")
plt.plot(t, vz, label="vz")
plt.title("Gráfico de velocidades (km/h)")
plt.legend()
plt.show()
# quit()
"""Calculando os pontos (x, y)"""

x = runge_kutta_integration(0, vx, t)
y = runge_kutta_integration(0, vy, t)
z = runge_kutta_integration(0, vz, t)

# Plotando o circuito
plt.figure(figsize=(10, 5))
# plt.plot(circuito_x, circuito_y, '-o', label="exato")
plt.plot(x, z, '-o', label="Calculado")
plt.axis("equal")
plt.xlabel("Posição X")
plt.ylabel("Posição Y")
plt.legend()
plt.show()

quit()