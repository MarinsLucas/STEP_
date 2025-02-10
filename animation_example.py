import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

# Criar figura e eixo
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'r-', animated=True)

# Função de inicialização
def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(-1, 1)
    return ln,

# Função de atualização do gráfico
def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame) + random.uniform(-0.1, 0.1))  # Simulando ruído
    ln.set_data(xdata, ydata)
    return ln,

ani = animation.FuncAnimation(fig, update, frames=np.linspace(0, 10, 100), init_func=init, blit=True)
plt.show()
