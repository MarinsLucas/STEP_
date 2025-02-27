import numpy as np
import re
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import matplotlib.animation as animation

class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state = initial_state  # Estado estimado (x)
        self.covariance = initial_covariance  # Covariância do estado (P)
        self.process_noise = process_noise  # Ruído do processo (Q)
        self.measurement_noise = measurement_noise  # Ruído da medição (R)
        
        self.state_transition = np.eye(len(initial_state))
        # Matriz de transição de estado (A): Assume-se que o estado não muda drasticamente
        
        # Matriz de observação (H): Assume-se que medimos o estado diretamente
        self.observation_matrix = np.eye(len(initial_state))
    
    def predict(self, dt):
    
        # Predição do estado
        self.state = np.dot(self.state_transition, self.state)
        
        # Predição da covariância
        self.covariance = np.dot(np.dot(self.state_transition, self.covariance), self.state_transition.T) + self.process_noise

        self.observation_matrix=np.array([
            [1.0, dt, 0.0], 
            [0.0, 1.0, dt], 
            [0.0, 0.0, 1.0]
        ])
    
    def update(self, measurement):
        # Calcula o ganho de Kalman (K)
        innovation_covariance = np.dot(np.dot(self.observation_matrix, self.covariance), self.observation_matrix.T) + self.measurement_noise
        kalman_gain = np.dot(np.dot(self.covariance, self.observation_matrix.T), np.linalg.inv(innovation_covariance))
        
        # Atualiza o estado estimado
        self.state = self.state + np.dot(kalman_gain, (measurement - np.dot(self.observation_matrix, self.state)))
        
        # Atualiza a covariância
        self.covariance = self.covariance - np.dot(np.dot(kalman_gain, self.observation_matrix), self.covariance)


def rectangular_integration(v0, a, t):
        dt = t[1]-t[0]
        v = []
        v.append(v0)

        for i in range(len(a)):
            v.append(v[-1] + a[i]*dt)

        return v[1:]

def trapezoidal_integration(v0, a, t):
        dt = t[1] - t[0]
        v = [v0]
        for i in range(len(a) - 1):
            v.append(v[-1] + 0.5 * (a[i] + a[i + 1]) * dt)
        return v


def get_rotation_matrix(gyro_data, dt):
    gx, gy, gz = gyro_data
    theta_x = gx 
    theta_y = gy 
    theta_z = gz 

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]])
    
    R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                    [0, 1, 0],
                    [-np.sin(theta_y), 0, np.cos(theta_y)]])
    
    R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def rotate_acceleration(accel_data, gyro_data, dt):
        R = get_rotation_matrix(gyro_data, dt)
        accel_rotated = np.dot(R, accel_data)
        return accel_rotated



# Exemplo de uso
if __name__ == "__main__":

    filename = "./testes/CarroAndando30.txt"
    with open(filename, 'r') as f:
        linhas = f.readlines()  # Lê todas as linhas do arquivo
        
        dados = []
        for linha in linhas:
            linha = linha.strip()
            partes = re.split(r'\t|,|;', linha)
            dados.append(partes)

    dts = [float(linha[0])/1000 for linha in dados[1:]] #s

    t = [0] #t
    for i in dts[1:]:
        t.append(t[-1] + i)
        
    # Extrai as acelerações
    ax = [float(linha[1]) - 0.7 for linha in dados[1:]]  # m/s²
    ay = [float(linha[2]) + 0.2 for linha in dados[1:]]  # m/s²
    az = [float(linha[3]) + 9.60 for linha in dados[1:]]  # m/s²

    print(sum(az)/len(az))

    # Extrai as velocidades angulares (giroscópio)
    gx = [float(linha[4]) + 0.105 for linha in dados[1:]]  # rad/s+
    gy = [float(linha[5]) - 0.02 for linha in dados[1:]]  # rad/s
    gz = [float(linha[6]) + 0.02 for linha in dados[1:]]  # rad/s

    # Cria a matriz measurements
    acc_measurements = np.column_stack((ax, ay, az))
    gyro_measurements = np.column_stack((gx, gy, gz))

    # Inicialização do filtro de Kalman
    acc_initial_state = np.array([ax[0], ay[0], az[0]])  # Estado inicial (ax, ay, az)
    acc_initial_covariance = np.eye(3) * 0.1  # Covariância inicial
    acc_process_noise = np.eye(3) * 0.001  # Ruído do processo #Quanto maior, o estado muda mais rápido
    acc_measurement_noise = np.eye(3) * 0.1  # Ruído da medição #Quanto maior, menos a gente confia na medição
    
        # Parâmetros do filtro de Kalman para o giroscópio
    gyro_initial_state = np.array([0.0, 0.0, 0.0])  # Estado inicial (gx, gy, gz)
    gyro_initial_covariance = np.eye(3) * 0.1  # Covariância inicial
    gyro_process_noise = np.eye(3) * 0.0001  # Ruído do processo
    gyro_measurement_noise = np.eye(3) * 0.5  # Ruído da medição

    
    acc_kf = KalmanFilter(acc_initial_state, acc_initial_covariance, acc_process_noise, acc_measurement_noise)
    gyro_kf = KalmanFilter(gyro_initial_state , gyro_initial_covariance, gyro_process_noise, gyro_measurement_noise)    # Aplicando o filtro de Kalman às medições
    
    acc_filtered_states = []
    gyro_filtered_states = []
    velocidades = [[0,0,0]]
    posicoes = [[0,0,0]]
    orientacoes = [[1,0,0]]
    orientacoes_q = [R.identity().as_quat()]
    GRAVIDADE_GLOBAL = np.array([0, 0, 9.81]) 
    aceleracao_real_antiga = [0,0,0]

    for i in range(len(acc_measurements)):
        acc_kf.predict(dts[i])
        acc_kf.update(acc_measurements[i])
        acc_filtered_states.append(acc_kf.state)
        
        gyro_kf.predict(dts[i])
        gyro_kf.update(gyro_measurements[i])
        gyro_filtered_states.append(gyro_kf.state)
       
        delta_angle = gyro_kf.state * dts[i]
        delta_rot = R.from_rotvec(delta_angle)

        nova_rotacao = delta_rot*R.from_quat(orientacoes_q[-1])
        orientacoes_q.append(nova_rotacao.as_quat())

        orientacoes.append(nova_rotacao.apply(orientacoes[0]))
       
        gravidade_no_sensor = nova_rotacao.inv().apply(GRAVIDADE_GLOBAL)

        aceleracao_real = acc_kf.state - gravidade_no_sensor

        velocidades.append(velocidades[-1] + dts[i]*(aceleracao_real + aceleracao_real_antiga)/2)
        aceleracao_real_antiga = aceleracao_real
        vel_dir = orientacoes[-1] * np.linalg.norm(velocidades[-1])
        
        posicoes.append(posicoes[-1] + dts[i]*vel_dir)

    acc_filtered_states = np.array(acc_filtered_states)
    gyro_filtered_states = np.array(gyro_filtered_states)
    plot_acc = False
    plot_gyro = False
    plot_pos = True
    plot_orient = False
    plot_velo = True

    if plot_acc:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
        ax1.plot(t, ax, label="ax", color='r')
        ax1.plot(t, acc_filtered_states[:, 0], label="f_ax", color='b')
        ax1.legend()
        ax1.grid()
        ax1.set_title('Gráfico de ax (latitudinal)')

        ax2.plot(t, ay, label="ay", color='g')
        ax2.plot(t, acc_filtered_states[:, 1], label="f_ay", color='b')
        ax2.legend()
        ax2.set_title('Gráfico de ay (longitudinal)')
        
        ax3.plot(t, az, label="az", color='y')
        ax3.plot(t, acc_filtered_states[:, 2], label="f_az", color='b')
        ax3.legend()
        ax3.set_title('Gráfico de az (vertical)')
        plt.tight_layout()
        plt.show()

        """ plt.scatter(ax, ay)
        plt.axis('equal')
        plt.title("Gráfico GG")
        plt.show() """

    if plot_gyro:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
        ax1.plot(t, gx, label="gx", color='r')
        ax1.plot(t, gyro_filtered_states[:, 0], label="f_ax", color='b')
        ax1.legend()
        ax1.grid()
        ax1.set_title('Gráfico de gx')

        ax2.plot(t, gy, label="gy", color='g')
        ax2.plot(t, gyro_filtered_states[:, 1], label="f_ay", color='b')
        ax2.legend()
        ax2.set_title('Gráfico de ay')
        
        ax3.plot(t, gz, label="gz", color='y')
        ax3.plot(t, gyro_filtered_states[:, 2], label="f_az", color='b')
        ax3.legend()
        ax3.set_title('Gráfico de az')
        plt.tight_layout()

        plt.show()

    t.append(t[-1]+dts[0]) # necessário para os plots seguintes
    posicoes = np.array(posicoes)
    velocidades= np.array(velocidades)
    
    if plot_velo:
        plt.plot(t, velocidades[:, 0], label="vx")
        plt.plot(t, velocidades[:, 1], label="vy")
        plt.plot(t, velocidades[:, 2], label="vz")
        plt.legend()
        plt.title("Velocidades")
        plt.show()


    if plot_pos:
        #plt.plot(posicoes[:,0], posicoes[:, 1])
        plt.plot(t, posicoes[:, 0])
        plt.plot(t, posicoes[:, 1])
        plt.plot(t, posicoes[:, 2])
        plt.title("Posicoes (m)")
        plt.show()
 
    plt.plot(posicoes[:,0], posicoes[:, 1])
    plt.quiver(posicoes[:, 0], posicoes[:, 1], velocidades[:,0], velocidades[:, 1], angles="xy", scale_units="xy", scale=100, color="r", width=0.003, label="Velocidade")
    plt.show()

    orientacoes = np.array(orientacoes)
    if plot_orient:
        plt.plot(t, (orientacoes[:, 0]*180/np.pi), label="o x")
        plt.plot(t, (orientacoes[:, 1]*180/np.pi), label="o y")
        plt.plot(t, (orientacoes[:, 2]*180/np.pi), label="o z")
        plt.title("Orientação (°)")
        plt.legend()
        plt.show()


    orientacoes_q = np.array(orientacoes_q)
    if plot_orient:
        plt.plot(t, (orientacoes_q[:, 0]*180/np.pi), label="q.w")
        plt.plot(t, (orientacoes_q[:, 1]*180/np.pi), label="q.x")
        plt.plot(t, (orientacoes_q[:, 2]*180/np.pi), label="q.y")
        plt.plot(t, (orientacoes_q[:, 3]*180/np.pi), label="q.z")
        plt.title("Orientação (°)")
        plt.legend()
        plt.show()
    # Criar figura e eixo
    fig, axis = plt.subplots()
    xdata, ydata = [], []
    ln, = plt.plot([], [], 'o-', animated=False)

    # Função de inicialização
    def init():
        axis.set_xlim(-200,200)
        axis.set_ylim(-200, 200)
        return ln,

    # Função de atualização do gráfico
    def update(frame):
        print(frame)
        step = 100
        xdata.append(float(posicoes[frame * step, 0]))  # Pega o valor de x a cada 100 pontos
        ydata.append(float(posicoes[frame * step, 1]))  # Pega o valor de y correspondente
        ln.set_data(xdata, ydata)
        return ln,


    ani = animation.FuncAnimation(fig, update, frames=range(len(t)//100), init_func=init, blit=False)
    plt.title("Trajeto em tempo real")
    plt.show()
