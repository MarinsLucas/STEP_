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
# Exemplo de uso
if __name__ == "__main__":

    filename = "./testes/RetaCarro.txt"
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
    ax = [float(linha[1])*0 for linha in dados[1:]]  # m/s²
    ay = [float(linha[2])+ 0.27 for linha in dados[1:]]  # m/s²
    az = [float(linha[3]) for linha in dados[1:]]  # m/s²

    # Extrai as velocidades angulares (giroscópio)
    gx = [float(linha[4]) for linha in dados[1:]]  # rad/s
    gy = [float(linha[5]) for linha in dados[1:]]  # rad/s
    gz = [float(linha[6])+0.02 for linha in dados[1:]]  # rad/s

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
    gyro_measurement_noise = np.eye(3) * 0.1  # Ruído da medição

    acc_kf = KalmanFilter(acc_initial_state, acc_initial_covariance, acc_process_noise, acc_measurement_noise)
    gyro_kf = KalmanFilter(gyro_initial_state, gyro_initial_covariance, gyro_process_noise, gyro_measurement_noise)    # Aplicando o filtro de Kalman às medições
    
    acc_filtered_states = []
    gyro_filtered_states = []
    for i in range(len(acc_measurements)):
        acc_kf.predict(dts[i])
        acc_kf.update(acc_measurements[i])
        acc_filtered_states.append(acc_kf.state)

        gyro_kf.predict(dts[i])
        gyro_kf.update(gyro_measurements[i])
        gyro_filtered_states.append(gyro_kf.state)
    

    acc_filtered_states = np.array(acc_filtered_states)
    gyro_filtered_states = np.array(gyro_filtered_states)

    acc_filtered_states = np.where(abs(acc_filtered_states) >= 0.5, acc_filtered_states, 0)

    # Função para calcular a matriz de rotação com base no giroscópio
    def get_rotation_matrix(gyro_data, dt):
        gx, gy, gz = gyro_data
        theta_x = gx * dt
        theta_y = gy * dt
        theta_z = gz * dt
        
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

    # Função para rotacionar a aceleração
    def rotate_acceleration(accel_data, gyro_data, dt):
        R = get_rotation_matrix(gyro_data, dt)
        accel_rotated = np.dot(R, accel_data)
        return accel_rotated

    naccel = []
    for i in range(len(acc_filtered_states)):
        ac = [0, acc_filtered_states[i, 0], 0]
        accel_rotated = rotate_acceleration(acc_filtered_states[i], gyro_filtered_states[i], dts[i])  # Aceleração rotacionada
        naccel.append(accel_rotated)
    print(naccel)

    plot_acc = True; 
    plot_gyro = False; 

    if plot_acc:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
        ax1.plot(t, ax, label="ax", color='r')
        ax1.plot(t, acc_filtered_states[:, 0], label="f_ax", color='b')
        ax1.legend()
        ax1.grid()
        ax1.set_title('Gráfico de ax')

        ax2.plot(t, ay, label="ay", color='g')
        ax2.plot(t, acc_filtered_states[:, 1], label="f_ay", color='b')
        ax2.legend()
        ax2.set_title('Gráfico de ay')
        
        ax3.plot(t, az, label="az", color='y')
        ax3.plot(t, acc_filtered_states[:, 2], label="f_az", color='b')
        ax3.legend()
        ax3.set_title('Gráfico de az')
        plt.tight_layout()
        plt.show()

        plt.scatter(ax, ay)
        plt.axis('equal')
        plt.title("Gráfico GG")
        plt.show()

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
    
   

    vx = rectangular_integration(0, acc_filtered_states[:, 0], t)
    vy = rectangular_integration(0, acc_filtered_states[:, 1], t)
    vz = rectangular_integration(0, acc_filtered_states[:, 2], t)
    
    

    plt.plot(t, vx, label="vx")
    plt.plot(t, vy, label="vy")
    # plt.plot(t, vz, label="vz")
    plt.title("Gráfico de velocidades (m/s)")
    plt.legend()
    plt.show()


    x = rectangular_integration(0, vx, t)
    y = rectangular_integration(0, vy, t)
    z = rectangular_integration(0, vz, t)

    x = np.array(x)
    y = np.array(y)
    # Plotando o circuito
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, '-o', label="Calculado")
    # plt.axis("equal")
    plt.xlabel("Posição X")
    plt.ylabel("Posição Y")
    plt.legend()
    plt.show()

   

    # Criar figura e eixo
    fig, axis = plt.subplots()
    xdata, ydata = [], []
    ln, = plt.plot([], [], 'r-', animated=False)

    # Função de inicialização
    def init():
        axis.set_xlim(-1000,1000)
        axis.set_ylim(-1000, 1000)
        return ln,

    # Função de atualização do gráfico
    def update(frame):
        print(frame)
        step = 100
        xdata.append(float(x[frame * step]))  # Pega o valor de x a cada 100 pontos
        ydata.append(float(y[frame * step]))  # Pega o valor de y correspondente
        ln.set_data(xdata, ydata)
        return ln,


    ani = animation.FuncAnimation(fig, update, frames=range(len(x)//100), init_func=init, blit=False)
    plt.title("Trajeto em tempo real")
    plt.show()
