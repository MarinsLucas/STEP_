import serial
import time

# Configuração da porta serial e arquivo
porta_serial = "/dev/ttyACM0"  # Substitua pela porta correta
taxa_baude = 115200      # Deve ser a mesma configurada no Arduino
arquivo_saida = "dados_serial.txt"
arduino = serial.Serial(porta_serial, taxa_baude)
try:
    # Abrindo a conexão serial
    arduino = serial.Serial(porta_serial, taxa_baude)
    print(f"Conectado a {porta_serial}")
    time.sleep(2)  # Aguarda inicialização do Arduino
    
    mensagem = input("Digite algo para enviar ao Arduino: ")
    arduino.write((mensagem + "\n").encode())  # Envia os dados
    print(f"Enviado: {mensagem}")

    # Criando/abrindo o arquivo para salvar os dados
    with open(arquivo_saida, "w") as arquivo:
        print("Iniciando leitura de dados... Pressione Ctrl+C para parar.")
        while True:
            if arduino.in_waiting > 0:  # Verifica se há dados disponíveis
                dado = arduino.readline().decode('utf-8', errors='ignore').strip()
                print(f"Dado recebido: {dado}")
                arquivo.write(dado + "\n")  # Salva o dado no arquivo
                
except KeyboardInterrupt:
    print("\nLeitura interrompida pelo usuário.")
except Exception as e:
    print(f"Erro: {e}")
finally:
    arduino.close()
    print("Conexão serial encerrada.")
