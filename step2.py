import numpy as np
import matplotlib.pyplot as plt
import re


filename = "./testes/parado_mexendo_cadeira.txt"
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

ax = [float(linha[1]) for linha in dados[1:]] #m/s²
ay = [float(linha[2]) for linha in dados[1:]] #m/s²
az = [float(linha[3]) for linha in dados[1:]] #m/s²

gx = [float(linha[4]) for linha in dados[1:]] #??
gy = [float(linha[5]) for linha in dados[1:]] #??
gz = [float(linha[6]) for linha in dados[1:]] #??



plt.plot(t, ax, label="ax")
plt.plot(t, ay, label="ay")
plt.plot(t, az, label="az")
plt.legend() 
plt.show()