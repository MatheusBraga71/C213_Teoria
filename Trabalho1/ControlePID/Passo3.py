import numpy as np
import control as cnt
import matplotlib.pyplot as plt
from scipy.io import loadmat

# escrevendo a função de transferência da planta
mat = loadmat('TransferFunction11.mat')

degrau = mat.get('degrau')
saida = mat.get('saida')
t1 = mat.get('t')

k = 3.18892999
tau = 11.85
Theta = 4.0499

num = np.array([k])
den = np.array([tau, 1])
H = cnt.tf(num, den)
n_pade = 20
(num_pade, den_pade) = cnt.pade(Theta, n_pade)
H_pade = cnt.tf(num_pade, den_pade)
Hs = cnt.series(H, H_pade)

t = np.linspace(0, 70, 100)
(t, y) = cnt.step_response(11*Hs, t)
plot = plt.plot(t, y, 'r', label='saida estimada')
plt.xlabel(' t [ s ] ')
plt.ylabel('Amplitude')

plot1 = plt.plot(t1.T, saida, label='saida original')
plot2 = plt.plot(t1.T, degrau, label='degrau de entrada')
plt.xlabel('t [s]')
plt.ylabel('Amplitude')
plt.legend(loc='upper left')
plt.title('Reposta original x Resposta estimada')
plt.grid()
plt.show()
