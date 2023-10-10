import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# considerando uma função de transferencia em malha aberta FT=k/(tau*s+1)
k = 3.18892999
tau = 11.85
Theta = 4.04999999

# Fazendo os cálculos de Kp, Ti, Td usando o método KN

kp_zn = (1.2*tau)/(k*Theta)

ti_zn = 2*Theta

td_zn = 0.5*Theta

print('Kp calculado por ZN = ', kp_zn)
print('Ti calculado por ZN = ', ti_zn)
print('Td calculado por ZN = ', td_zn)

# escrevendo a função de transferência da planta
num = np.array([k])
den = np.array([tau, 1])
H = cnt.tf(num, den)
n_pade = 20
(num_pade, den_pade) = cnt.pade(Theta, n_pade)
H_pade = cnt.tf(num_pade, den_pade)
Hs = cnt.series(H, H_pade)

# Controlador proporcional
numkp = np.array([kp_zn])
denkp = np.array([1])
# integral
numki = np.array([kp_zn])
denki = np.array([ti_zn, 0])

# derivativo
numkd = np.array([kp_zn * td_zn, 0])
denkd = np.array([1])

# Construindo o controlador PID
Hkp = cnt.tf(numkp, denkp)
Hki = cnt.tf(numki, denki)
Hkd = cnt.tf(numkd, denkd)
Hctrl1 = cnt.parallel(Hkp, Hki)
Hctrl = cnt.parallel(Hctrl1, Hkd)
Hdel = cnt.series(Hs, Hctrl)

# Fazendo a realimentação
Hcl = cnt.feedback(Hdel, 1)

t = np.linspace(0, 60, 100)
(t, y) = cnt.step_response(11*Hcl, t)
plt.plot(t, y)
plt.xlabel(' t [ s ] ')
plt.ylabel('Amplitude')
plt.legend(loc='upper right')
plt.title('Controle PID - Método Clássico KN')

plt.grid()
plt.show()
