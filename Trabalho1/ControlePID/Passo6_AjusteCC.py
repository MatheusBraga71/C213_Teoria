import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# considerando uma função de transferencia em malha aberta FT=k/(tau*s+1)
k = 3.18892999
tau = 11.85
Theta = 4.04999999

# Fazendo os cálculos de Kp, Ti, Td usando o método CC

kp_cc = (1/k)*(tau/Theta)*((4/3)+((1/4)*(Theta/tau)))

ti_cc = Theta*((32+(6*(Theta/tau)))/(13+(8*(Theta/tau))))

td_cc = Theta*(4/(11+(2*(Theta/tau))))

print('Kp calculado por CC = ', kp_cc)
print('Ti calculado por CC = ', ti_cc)
print('Td calculado por CC = ', td_cc)

# escrevendo a função de transferência da planta
num_cc = np.array([k])
den_cc = np.array([tau, 1])
H_cc = cnt.tf(num_cc, den_cc)
n_pade = 20
(num_pade, den_pade) = cnt.pade(Theta, n_pade)
H_pade = cnt.tf(num_pade, den_pade)
Hs_cc = cnt.series(H_cc, H_pade)

novo_kp = kp_cc - 0.6
novo_ti = ti_cc
novo_td = td_cc

print("Novo Kp = ", novo_kp)

# Controlador proporcional
numkp_cc = np.array([novo_kp])
denkp_cc = np.array([1])
# integral
numki_cc = np.array([novo_kp])
denki_cc = np.array([novo_ti, 0])

# derivativo
numkd_cc = np.array([novo_kp * novo_td, 0])
denkd_cc = np.array([1])

# Construindo o controlador PID
Hkp_cc = cnt.tf(numkp_cc, denkp_cc)
Hki_cc = cnt.tf(numki_cc, denki_cc)
Hkd_cc = cnt.tf(numkd_cc, denkd_cc)
Hctrl1 = cnt.parallel(Hkp_cc, Hki_cc)
Hctrl_cc = cnt.parallel(Hctrl1, Hkd_cc)
Hdel_cc = cnt.series(Hs_cc, Hctrl_cc)

# Fazendo a realimentação
Hcl_cc = cnt.feedback(Hdel_cc, 1)

t = np.linspace(0, 60, 100)
(t, y) = cnt.step_response(11*Hcl_cc, t)
plt.plot(t, y)
plt.xlabel(' t [ s ] ')
plt.ylabel('Amplitude')
# plt.legend(loc='upper right')
plt.title('Ajuste Kp calculado - 0.6')

plt.grid()
plt.show()
