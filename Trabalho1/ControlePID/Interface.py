
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# considerando uma função de transferencia em malha aberta FT=k/(tau*s+1)
k = 3.18892999
tau = 11.85
Theta = 4.04999999

def KN ():
    kp_zn = (1.2 * tau) / (k * Theta)
    ti_zn = 2 * Theta
    td_zn = 0.5 * Theta

    print('Kp calculado por ZN = ', kp_zn)
    print('Ti calculado por ZN = ', ti_zn)
    print('Td calculado por ZN = ', td_zn)
    print('')

def CC():
    kp_cc = (1 / k) * (tau / Theta) * ((4 / 3) + ((1 / 4) * (Theta / tau)))
    ti_cc = Theta * ((32 + (6 * (Theta / tau))) / (13 + (8 * (Theta / tau))))
    td_cc = Theta * (4 / (11 + (2 * (Theta / tau))))

    print('Kp calculado por CC = ', kp_cc)
    print('Ti calculado por CC = ', ti_cc)
    print('Td calculado por CC = ', td_cc)
    print('')

def entradaPID():
    print('Insira agora os dados dos parâmetros PID')
    kp = float(input('Kp = '))
    ti = float(input('Ti = '))
    td = float(input('Td = '))
    sp = float(input('SetPoint = '))
    print('')

    # escrevendo a função de transferência da planta
    num = np.array([k])
    den = np.array([tau, 1])
    H = cnt.tf(num, den)
    n_pade = 20
    (num_pade, den_pade) = cnt.pade(Theta, n_pade)
    H_pade = cnt.tf(num_pade, den_pade)
    Hs = cnt.series(H, H_pade)

    # Controlador proporcional
    numkp = np.array([kp])
    denkp = np.array([1])
    # integral
    numki = np.array([kp])
    denki = np.array([ti, 0])

    # derivativo
    numkd = np.array([kp * td, 0])
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
    (t, y) = cnt.step_response(sp * Hcl, t)
    plt.plot(t, y)
    plt.xlabel(' t [ s ] ')
    plt.ylabel('Amplitude')
    plt.title('Controle PID')

    plt.grid()
    plt.show()

while True:
    print('Menu de Opções: ')
    print('[1] - Visualizar parâmetros do PID obtidos pelo método de KN.')
    print('[2] - Visualizar parâmetros do PID obtidos pelo método de CC.')
    print('[3] - Entrar com os parâmetros do PID manualmente.')
    print('[4] - Sair.')

    escolha = int(input('Selecione a opção desejada: '))
    print('')

    if(escolha == 1):
        KN()
    elif(escolha == 2):
        CC()
    elif(escolha == 3):
        entradaPID()
    elif(escolha == 4):
        break
    else:
        print('Opção Inválida, escolha novamente!')
