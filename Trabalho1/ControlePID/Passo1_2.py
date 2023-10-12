from scipy.io import loadmat

mat = loadmat('TransferFunction11.mat')
degrau = mat.get('degrau')
saida = mat.get('saida')
t1 = mat.get('t')

print('Calculando os valores de k, t1, t2 usando o método de smith')
deltaY = max(saida) - min(saida)
print('Calculo DeltaY = ', deltaY)

deltaU = degrau[1]
print('DeltaU = ', deltaU)

yTempo1 = max(saida) * 0.283
tempo1 = 8
print('Y(t1) = ', yTempo1, ' | t1 = ', tempo1)

yTempo2 = max(saida) * 0.632
tempo2 = 15.9
print('Y(t2) = ', yTempo2, ' | t2 = ', tempo2)

k = deltaY/deltaU
print('K = ', k)

tau = 1.5*(tempo2 - tempo1)
print('Tau =', tau)

Theta = tempo2 - tau
print('Theta = ', Theta)
