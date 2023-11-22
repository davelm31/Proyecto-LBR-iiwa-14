import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/modelo23.urdf')
# Grados de libertad
ndof = modelo.q_size


# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6,0, 0])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0, 0, 0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0, 0])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(8)               # Vector identidad
m     =np.zeros(ndof)
b     = np.zeros(ndof)          
# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
#print(tau)

# Calculo de vector gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
#print(g)

# Calculo de vector Coriolis
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c-g
print (c)
rbdl.NonlinearEffects(modelo, q, dq, b)
print(b)

for i in range(ndof):
   rbdl.InverseDynamics(modelo,q,zeros,e[i],m)
   M[:,i] = m-g
#print(M)

tau_exp = M.dot(ddq)+c+g
#print(tau_exp)

