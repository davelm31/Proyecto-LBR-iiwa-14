import numpy as np
from copy import copy
#import rbdl
cos=np.cos; sin=np.sin; pi=np.pi


#class Robot(object):
#    def __init__(self, q0, dq0, ndof, dt):
#        self.q = q0    # numpy array (ndof x 1)
#        self.dq = dq0  # numpy array (ndof x 1)
#        self.M = np.zeros([ndof, ndof])
#        self.b = np.zeros(ndof)
#        self.dt = dt
#        self.robot = rbdl.loadModel('../Proyecto/src/iiwa_ros/iiwa_description/urdf/model.urdf')
#    def send_command(self, tau):
#        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
#        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
#        ddq = np.linalg.inv(self.M).dot(tau-self.b)
#        self.q = self.q + self.dt*self.dq
#        self.dq = self.dq + self.dt*ddq
#
#    def read_joint_positions(self):
#        return self.q
#
#    def read_joint_velocities(self):
#        return self.dq


#------------------Cinematica directa-------------------------
def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                   [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                   [0, sin(alpha), cos(alpha), d],
                   [0,0,0,1]])
    return T
    
    

def fkine_iiwa(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.34,q[0],0,-pi/2)
    T2 = dh(0,q[1],0,pi/2)
    T3 = dh(0.42,q[2],0,pi/2)
    T4 = dh(0,q[3],0,-pi/2)
    T5 = dh(0.4,q[4],0,-pi/2)
    T6 = dh(0,q[5],0,pi/2)
    T7 = dh(0.179,q[6],-0.3,-pi/2)
    T8 = dh(0,0,-q[7],0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7).dot(T8)
    return T
 
 
#---------------Cinematica inversa---------------------  
#Jacobiano
def jacobian_iiwa14(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,8))
    # Transformacion homogenea inicial (usando q)
    T=fkine_iiwa(q)
    
    # Iteracion para la derivada de cada columna
    for i in range(8):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        Tq = fkine_iiwa(dq)
        
        J[0][i] = (Tq[0][3]-T[0][3])/delta
        J[1][i] = (Tq[1][3]-T[1][3])/delta
        J[2][i] = (Tq[2][3]-T[2][3])/delta

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
    
    return J
def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,8))
    # Implementar este Jacobiano aqui
    T = fkine_iiwa(q)
    R = np.array(T[0:3,0:3])
    Q = TF2xyzquat(T)
    #Qi = np.array(Q)
    for i in range (8):
        # Copiar la configuracion articular inicial (usar este dq para cada
        # incremento en una articulacion)
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+dq)
        Tq = fkine_iiwa(dq)
        Rq = np.array(Tq[0:3,0:3])
        Qq = TF2xyzquat(Tq)
        #Qq = np.array(Qqq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0][i] = (Tq[0][3]-T[0][3])/delta #x
        J[1][i] = (Tq[1][3]-T[1][3])/delta #y
        J[2][i] = (Tq[2][3]-T[2][3])/delta #z
        J[3][i] = (Qq[3]-Q[3])/delta #w
        J[4][i] = (Qq[4]-Q[4])/delta #ex
        J[5][i] = (Qq[5]-Q[5])/delta #ey
        J[6][i] = (Qq[6]-Q[6])/delta #ez

    
    return J
#   
def ikine_iiwa14(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    u_limit = np.array([2.82, 1.99, 2.82, 1.99, 2.82, 1.99, 2.99, 0.3])
    l_limit = np.array([-2.82, -1.99, -2.82, -1.99, -2.82, -1.99, -2.99, -0.001])
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    # archivo temporar para el error
    fe = open("/home/dace/Proyecto/error.txt","w")
    for i in range(max_iter):
        # Main loop
        J = jacobian_iiwa14(q,delta)
        T = fkine_iiwa(q)
        #deseado pos
        F = T[0:3,3]
        e =xdes-F
        qc = q+np.linalg.pinv(J).dot(e)
        
        enorm = np.linalg.norm(e)
        fe.write(str(i)+' '+str(enorm)+'\n')
        for i in range (8):
            if (q[i] < u_limit[i]) and (q[i] > l_limit[i] ):
               q[i] = qc[i]
        if (enorm<epsilon):
          print(enorm)
          break
        if (i==max_iter-1):
          print("no llego al valor deseado")
        
        
        #pass
    fe.close()
    return q

def ik_gradient_iiwa14(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    alpha = 0.3
    u_limit = np.array([2.82, 1.99, 2.82, 1.99, 2.82, 1.99, 2.99, 0.3])
    l_limit = np.array([-2.82, -1.99, -2.82, -1.99, -2.82, -1.99, -2.99, -0.001])

    q  = copy(q0)
    # archivo temporar para el error
    fe = open("/home/dace/Proyecto/error.txt","w")
    for i in range(max_iter):
         # Main loop
        J = jacobian_iiwa14(q,delta)
        T = fkine_iiwa(q)
        #deseado pos
        F = T[0:3,3]
        e =xdes-F
        qc = q+alpha*np.dot(J.T,e)
        
        enorm = np.linalg.norm(e)
        fe.write(str(i)+' '+str(enorm)+'\n')
        for i in range (8):
            if (q[i] < u_limit[i]) and (q[i] > l_limit[i] ):
               q[i] = qc[i]
        if (enorm<epsilon):
          break
        if (i==max_iter-1):
          print("no llego al valor deseado")
        
        #pass
    
    return q
def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion
    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]
    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
