import sympy as sm
import numpy as np
from numpy.linalg import inv
from sympy import cos, sin, atan2, pi, acos
import pygame, sys
import sim
import time
import threading
import math
import socket, pickle

b = 0
c = 0
start_time = time.time()
tact=0

coeff1=0
coeff2=0
coeff3=0
cliente=0
mot=[]
desfase=[]
rot1=0
rot2=0
rot3=0

def conectar(port):
    sim.simxFinish(-1)
    dirCliente = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if dirCliente == 0:
        print("Conexión OK", port)
    else:
        print("Conexión fallo")
    return dirCliente

HOST = ""
PORT = 65433

def hilo(cliente, mot, desfase, coeff1, coeff2, coeff3, rot1,rot2,rot3, b, c):
    b = b+0.05
    #print(b)
    #print(c)
    if(b<=c):
        ang1 = coeff1[0, 0] + coeff1[0, 1] * b + coeff1[0, 2] * (b ** 2) + coeff1[0, 3] * (b ** 3) + coeff1[0, 4] * (
                    b ** 4)
        ang2 = coeff2[0, 0] + coeff2[0, 1] * b + coeff2[0, 2] * (b ** 2) + coeff2[0, 3] * (b ** 3) + coeff2[0, 4] * (
                    b ** 4)
        ang3 = coeff3[0, 0] + coeff3[0, 1] * b + coeff3[0, 2] * (b ** 2) + coeff3[0, 3] * (b ** 3) + coeff3[0, 4] * (
                    b ** 4)

        if ang1 < 0:
            ang1 = ang1 % -3.1416
        else:
            ang1 = ang1 % 3.1416

        if ang2 < 0:
            ang2 = ang2 % -3.1416
        else:
            ang2 = ang2 % 3.1416

        if ang3 < 0:
            ang3 = ang3 % -3.1416
        else:
            ang3 = ang3 % 3.1416

        print('--------------------------------')#&
        print([ang1,  ang2, ang3,rot1,rot2,rot3])
        print(b, 'tiempo')#
        ret = sim.simxSetJointTargetPosition(cliente, mot[0], ang1 - 1.56,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[1], ang2 - 1.56,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[2], ang3,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[3], rot1,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[3], rot2,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[3], rot3,
                                             sim.simx_opmode_oneshot)
    threading.Timer(0.1, hilo, args=(cliente, mot, desfase, coeff1, coeff2, coeff3,rot2,rot2,rot3, b, c,)).start()
    if abs(b-c)<0.05:
        threading.Timer(0.1, hilo, args=(cliente, mot, desfase, coeff1, coeff2, coeff3,rot1,rot2,rot3, b, c,)).cancel()
threading.Timer(0.1, hilo, args=(cliente, mot, desfase, coeff1, coeff2, coeff3,rot1,rot2,rot3, b, c,)).start()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    con, addr = s.accept()
    todos = [637892.4395, -536.289, 348059.902394]
    motores = [0,0,0,0,0,0]
    t1 = 0
    t2 = 0
    t3 = 0
    PuntosAlmacenados1 = [0, 0, 0]
    PuntosAlmacenados2 = [0, 0, 0]
    PuntosAlmacenados3 = [0, 0, 0]
    PuntosRotacion = sm.Matrix([0, 0, 0])
    PuntosRotacion2 = sm.Matrix([0, 0, 0])
    PuntosRotacion3 = sm.Matrix([0, 0, 0])
    PR = sm.Matrix([0, 0, 0])
    cliente = conectar(19995)
    mot = []
    for i in range(0, 6, 1):
        ret, m = sim.simxGetObjectHandle(cliente, 'm' + str(i + 1), sim.simx_opmode_blocking)
        mot.append(m)
    print(mot)
    with con:
        print(addr)
        while True:
            comp = pickle.dumps(todos)
            con.sendall(comp)
            data =con.recv(1024)
            desco=pickle.loads(data)

            print(desco)

            print(desco[1])
            if desco[0] == 0:
                 motores[0] = desco[1]
                 motores[1] = desco[2]
                 motores[2] = desco[3]
                 motores[3] = desco[4]
                 motores[4] = desco[5]
                 motores[5] = desco[6]

            if desco[0] == 1:
                PuntosAlmacenados1 = [desco[1], desco[2], desco[3]]
                t1 = desco[4]
                PuntosAlmacenados2 = [desco[5], desco[6], desco[7]]
                t2 = desco[8]
                PuntosAlmacenados3 = [desco[9], desco[10], desco[11]]
                t3 = desco[12]
                PuntosRotacion = [desco[13], desco[14], desco[15]]
                PuntosRotacion2 = [desco[16], desco[17], desco[18]]
                PuntosRotacion3 = [desco[19], desco[20], desco[21]]
                tact=time.time() - start_time

                if ( tact >= t1 and tact<t2):
                    motores[3] = PuntosRotacion[0]
                    motores[4] = PuntosRotacion[1]
                    motores[5] = PuntosRotacion[2]
                elif ( tact >= t2 and tact<t3):
                    motores[3] = PuntosRotacion2[0]
                    motores[4] = PuntosRotacion2[1]
                    motores[5] = PuntosRotacion2[2]
                elif tact >= t3:
                    motores[3] = PuntosRotacion3[0]
                    motores[4] = PuntosRotacion3[1]
                    motores[5] = PuntosRotacion3[2]


            ret = sim.simxSetJointTargetPosition(cliente, mot[0], motores[0] - 1.56, sim.simx_opmode_oneshot)
            ret = sim.simxSetJointTargetPosition(cliente, mot[1], motores[1] - 1.56, sim.simx_opmode_oneshot)
            ret = sim.simxSetJointTargetPosition(cliente, mot[2], motores[2],
                                                         sim.simx_opmode_oneshot)
            ret = sim.simxSetJointTargetPosition(cliente, mot[3], motores[3],
                                                         sim.simx_opmode_oneshot)
            ret = sim.simxSetJointTargetPosition(cliente, mot[4], motores[4],
                                                         sim.simx_opmode_oneshot)
            ret = sim.simxSetJointTargetPosition(cliente, mot[5], motores[5],
                                                         sim.simx_opmode_oneshot)
            print(t3)
            if t3 != 0:
                Axyz = np.matrix([[1, t1, t1 ** 2, t1 ** 3, t1 ** 4],
                                  [1, t2, t2 ** 2, t2 ** 3, t2 ** 4],
                                  [1, t3, t3 ** 2, t3 ** 3, t3 ** 4],
                                  [0, 1, 2 * t1, 3 * t1 ** 2, 4 * t1 ** 3],
                                  [0, 1, 2 * t3, 3 * t3 ** 2, 4 * t3 * 3]])

                Bx = [PuntosAlmacenados1[0],
                      PuntosAlmacenados2[0],
                      PuntosAlmacenados3[0],
                      0,
                      0]
                Cx = np.matmul(np.linalg.pinv(Axyz), Bx)
#
                By = [PuntosAlmacenados1[1],
                      PuntosAlmacenados2[1],
                      PuntosAlmacenados3[1],
                      0,
                      0]
                Cy = np.matmul(np.linalg.pinv(Axyz), By)

                Bz = [PuntosAlmacenados1[2],
                      PuntosAlmacenados2[2],
                      PuntosAlmacenados3[2],
                      0,
                      0]
                Cz = np.matmul(np.linalg.pinv(Axyz), Bz)
                print(Cx)
                print(Cy)
                print(Cz)

                PR[0] = motores[3]
                PR[1] = motores[4]
                PR[2] = motores[5]

                time.sleep(2)
                hilo(cliente, mot, desfase, Cx, Cy, Cz, PR[0], PR[1], PR[2], t1, t3,)
                break

if __name__ == "__main__":
    main()
