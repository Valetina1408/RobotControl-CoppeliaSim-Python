import sympy as sm
import socket, pickle
from sympy import cos, sin, atan2, pi, acos
import pygame, sys
import sim, time
import math


bandera = 0

def rot_x(ax):
    rotx = sm.Matrix([[1, 0, 0],
                      [0, cos(ax), -sin(ax)],
                      [0, sin(ax), cos(ax)]])
    return rotx

def rot_y(ay):
    roty = sm.Matrix([[cos(ay), 0, sin(ay)],
                      [0, 1, 0],
                      [-sin(ay), 0, cos(ay)]])
    return roty

def rot_z(az):
    rotz = sm.Matrix([[cos(az), -sin(az), 0],
                      [sin(az), cos(az), 0],
                      [0, 0, 1]])
    return rotz


def conectar(port):
    sim.simxFinish(-1)
    dirCliente = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if dirCliente == 0:
        print("Conexión OK", port)
    else:
        print("Conexión fallo")
    return dirCliente


def main():
    start_time = time.time()

    # d1, a2, d4, d6 = 3.63e-1, 0.29, 0.3053, 0.14027
    d1, a2, d4, d6, a1 = 352e-3, 360e-3, 380e-3, 59e-3, 70e-3
    q1, q2, q3, r3, r6, r9 = sm.symbols('q1 q2 q3 r3 r6 r9')

    # qn = sm.Matrix([-0.104625022600471, 0.700283664772426, 0.112696502944828])
    qn = sm.Matrix([(89 * (3.1415 / 180)), (89 * (3.1415 / 180)), 0])
    qs = sm.Matrix([q1, q2, q3])

    pygame.init()
    ventana = pygame.display.set_mode((300, 300))
    dx, dy, dz = 0, 0, 0
    rot1, rot2, rot3 = 0, 0, 0

    # Matriz cinematica directa
    cd = sm.Matrix([cos(q1) * (a1 + a2 * cos(q2)) + d4 * sin(q2 + q3) * cos(q1) + d6 * r3,
                    sin(q1) * (a1 + a2 * cos(q2)) + d4 * sin(q2 + q3) * sin(q1) + d6 * r6,
                    d1 - d4 * cos(q2 + q3) + a2 * sin(q2) + d6 * r9])

    # cd = sm.Matrix([-cos(q1) * (d4 * sin(q2 + q3) - a2 * cos(q2)) + d6 * r3,
    #                -sin(q1) * (d4 * sin(q2 + q3) - a2 * cos(q2)) + d6 * r6,
    #                d1 + d4*cos(q2+q3) + a2*sin(q2) + d6*r9])

    jac = cd.jacobian(qs)
    cliente = conectar(19995)
    mot = []
    for i in range(0, 6, 1):
        ret, m = sim.simxGetObjectHandle(cliente, 'm' + str(i + 1), sim.simx_opmode_blocking)
        mot.append(m)
    print(mot)
    HOST = "192.168.137.1"
    PORT = 65433

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        guardar = 0
        contador = 0

        PuntosAlmacenados1 = sm.Matrix([0, 0, 0])
        PuntosAlmacenados2 = sm.Matrix([0, 0, 0])
        PuntosAlmacenados3 = sm.Matrix([0, 0, 0])
        PuntosRotacion = sm.Matrix([0, 0, 0])
        PuntosRotacion2 = sm.Matrix([0, 0, 0])
        PuntosRotacion3 = sm.Matrix([0, 0, 0])
        PR = sm.Matrix([0, 0, 0])

        t1 = 0
        t2 = 0
        t3 = 0

        MotoresAngulos = sm.Matrix([0, 0, 0])

        while True:
            x = 0
            y = 0
            z = 0

            r11 = 0
            r22 = 0
            r33 = 0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_a:
                        x = x + 15e-3
                    elif event.key == pygame.K_d:
                        x = x - 15e-3
                    elif event.key == pygame.K_w:
                        y = y + 15e-3
                    elif event.key == pygame.K_s:
                        y = y - 15e-3
                    elif event.key == pygame.K_q:
                        z = z + 15e-3
                    elif event.key == pygame.K_e:
                        z = z - 15e-3
                    elif event.key == pygame.K_u:
                        r11 = r11 - 0.08727
                    elif event.key == pygame.K_j:
                        r11 = r11 + 0.08727
                    elif event.key == pygame.K_i:
                        r22 = r22 - 0.08727
                    elif event.key == pygame.K_k:
                        r22 = r22 + 0.08727
                    elif event.key == pygame.K_o:
                        r33 = r33 - 0.08727
                    elif event.key == pygame.K_l:
                        r33 = r33 + 0.08727
                    elif event.key == pygame.K_g:
                        guardar = guardar + 1
                    elif event.key == pygame.K_b:
                        contador = 1
                    elif event.key == pygame.K_n:
                        contador = 2
            dx = dx + x
            dy = dy + y
            dz = dz + z

            rot1 = rot1 + r11
            rot2 = rot2 + r22
            rot3 = rot3 + r33

            rz1 = rot_y(rot1)
            rx1 = rot_x(rot2)
            rz2 = rot_y(rot3)
            rot = rz1 * rx1 * rz2

            if rot[5] > 1.5708:
                rot[5] = 1.5708
            elif rot[5] < -1.5708:
                rot[5] = -1.5708
            if rot[2] > 1.5708:
                rot[2] = 1.5708
            elif rot[2] < -1.5708:
                rot[2] = -1.5708
            if rot[8] > 1.5708:
                rot[8] = 1.5708
            elif rot[8] < -1.5708:
                rot[8] = -1.5708
            if rot[7] > 1.5708:
                rot[7] = 1.5708
            elif rot[7] < -1.5708:
                rot[7] = -1.5708
            if rot[6] > 1.5708:
                rot[6] = 1.5708
            elif rot[6] < -1.5708:
                rot[6] = -1.5708

                
            T4 = atan2(rot[5], rot[2])

            T5 = acos(rot[8])

            T6 = -atan2(rot[7], rot[6])

            if math.isnan(T4):
                T4 = 0

            if math.isnan(T5):
                T5 = 0

            if math.isnan(T6):
                T6 = 0


            # posd = sm.Matrix([1.93770574585805e-9 + dx, -2.03475482990489e-10 + dy, 0.899999988389148 + dz])
            posd = sm.Matrix([dx, 0.515 + dy, 0.712 + dz])  # El 0.01 es el desfase chiquito que dijo Oscar
            cdeval = cd.subs([(q1, qn[0]), (q2, qn[1]), (q3, qn[2]),
                              (r3, rot[2]), (r6, rot[5]), (r9, rot[8])]).evalf()
            print(cdeval)
            error = posd - cdeval
            niter = 0
            e = 1e-6
            qa = qn

            while True:
                if abs(error[0]) < e and abs(error[1]) < e and abs(error[2]) < e or niter > 50:
                    if niter < 50:
                        qn = qa.evalf()
                    break
                j = jac.subs([(q1, qa[0]), (q2, qa[1]), (q3, qa[2]),
                              (r3, rot[2]), (r6, rot[5]), (r9, rot[8])]).evalf()
                jinv = j.pinv()
                qa = (qa + jinv * error).evalf()
                if qa[0] < 0:
                    qa[0] = qa[0] % -3.1416
                else:
                    qa[0] = qa[0] % 3.1416

                if qa[1] < 0:
                    qa[1] = qa[1] % -3.1416
                else:
                    qa[1] = qa[1] % 3.1416

                if qa[2] < 0:
                    qa[2] = qa[2] % -3.1416
                else:
                    qa[2] = qa[2] % 3.1416

                cdeval = cd.subs([(q1, qa[0]), (q2, qa[1]), (q3, qa[2]),
                                  (r3, rot[2]), (r6, rot[5]), (r9, rot[8])]).evalf()

                error = posd - cdeval
                niter = niter + 1
            # print(qn)
            MotoresAngulos = qn
            A = []
            B = []
            C = []

            if guardar == 1:
                PuntosAlmacenados1[0] = MotoresAngulos[0]
                PuntosAlmacenados1[1] = MotoresAngulos[1]
                PuntosAlmacenados1[2] = MotoresAngulos[2]
                PuntosRotacion[0] = T4
                PuntosRotacion[1] = T5
                PuntosRotacion[2] = T6
                t1 = (time.time() - start_time)

                guardar = 2
            else:
                PuntosAlmacenados1[0] = PuntosAlmacenados1[0]
                PuntosAlmacenados1[1] = PuntosAlmacenados1[1]
                PuntosAlmacenados1[2] = PuntosAlmacenados1[2]
                PuntosRotacion[0] = PuntosRotacion[0]
                PuntosRotacion[1] = PuntosRotacion[1]
                PuntosRotacion[2] = PuntosRotacion[2]
            if guardar == 3:
                PuntosAlmacenados2[0] = MotoresAngulos[0]
                PuntosAlmacenados2[1] = MotoresAngulos[1]
                PuntosAlmacenados2[2] = MotoresAngulos[2]
                PuntosRotacion2[0] = T4
                PuntosRotacion2[1] = T5
                PuntosRotacion2[2] = T6
                t2 = (time.time() - start_time)

                guardar = 4
            else:
                PuntosAlmacenados2[0] = PuntosAlmacenados2[0]
                PuntosAlmacenados2[1] = PuntosAlmacenados2[1]
                PuntosAlmacenados2[2] = PuntosAlmacenados2[2]

                PuntosRotacion2[0] = PuntosRotacion2[0]
                PuntosRotacion2[1] = PuntosRotacion2[1]
                PuntosRotacion2[2] = PuntosRotacion2[2]

            if guardar == 5:
                PuntosAlmacenados3[0] = MotoresAngulos[0]
                PuntosAlmacenados3[1] = MotoresAngulos[1]
                PuntosAlmacenados3[2] = MotoresAngulos[2]
                PuntosRotacion3[0] = T4
                PuntosRotacion3[1] = T5
                PuntosRotacion3[2] = T6
                t3 = (time.time() - start_time)

                guardar = 0
            else:
                PuntosAlmacenados3[0] = PuntosAlmacenados3[0]
                PuntosAlmacenados3[1] = PuntosAlmacenados3[1]
                PuntosAlmacenados3[2] = PuntosAlmacenados3[2]
                PuntosRotacion3[0] = PuntosRotacion3[0]
                PuntosRotacion3[1] = PuntosRotacion3[1]
                PuntosRotacion3[2] = PuntosRotacion3[2]
            # print("------------------------------------------")
            print(PuntosAlmacenados1,'P1')
            print(t1)
            print(PuntosAlmacenados2,'P2')
            print(t2)
            print(PuntosAlmacenados3,'P3')
            print(t3)

            if t3 == 0:
                bandera = 0
            if t3 != 0:
                bandera = 1

            if bandera == 0:
                DATOS = [bandera, qn[0], qn[1], qn[2], T4, T5, T6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            if bandera == 1:
                if contador==1:
                    DATOS = [bandera, PuntosAlmacenados1[0], PuntosAlmacenados1[1], PuntosAlmacenados1[2], t1,
                             PuntosAlmacenados2[0], PuntosAlmacenados2[1], PuntosAlmacenados2[2], t2,
                             PuntosAlmacenados3[0], PuntosAlmacenados3[1], PuntosAlmacenados3[2], t3,
                             PuntosRotacion[0], PuntosRotacion[1], PuntosRotacion[2], PuntosRotacion2[0], PuntosRotacion2[1], PuntosRotacion2[2],
                             PuntosRotacion3[0], PuntosRotacion3[1], PuntosRotacion3[2]]
            if contador==2:
                # PuntosAlmacenados1[0] = 1.42117899528066
                # PuntosAlmacenados1[1] = 1.26212195104690
                # PuntosAlmacenados1[2] = -0.44693917446773
                PuntosAlmacenados1[0] = 1.48928936644763
                PuntosAlmacenados1[1] = 0.812737021905390
                PuntosAlmacenados1[2] = -0.339469582059367
                PuntosRotacion[0] = -0.565062658704968 + 3.1415
                PuntosRotacion[1] = -0.504499752344792
                PuntosRotacion[2] = -0.506701081057118
                t1 = 0.0

                # PuntosAlmacenados2[0] = 1.86254476056805
                # PuntosAlmacenados2[1] = 1.04822932641427
                # PuntosAlmacenados2[2] = -0.308514455247146
                PuntosAlmacenados2[0] = 1.999747490863954
                PuntosAlmacenados2[1] = 0.767784000264070
                PuntosAlmacenados2[2] = -0.3299891534761806

                PuntosRotacion2[0] = 0.354025967627701 - 3.14
                PuntosRotacion2[1] = 1.4694140874823
                PuntosRotacion2[2] = -0.0602168741768234
                t2 = 12.0

                # PuntosAlmacenados3[0] = 1.61611170299229
                # PuntosAlmacenados3[1] = 1.44331875962345
                # PuntosAlmacenados3[2] = -0.0784787285254158
                PuntosAlmacenados3[0] = 1.90977813881032
                PuntosAlmacenados3[1] = 1.67298128272798
                PuntosAlmacenados3[2] = -0.234632536395920
                PuntosRotacion3[0] = -3.1415
                PuntosRotacion3[1] = -1.57086000000000
                PuntosRotacion3[2] = 0.0

                t3 = 20.0

                DATOS = [bandera, PuntosAlmacenados1[0], PuntosAlmacenados1[1], PuntosAlmacenados1[2], t1,
                         PuntosAlmacenados2[0], PuntosAlmacenados2[1], PuntosAlmacenados2[2], t2,
                         PuntosAlmacenados3[0], PuntosAlmacenados3[1], PuntosAlmacenados3[2], t3,
                         PuntosRotacion[0], PuntosRotacion[1], PuntosRotacion[2], PuntosRotacion2[0],
                         PuntosRotacion2[1], PuntosRotacion2[2], PuntosRotacion3[0], PuntosRotacion3[1], PuntosRotacion3[2]]

            print(DATOS,'Datos')
            todos = DATOS
            g = pickle.dumps(todos)
            s.sendall(g)
            data = s.recv(1024)
            decom = pickle.loads(data)
"""
        ret = sim.simxSetJointTargetPosition(cliente, mot[0], qn[0] - 1.56,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[1], qn[1] - 1.56,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[2], qn[2],
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[3], T4,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[4], T5,
                                             sim.simx_opmode_oneshot)
        ret = sim.simxSetJointTargetPosition(cliente, mot[5], T6,
                                             sim.simx_opmode_oneshot)

"""
if __name__ == "__main__":
    main()


