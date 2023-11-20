import sympy as sm
from sympy import cos, sin, atan2, pi, acos
import pygame, sys
import sim, time
import math


def rot_x(ax):
    rotx = sm.Matrix([[1, 0, 0],
                      [0, cos(ax), -sin(ax)],
                      [0, sin(ax), cos(ax)]])
    return rotx


def rot_y(ay):
    roty = sm.Matrix([[1, 0, 0],
                      [0, cos(ay), -sin(ay)],
                      [0, sin(ay), cos(ay)]])
    return roty


def rot_z(az):
    rotz = sm.Matrix([[1, 0, 0],
                      [0, cos(az), -sin(az)],
                      [0, sin(az), cos(az)]])
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
    # d1, a2, d4, d6 = 3.63e-1, 0.29, 0.3053, 0.14027
    d1, a2, d4, d6, a1 = 352e-3, 360e-3, 380e-3, 59e-3, 70e-3
    q1, q2, q3, r3, r6, r9 = sm.symbols('q1 q2 q3 r3 r6 r9')
    # qn = sm.Matrix([-0.104625022600471, 0.700283664772426, 0.112696502944828])
    qn = sm.Matrix([(89 * (3.1415 / 180)), (89 * (3.1415 / 180)), 0])
    qs = sm.Matrix([q1, q2, q3])

    pygame.init()
    ventana = pygame.display.set_mode((300, 300))
    dx, dy, dz = 0, 0, 0

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
                    x = x + 5e-3
                elif event.key == pygame.K_d:
                    x = x - 5e-3
                elif event.key == pygame.K_w:
                    y = y + 5e-3
                elif event.key == pygame.K_s:
                    y = y - 5e-3
                elif event.key == pygame.K_q:
                    z = z + 5e-3
                elif event.key == pygame.K_e:
                    z = z - 5e-3
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
        dx = dx + x
        dy = dy + y
        dz = dz + z

        rz1 = rot_z(r11)
        rx1 = rot_x(r22)
        rz2 = rot_y(r33)
        rot = rz1 * rx1 * rz2

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


if __name__ == "__main__":
    main()


