from re import T
import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

timestr = time.strftime("%Y%m%d-%H%M%S")
fp = open(timestr + '_setpos1.csv', 'w')

URI1 = 'radio://0/80/2M/E7E7E7E703'

CX = 0.5
CY = 0.0
k = 1.0
R = 0.3
v_f = 0.05
v_cruis = 0.3
k_f = 0.1

T_Z = 0.2
v_z = 0.05

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0, 0]

def log_pos_callback_cf1(timestamp, data, logconf):
    global position_estimate_cf1
    position_estimate_cf1[0] = data['kalman.stateX']
    position_estimate_cf1[1] = data['kalman.stateY']
    position_estimate_cf1[2] = data['kalman.stateZ']



def take_off(cf1, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print("take_off" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def forward(cf, distance):
    sleep_time = 0.1
    vx = 0.1
    steps = int(distance / vx / sleep_time)

    for i in range(steps):
        print("forward" + str(i))
        cf.commander.send_velocity_world_setpoint(vx, 0, 0, 0)
        time.sleep(sleep_time)


def init_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(k) + ';')
    fp.write('\n')


def write_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(v) + ';')
    fp.write('\n')


def forward_circle(cf1):
    steps = 20000
    for i in range(steps):

        print("forward_circle" + str(i))
        print(position_estimate_cf1)

        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]
        pz_1 = position_estimate_cf1[2]


        d_1, phi_1 = distance_to_centre(px_1, py_1)
        angle_1 = phase_angle(d_1, phi_1)

        v1 = v_cruis

        vx1, vy1 = get_velocity(v1, angle_1)

        setPx1 = px_1 + vx1/5
        setPy1 = py_1 + vy1/5

        if i == 0:
            init_log(i=i, T_Z=T_Z, v_z=v_z, CX=CX, CY=CY, k=k, R=R,
                     v_f=v_f, v_cruis=v_cruis, k_f=k_f,
                     px_1=px_1, py_1=py_1, pz_1=pz_1, d_1=d_1, phi_1=phi_1, angle_1=angle_1, v1=v1, vx1=vx1, vy1=vy1, setPx1=setPx1, setPy1=setPy1,
                     )

        write_log(i=i, T_Z=T_Z, v_z=v_z, CX=CX, CY=CY, k=k, R=R,
                     v_f=v_f, v_cruis=v_cruis, k_f=k_f,
                     px_1=px_1, py_1=py_1, pz_1=pz_1, d_1=d_1, phi_1=phi_1, angle_1=angle_1, v1=v1, vx1=vx1, vy1=vy1, setPx1=setPx1, setPy1=setPy1,
                     )

        cf1.commander.send_position_setpoint(setPx1, setPy1, T_Z, 0)

        time.sleep(0.1)


def land(cf1, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print("land" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf1.commander.send_stop_setpoint()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def phase_shift(px_a, py_a, px_b, py_b):
    dot_product = (px_a - CX)*(px_b - CX) + (py_a - CY)*(py_b - CY)
    magnitude_i = math.sqrt((px_a - CX)**2 + (py_a - CY)**2)
    magnitude_j = math.sqrt((px_b - CX)**2 + (py_b - CY)**2)
    triple_product = (px_a - CX)*(py_b - CY) + (px_b - CX)*(py_a - CY)
    p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
    if triple_product > 0:
        p_ab = 2*math.pi - p_ab
    return p_ab



def distance_to_centre(px, py):
    d = math.sqrt((px - CX)**2 + (py - CY)**2)
    phi = math.atan2(px - CX, py - CY)
    print('d= ' + str(d))
    print('phi= ' + str(phi))
    return (d, phi)


def phase_angle(d, phi):
    angle = phi + math.pi/2 + math.atan(k * (d - R))
    print('angle= ' + str(angle))
    return angle


def get_velocity(v, angle):
    vx = v * math.sin(angle)
    vy = v * math.cos(angle)
    print('vx= ' + str(vx))
    print('vy= ' + str(vy))
    return (vx.real, vy.real)


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI1, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf1:
                logconf1 = LogConfig(name='Position', period_in_ms=10)
                logconf1.add_variable('kalman.stateX', 'float')
                logconf1.add_variable('kalman.stateY', 'float')
                logconf1.add_variable('kalman.stateZ', 'float')
                scf1.cf.log.add_config(logconf1)
                logconf1.data_received_cb.add_callback(log_pos_callback_cf1)

                logconf1.start()


                cf1 = scf1.cf


                # взлетаем
                take_off(cf1,  T_Z)

                # летим по кругу
                forward_circle(cf1)

                # садимся
                land(cf1,  0)

                logconf1.stop()

    fp.close()
