#Flight of two copters

from re import T
import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

timestr = time.strftime("%Y%m%d-%H%M%S")
fp = open(timestr + '_setpos2.csv', 'w')

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'

MIN_BAT = 3.0

CX = 0
CY = 0
k = 5.0
R = 0.3
v_f = 0.7
D_12 = 2 * math.pi / 2
v_cruis = 0.7
k_f = 3

T_Z = 0.2
v_z = 0.05

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0, 0]
position_estimate_cf2 = [0, 0, 0]

lighthouse_status_cf1 = 0
lighthouse_status_cf2 = 0

bat_cf1 = 0
bat_cf2 = 0

def log_pos_callback_cf1(timestamp, data, logconf):
    global position_estimate_cf1
    position_estimate_cf1[0] = data['kalman.stateX']
    position_estimate_cf1[1] = data['kalman.stateY']
    position_estimate_cf1[2] = data['kalman.stateZ']

    global lighthouse_status_cf1
    global bat_cf1

    lighthouse_status_cf1 = data['lighthouse.status']
    bat_cf1 = data['pm.vbat']


def log_pos_callback_cf2(timestamp, data, logconf):
    global position_estimate_cf2
    position_estimate_cf2[0] = data['kalman.stateX']
    position_estimate_cf2[1] = data['kalman.stateY']
    position_estimate_cf2[2] = data['kalman.stateZ']

    global lighthouse_status_cf2
    global bat_cf2

    lighthouse_status_cf2 = data['lighthouse.status']
    bat_cf2 = data['pm.vbat']


def take_off(cf1, cf2, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print("take_off" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
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


def forward_circle(cf1, cf2):
    global CX
    global CY

    steps = 900
    for i in range(steps):

        # CX += 0.001
        # CY += 0.001

        print("forward_circle" + str(i))
        print(position_estimate_cf1)
        print(position_estimate_cf2)

        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]
        pz_1 = position_estimate_cf1[2]

        px_2 = position_estimate_cf2[0]
        py_2 = position_estimate_cf2[1]
        pz_2 = position_estimate_cf2[2]


        d_1, phi_1 = distance_to_centre(px_1, py_1)
        angle_1 = phase_angle(d_1, phi_1)

        d_2, phi_2 = distance_to_centre(px_2, py_2)
        angle_2 = phase_angle(d_2, phi_2)

        p12 = phase_shift(px_1, py_1, px_2, py_2)

        v1, v2= velocity(p12)

        vx1, vy1 = get_velocity(v1, angle_1)
        vx2, vy2 = get_velocity(v2, angle_2)


        setPx1 = px_1 + vx1/10
        setPx2 = px_2 + vx2/10


        setPy1 = py_1 + vy1/10
        setPy2 = py_2 + vy2/10


        if i == 0:
            init_log(i=i, T_Z=T_Z, v_z=v_z, CX=CX, CY=CY, k=k, R=R, D_12=D_12,
                     v_f=v_f, v_cruis=v_cruis, k_f=k_f, p12=p12, 
                     px_1=px_1, py_1=py_1, pz_1=pz_1, d_1=d_1, phi_1=phi_1, angle_1=angle_1, v1=v1, vx1=vx1, vy1=vy1, setPx1=setPx1, setPy1=setPy1, bat_cf1=bat_cf1, lighthouse_status_cf1 = lighthouse_status_cf1,
                     px_2=px_2, py_2=py_2, pz_2=pz_2, d_2=d_2, phi_2=phi_2, angle_2=angle_2, v2=v2, vx2=vx2, vy2=vy2, setPx2=setPx2, setPy2=setPy2, bat_cf2=bat_cf2, lighthouse_status_cf2 = lighthouse_status_cf2,
                     )

        write_log(i=i, T_Z=T_Z, v_z=v_z, CX=CX, CY=CY, k=k, R=R, D_12=D_12,
                     v_f=v_f, v_cruis=v_cruis, k_f=k_f, p12=p12, 
                     px_1=px_1, py_1=py_1, pz_1=pz_1, d_1=d_1, phi_1=phi_1, angle_1=angle_1, v1=v1, vx1=vx1, vy1=vy1, setPx1=setPx1, setPy1=setPy1, bat_cf1=bat_cf1, lighthouse_status_cf1 = lighthouse_status_cf1,
                     px_2=px_2, py_2=py_2, pz_2=pz_2, d_2=d_2, phi_2=phi_2, angle_2=angle_2, v2=v2, vx2=vx2, vy2=vy2, setPx2=setPx2, setPy2=setPy2, bat_cf2=bat_cf2, lighthouse_status_cf2 = lighthouse_status_cf2,
                     )

        cf1.commander.send_position_setpoint(setPx1, setPy1, T_Z, 0)
        cf2.commander.send_position_setpoint(setPx2, setPy2, T_Z, 0)

        if (bat_cf1 < MIN_BAT) or (bat_cf2 < MIN_BAT):
            print('BATTERY LOW: STOPPING')
            break

        time.sleep(0.1)


def land(cf1, cf2, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print("land" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf1.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def phase_shift(px_a, py_a, px_b, py_b):
    dot_product = (px_a - CX)*(px_b - CX) + (py_a - CY)*(py_b - CY)
    magnitude_i = math.sqrt((px_a - CX)**2 + (py_a - CY)**2)
    magnitude_j = math.sqrt((px_b - CX)**2 + (py_b - CY)**2)
    triple_product = (px_a - CX)*(py_b - CY) - (px_b - CX)*(py_a - CY)
    p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
    if triple_product > 0:
        p_ab = 2*math.pi - p_ab
    return p_ab


def velocity(p_12):
    v1 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (p_12 - D_12))
    v2 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (-p_12 + D_12))
    return (v1, v2)


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
        with SyncCrazyflie(URI2, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf2:
                logconf1 = LogConfig(name='Position', period_in_ms=10)
                logconf1.add_variable('kalman.stateX', 'float')
                logconf1.add_variable('kalman.stateY', 'float')
                logconf1.add_variable('kalman.stateZ', 'float')
                logconf1.add_variable('pm.vbat', 'float')
                logconf1.add_variable('lighthouse.status', 'uint8_t')
                scf1.cf.log.add_config(logconf1)
                logconf1.data_received_cb.add_callback(log_pos_callback_cf1)

                logconf2 = LogConfig(name='Position', period_in_ms=10)
                logconf2.add_variable('kalman.stateX', 'float')
                logconf2.add_variable('kalman.stateY', 'float')
                logconf2.add_variable('kalman.stateZ', 'float')
                logconf2.add_variable('pm.vbat', 'float')
                logconf2.add_variable('lighthouse.status', 'uint8_t')
                scf2.cf.log.add_config(logconf2)
                logconf2.data_received_cb.add_callback(log_pos_callback_cf2)

                logconf1.start()
                logconf2.start()


                cf1 = scf1.cf
                cf2 = scf2.cf

                # cf1.param.set_value('lighthouse.method', 0)
                # cf2.param.set_value('lighthouse.method', 0)

                # ????????????????
                take_off(cf1, cf2, T_Z)

                # ?????????? ???? ??????????
                forward_circle(cf1, cf2)

                # ??????????????
                land(cf1, cf2, 0)

                logconf1.stop()
                logconf2.stop()

    fp.close()
