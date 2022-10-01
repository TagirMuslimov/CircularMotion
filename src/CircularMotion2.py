import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E703'

CX = 0.5
CY = 0.0
k = 5.0
R = 0.5
v_f = 0.05
D_12 = 2 * math.pi / 2
v_cruis = 0.05
k_f = 0.1

T_Z = 0.3
v_z = 0.05

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0, 0]
position_estimate_cf2 = [0, 0, 0]

def log_pos_callback_cf1(timestamp, data, logconf):
    global position_estimate_cf1
    position_estimate_cf1[0] = data['stateEstimate.x']
    position_estimate_cf1[1] = data['stateEstimate.y']
    position_estimate_cf1[2] = data['stateEstimate.z']

def log_pos_callback_cf2(timestamp, data, logconf):
    global position_estimate_cf2
    position_estimate_cf2[0] = data['stateEstimate.x']
    position_estimate_cf2[1] = data['stateEstimate.y']
    position_estimate_cf2[2] = data['stateEstimate.z']

def take_off(cf1, cf2, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print ("take_off" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def forward(cf, distance):
    sleep_time = 0.1
    vx = 0.1
    steps = int(distance / vx / sleep_time)

    for i in range(steps):
        print ("forward" + str(i))
        cf.commander.send_velocity_world_setpoint(vx, 0, 0, 0)
        time.sleep(sleep_time)

def forward_circle(cf1, cf2):
    fp = open('log.csv', 'w')
    fp.write('i; TZ; vz; CX; CY; k; R; D_12; v_f; v_cruis; k_f; p12;')
    fp.write('px1; py1; pz1; d1; phi1; angle1; v1; vx1; vy1; setPx1;setPy1;setPz1;')
    fp.write('px2; py2; pz2; d2; phi2; angle2; v2; vx2; vy2; setPx2;setPy2;setPz2')
    fp.write('\n')
    steps = 20000
    for i in range (steps):

        print ("forward_circle" + str(i))
        print(position_estimate_cf1)
        print(position_estimate_cf2)
        
        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]
        pz_1 = position_estimate_cf1[2]

        px_2 = position_estimate_cf2[0]
        py_2 = position_estimate_cf2[1]
        pz_2 = position_estimate_cf2[2]

        d_1, phi_1 = distance_to_centre (px_1, py_1)
        angle_1 = phase_angle (d_1, phi_1)

        d_2, phi_2 = distance_to_centre (px_2, py_2)
        angle_2 = phase_angle (d_2, phi_2)

        p12 = phase_shift(px_1, py_1, px_2, py_2)

        v1, v2 = velocity(p12)

        vx1, vy1 = get_velocity(v1, angle_1)
        vx2, vy2 = get_velocity(v2, angle_2)

        setPx1 = px_1 + vx1
        setPx2 = px_2 + vx2

        setPy1 = py_1 + vy1
        setPy2 = py_2 + vy2

        setPz1 = T_Z
        setPz2 = T_Z
        
        fp.write(
            str(i) + ';' +
            str(T_Z) + ';' +
            str(v_z) + ';' +
            str(CX) + ';' +
            str(CY) + ';' +
            str(k) + ';' +
            str(R) + ';' +
            str(D_12) + ';' +
            str(v_f) + ';' +
            str(v_cruis) + ';' +
            str(k_f) + ';' +
            str(p12) + ';' +

            str(px_1) + ';' +
            str(py_1) + ';' +
            str(pz_1) + ';' +
            str(d_1) + ';' +
            str(phi_1) + ';' +
            str(angle_1) + ';' +
            str(v1) + ';' +
            str(vx1) + ';' +
            str(vy1) + ';' +
            str(setPx1) + ';' +
            str(setPy1) + ';' +
            str(setPz1) + ';' +

            str(px_2) + ';' +
            str(py_2) + ';' +
            str(pz_2) + ';' +
            str(d_2) + ';' +
            str(phi_2) + ';' +
            str(angle_2) + ';' +
            str(v2) + ';' +
            str(vx2) + ';' +
            str(vy2) + ';' +
            str(setPx2) + ';' +
            str(setPy2) + ';' +
            str(setPz2) + ';' +

            '\n'
        )

        cf1.commander.send_position_setpoint(setPx1, setPy1, setPz1, 0)
        cf2.commander.send_position_setpoint(setPx2, setPy2, setPz2, 0)
    
    fp.close()

def land(cf1, cf2, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print ("land" + str(i))
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
    triple_product = (px_a - CX)*(py_b - CY) + (px_b - CX)*(py_a - CY)
    p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
    if triple_product > 0:
        p_ab = 2*math.pi - p_ab
    return p_ab

def velocity(p_12):
    v1 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (p_12 - D_12))
    v2 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (-p_12 + D_12))
    return (v1, v2)

def distance_to_centre (px, py):
    d = math.sqrt((px - CX)**2 + (py - CY)**2)
    phi = math.atan2(px - CX, py - CY)
    print('d= ' + str(d))
    print('phi= ' + str(phi))
    return (d, phi)

def phase_angle (d, phi):
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
            logconf1.add_variable('stateEstimate.x', 'float')
            logconf1.add_variable('stateEstimate.y', 'float')
            logconf1.add_variable('stateEstimate.z', 'float')
            scf1.cf.log.add_config(logconf1)
            logconf1.data_received_cb.add_callback(log_pos_callback_cf1)

            logconf2 = LogConfig(name='Position', period_in_ms=10)
            logconf2.add_variable('stateEstimate.x', 'float')
            logconf2.add_variable('stateEstimate.y', 'float')
            logconf2.add_variable('stateEstimate.z', 'float')
            scf2.cf.log.add_config(logconf2)
            logconf2.data_received_cb.add_callback(log_pos_callback_cf2)

            logconf1.start()
            logconf2.start()

            cf1=scf1.cf
            cf2=scf2.cf

            # взлетаем
            take_off(cf1, cf2, T_Z)

            # летим по кругу
            forward_circle(cf1, cf2)

            # садимся
            land(cf1, cf2, 0)

            logconf1.stop()
            logconf2.stop()
