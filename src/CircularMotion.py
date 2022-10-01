import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
URI3 = 'radio://0/80/2M/E7E7E7E703'

CX = 0.5
CY = 0.0
k = 5.0
R = 0.5
v_f = 0.05
D_12 = 2 * math.pi / 3
D_23 = 2 * math.pi / 3
v_cruis = 0.1
k_f = 1

T_Z = 0.5
v_z = 0.1

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0, 0]
position_estimate_cf2 = [0, 0, 0]
position_estimate_cf3 = [0, 0, 0]

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

def log_pos_callback_cf3(timestamp, data, logconf):
    global position_estimate_cf3
    position_estimate_cf3[0] = data['stateEstimate.x']
    position_estimate_cf3[1] = data['stateEstimate.y']
    position_estimate_cf3[2] = data['stateEstimate.z']

def take_off(cf1, cf2, cf3, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print ("take_off" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf3.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def forward(cf, distance):
    sleep_time = 0.1
    vx = 0.1
    steps = int(distance / vx / sleep_time)

    for i in range(steps):
        print ("forward" + str(i))
        cf.commander.send_velocity_world_setpoint(vx, 0, 0, 0)
        time.sleep(sleep_time)

def forward_circle(cf1, cf2, cf3):
    fp = open('log.csv', 'w')
    fp.write('i; TZ; vz; CX; CY; k; R; D_12; D_23; v_f; v_cruis; k_f; p12; p23;')
    fp.write('px1; py1; pz1; d1; phi1; angle1; v1; vx1; vy1; vz1;')
    fp.write('px2; py2; pz2; d2; phi2; angle2; v2; vx2; vy2; vz2;')
    fp.write('px3; py3; pz3; d3; phi3; angle3; v3; vx3; vy3; vz3;')
    fp.write('\n')
    steps = 20000
    for i in range (steps):

        print ("forward_circle" + str(i))
        print(position_estimate_cf1)
        print(position_estimate_cf2)
        print(position_estimate_cf3)
        
        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]
        pz_1 = position_estimate_cf1[2]

        px_2 = position_estimate_cf2[0]
        py_2 = position_estimate_cf2[1]
        pz_2 = position_estimate_cf2[2]

        px_3 = position_estimate_cf3[0]
        py_3 = position_estimate_cf3[1]
        pz_3 = position_estimate_cf1[3]

        d_1, phi_1 = distance_to_centre (px_1, py_1)
        angle_1 = phase_angle (d_1, phi_1)

        d_2, phi_2 = distance_to_centre (px_2, py_2)
        angle_2 = phase_angle (d_2, phi_2)

        d_3, phi_3 = distance_to_centre (px_3, py_3)
        angle_3 = phase_angle (d_3, phi_3)

        p12 = phase_shift(px_1, py_1, px_2, py_2)
        p23 = phase_shift(px_2, py_2, px_3, py_3)

        v1, v2, v3 = velocity(p12, p23)

        vx1, vy1 = get_velocity(v1, angle_1)
        vx2, vy2 = get_velocity(v2, angle_2)
        vx3, vy3 = get_velocity(v3, angle_3)

        vz1 = 0
        if pz_1 < T_Z:
            vz1 = v_z
        if pz_1 > T_Z:
            vz1 = -v_z

        vz2 = 0
        if pz_2 < T_Z:
            vz2 = v_z
        if pz_2 > T_Z:
            vz2 = -v_z

        vz3 = 0
        if pz_3 < T_Z:
            vz3 = v_z
        if pz_3 > T_Z:
            vz3 = -v_z              
        
        fp.write(
            str(i) + ';' +
            str(T_Z) + ';' +
            str(v_z) + ';' +
            str(CX) + ';' +
            str(CY) + ';' +
            str(k) + ';' +
            str(R) + ';' +
            str(D_12) + ';' +
            str(D_23) + ';' +
            str(v_f) + ';' +
            str(v_cruis) + ';' +
            str(k_f) + ';' +
            str(p12) + ';' +
            str(p23) + ';' +

            str(px_1) + ';' +
            str(py_1) + ';' +
            str(pz_1) + ';' +
            str(d_1) + ';' +
            str(phi_1) + ';' +
            str(angle_1) + ';' +
            str(v1) + ';' +
            str(vx1) + ';' +
            str(vy1) + ';' +
            str(vz1) + ';' +

            str(px_2) + ';' +
            str(py_2) + ';' +
            str(pz_2) + ';' +
            str(d_2) + ';' +
            str(phi_2) + ';' +
            str(angle_2) + ';' +
            str(v2) + ';' +
            str(vx2) + ';' +
            str(vy2) + ';' +
            str(vz2) + ';' +

            str(px_3) + ';' +
            str(py_3) + ';' +
            str(pz_3) + ';' +
            str(d_3) + ';' +
            str(phi_3) + ';' +
            str(angle_3) + ';' +
            str(v3) + ';' +
            str(vx3) + ';' +
            str(vy3) + ';' +
            str(vz3) + ';' +
            '\n'
        )

        cf1.commander.send_velocity_world_setpoint(vx1, vy1, vz1, 0)
        cf2.commander.send_velocity_world_setpoint(vx2, vy2, vz2, 0)
        cf3.commander.send_velocity_world_setpoint(vx3, vy3, vz3, 0)
    
    fp.close()

def land(cf1, cf2, cf3, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print ("land" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf3.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf1.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()
    cf3.commander.send_stop_setpoint()

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

def velocity(p_12, p_23):
    v1 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (p_12 - D_12))
    v2 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (-p_12 + D_12 + p_23 - D_23))
    v3 = v_cruis + v_f * (2 / math.pi) * math.atan(k_f * (-p_23 + D_23))
    return (v1, v2, v3)

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
            with SyncCrazyflie(URI3, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf3:
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

                logconf3 = LogConfig(name='Position', period_in_ms=10)
                logconf3.add_variable('stateEstimate.x', 'float')
                logconf3.add_variable('stateEstimate.y', 'float')
                logconf3.add_variable('stateEstimate.z', 'float')
                scf3.cf.log.add_config(logconf3)
                logconf3.data_received_cb.add_callback(log_pos_callback_cf3)

                logconf1.start()
                logconf2.start()
                logconf3.start()

                cf1=scf1.cf
                cf2=scf2.cf
                cf3=scf3.cf

                # взлетаем
                take_off(cf1, cf2, cf3, T_Z)

                # летим по кругу
                forward_circle(cf1, cf2, cf3)

                # садимся
                land(cf1, cf2, cf3, 0)

                logconf1.stop()
                logconf2.stop()
                logconf3.stop()
