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

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0]
position_estimate_cf2 = [0, 0]
position_estimate_cf3 = [0, 0]

def log_pos_callback_cf1(timestamp, data, logconf):
    global position_estimate_cf1
    position_estimate_cf1[0] = data['stateEstimate.x']
    position_estimate_cf1[1] = data['stateEstimate.y']

def log_pos_callback_cf2(timestamp, data, logconf):
    global position_estimate_cf2
    position_estimate_cf2[0] = data['stateEstimate.x']
    position_estimate_cf2[1] = data['stateEstimate.y']

def log_pos_callback_cf3(timestamp, data, logconf):
    global position_estimate_cf3
    position_estimate_cf3[0] = data['stateEstimate.x']
    position_estimate_cf3[1] = data['stateEstimate.y']

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
    # fp = open('log.csv', 'w')
    # fp.write('i; x; y; d; phi; angle; vx; vy; CX; CY; k; R; v \n')
    steps = 20000
    for i in range (steps):

        print ("forward_circle" + str(i))
        print(position_estimate_cf1)
        print(position_estimate_cf2)
        print(position_estimate_cf3)
        
        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]

        px_2 = position_estimate_cf2[0]
        py_2 = position_estimate_cf2[1]

        px_3 = position_estimate_cf3[0]
        py_3 = position_estimate_cf3[1]

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
        # fp.write(str(i) + ';' +
        #     str(position_estimate[0]) + ';' +
        #     str(position_estimate[1]) + ';' +
        #     str(d) + ';' +
        #     str(phi) + ';' +
        #     str(angle) + ';' +
        #     str(vx) + ';' +
        #     str(vy) + ';' +
        #     str(CX) + ';' +
        #     str(CY) + ';' +
        #     str(k) + ';' +
        #     str(R) + ';' +
        #     str(v) + '\n'
        # )
        cf1.commander.send_velocity_world_setpoint(vx1, vy1, 0, 0)
        cf2.commander.send_velocity_world_setpoint(vx2, vy2, 0, 0)
        cf3.commander.send_velocity_world_setpoint(vx3, vy3, 0, 0)
    
    # fp.close()

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
                scf1.cf.log.add_config(logconf1)
                logconf1.data_received_cb.add_callback(log_pos_callback_cf1)

                logconf2 = LogConfig(name='Position', period_in_ms=10)
                logconf2.add_variable('stateEstimate.x', 'float')
                logconf2.add_variable('stateEstimate.y', 'float')
                scf2.cf.log.add_config(logconf2)
                logconf2.data_received_cb.add_callback(log_pos_callback_cf2)

                logconf3 = LogConfig(name='Position', period_in_ms=10)
                logconf3.add_variable('stateEstimate.x', 'float')
                logconf3.add_variable('stateEstimate.y', 'float')
                scf3.cf.log.add_config(logconf3)
                logconf3.data_received_cb.add_callback(log_pos_callback_cf3)

                logconf1.start()
                logconf2.start()
                logconf3.start()

                cf1=scf1.cf
                cf2=scf2.cf
                cf3=scf3.cf

                # взлетаем
                take_off(cf1, cf2, cf3, 0.5)

                # летим по кругу
                forward_circle(cf1, cf2, cf3)

                # садимся
                land(cf1, cf2, cf3, 0)

                logconf1.stop()
                logconf2.stop()
                logconf3.stop()
