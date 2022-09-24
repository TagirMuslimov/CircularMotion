import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M/E7E7E7E703'

CX = 0.2
CY = 0.0
k = 0.1
R = 0.1
v = 0.1

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print ("take_off" + str(i))
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def forward(cf, distance):
    sleep_time = 0.1
    vx = 0.1
    steps = int(distance / vx / sleep_time)

    for i in range(steps):
        print ("forward" + str(i))
        cf.commander.send_velocity_world_setpoint(vx, 0, 0, 0)
        time.sleep(sleep_time)

def forward_circle(cf):
    fp = open('log.csv', 'w')
    fp.write('i; x; y; d; phi; angle; vx; vy; CX; CY; k; R; v \n')
    steps = 20000
    for i in range (steps):

        print ("forward_circle" + str(i))
        print(position_estimate)
        px = position_estimate[0]
        py = position_estimate[1]
        d, phi = distance_to_centre (px, py)
        angle = phase_angle (d, phi)
        vx, vy = get_velocity(v, angle)
        fp.write(str(i) + ';' +
            str(position_estimate[0]) + ';' +
            str(position_estimate[1]) + ';' +
            str(d) + ';' +
            str(phi) + ';' +
            str(angle) + ';' +
            str(vx) + ';' +
            str(vy) + ';' +
            str(CX) + ';' +
            str(CY) + ';' +
            str(k) + ';' +
            str(R) + ';' +
            str(v) + '\n'
        )
        cf.commander.send_velocity_world_setpoint(vx, vy, 0, 0)
    
    fp.close()

def land(cf, position):
    landing_time = 5.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print ("land" + str(i))
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

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

def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf:

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        logconf.start()

        cf=scf.cf
        take_off(cf, 0.5)
        forward_circle(cf)
        land(cf, 0)

        logconf.stop()
