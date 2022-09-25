CX = 0.8
CY = -0.4
k = 0.01
R = 0.3
v = 0.1

def distance_to_centre (px, py):
    d = sqrt((px - CX)**2 + (py - CY)**2)
    phi = math.atan2(px - CX, py - CY)
    print('d= ' + str(d))
    print('phi= ' + str(phi))
    return (d, phi)

def phase_angle (d, phi):
    angle = phi + pi/2 + atan(k * (d - R))
    print('angle= ' + str(angle))
    return angle

def get_velocity(v, angle):
    vx = v * sin(angle)
    vy = v * cos(angle)
    print('vx= ' + str(vx))
    print('vy= ' + str(vy))
    return (vx, vy)
