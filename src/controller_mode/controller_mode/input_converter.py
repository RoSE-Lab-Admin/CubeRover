import numpy as np



def turn_calc(rX, rY, trig):
    rX = .01 if rX == 0 else rX

    MAXSPEED = 35 #cm/s
    WHEELBASE = 40.96 / 2 #cm for dist from each wheel to center


    angle = np.arctan2(rY, rX)  # radians, -pi to pi

    #turning radius
    min = 20.48
    max = 160 #based off of bailey's mobility tests

    norm_angle = abs(np.rad2deg(angle))  # 0 to 180 degrees
    turn_radius = (norm_angle / 180.0) * (max - min) + max

    if rX > 0:
        r_inner = turn_radius - WHEELBASE
        r_outer = turn_radius + WHEELBASE
    else:
        r_inner = turn_radius + WHEELBASE
        r_outer = turn_radius - WHEELBASE
    
    v = vel_calc(trig)
    omega = v / turn_radius  # rad/s

    vL = omega * r_inner
    vR = omega * r_outer

    # Flip sign for reverse
    if trig < 0:
        vL *= -1
        vR *= -1

    return np.rad2deg(angle), turn_radius, vL, vR



    return norm_angle, radius, vel1_enc, vel2_enc

def vel_calc(trig):
    max_speed = 35 #cm/s
    return trig/2 * max_speed

def linvel_calc(trig):
    #10 cm/s fastest speed. wheel radius 15 cm 
    #1 inch is 2.54 cm
    #need rotations per minute, rpm * 2pi / 60 = w
    #v=wr
    #trig vals are 0-2
    encoder = 5281.7 #enc/rot
    max_speed = 35 #cm/s
    wheel_radius = 15 #cm

    max_enc = max_speed #max val in en/s

    enc_speed = trig/2 * max_enc

    return enc_speed

def testvel_calc(vel):
    encoder = 5281.7 #enc/rot
    wheel_radius = 15 #cm

    if vel > 35:
        vel = 35

    enc_speed = (vel/wheel_radius)/(2*np.pi)*encoder #max val in en/s

    return enc_speed

def position_calc(dist):
    encoder = 5281.7
    wheel_radius = 15

    pos = dist/(2*np.pi*wheel_radius)*encoder

    return pos

    




