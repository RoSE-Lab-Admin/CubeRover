import numpy as np



def turn_calc(rX, rY, trig):
    rX = .01 if rX == 0 else rX

    encoder = 5281.7 #encoder counts per rotation

    fraction = rY/rX
    angle = np.arctan(fraction) * 180/np.pi

    #turning radius
    min = .2048 
    max = 1.60 #based off of bailey's mobility tests

    range = max-min

    ang = anglin_calc(abs(trig))

    if rX > 0: 
        norm_angle = 90-angle
        radius = abs(norm_angle)/180 * range + min #normalizes 
        r1 = radius+.2048
        r2 = radius-.2048

    else:
        norm_angle = -90-angle
        radius = abs(norm_angle)/180 * range + min #normalizes 
        r1 = radius-.2048
        r2 = radius+.2048

    
    vel1 = ang*r1 #rot/sec
    vel2 = ang*r2
    
    vel1_enc = vel1
    vel2_enc = vel2

    if trig < 0:
        vel1_enc = -1 * vel1_enc
        vel2_enc = -1 * vel2_enc



    return norm_angle, radius, vel1_enc, vel2_enc

def anglin_calc(trig):
    max_speed = 35 #cm/s

    ang = trig/2 * max_speed
    return ang

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

    




