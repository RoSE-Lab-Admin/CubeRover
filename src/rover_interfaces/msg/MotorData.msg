#Encoder and motor telemetry

#revolutions of encoders from boot
int32 enc1 #front left
int32 enc2 #back left
int32 enc3 #front right
int32 enc4 #back right

#current speed of motors
int32 vel1
int32 vel2
int32 vel3
int32 vel4

#current draw from motors (measured in mA)
int32 m1current
int32 m2current
int32 m3current
int32 m4current