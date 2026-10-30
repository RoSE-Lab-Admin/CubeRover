from gpiozero import OutputDevice
from time import sleep

pin = OutputDevice(5)

try:
    while True:
        pin.on()
        print("GPIO 5 ON")
        sleep(1)

        pin.off()
        print("GPIO 5 OFF")
        sleep(1)

except KeyboardInterrupt:
    pin.off()
    print("STOPPED")