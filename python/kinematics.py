import math
import serial
import sys
import time

comm = serial.Serial(port=sys.argv[1], baudrate=115200, timeout=.1)

def move(s, k, phi, duration):
    command = f"{s:.5f},{k:.5f},{phi:.5f},{duration:.5f}"
    print(command)
    comm.write(bytes(command, 'utf-8'))

def main():
    # allow time for hose to reach 0
    time.sleep(5)
    s = 740 # arc length [mm]
    radius = 300 # radius of circle formed by backbone curve [mm]
    k = 1 / radius # curvature (1/r) [mm^-1]
    duration = 1000
    phi = 0 * math.pi / 180 # angle about z_0 axis [radians]
    move(s, k, phi, duration)
    duration = 5000
    phi = 180 * math.pi / 180 # angle about z_0 axis [radians]
    move(s, k, phi, duration)

if __name__ == "__main__":
    main()
