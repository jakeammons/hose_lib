import math
import serial
import sys
import time

comm = serial.Serial(port=sys.argv[1], baudrate=9600, timeout=.1)

def move(s, k, phi):
    R = 30 # distance between tendon and backbone [mm]
    l_1 = s * (1 - R * k * math.cos(phi)) # [mm]
    l_2 = s * (1 - R * k * math.cos((4 * math.pi / 3) - phi)) # [mm]
    l_3 = s * (1 - R * k * math.cos((2 * math.pi / 3) - phi)) # [mm]

    l_1_d = l_1 - s
    l_2_d = l_2 - s
    l_3_d = l_3 - s
    duration = 1250
    command = f"{l_1_d:.2f},{l_2_d:.2f},{l_3_d:.2f},{duration}"
    print(command)
    comm.write(bytes(command, 'utf-8'))

def main():
    # allow time for hose to reach 0
    time.sleep(5);
    s = 740 # arc length [mm]
    k = .001 # curvature (1/1000) [mm^-1]
    for i in range(0, 375, 15):
        phi = i * math.pi / 180 # angle about z_0 axis [radians]
        move(s, k, phi);
        time.sleep(2.5);

if __name__ == "__main__":
    main()
