import time
from icm20948 import ICM20948

time.sleep(0.1)
imu = ICM20948(0x69)
time.sleep(0.1)

while True:
    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
    print("Accel: {:05.2f} {:05.2f} {:05.2f} Gyro:  {:05.2f} {:05.2f} {:05.2f}".format(ax,ay,az,gx,gy,gz))
    time.sleep(0.25)
