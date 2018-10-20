from sensor import Sensor
from madgwick import Madgwick
import numpy as np
import math

#ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port
mpu9250 = Sensor('/dev/ttyACM0', 115200)
m = Madgwick()
while True:
    mpu9250.update_values()
    #mpu9250.update_values_with_average(5)
    m.update(mpu9250.accelerometer, mpu9250.gyroscope * np.pi / 180, mpu9250.magnetometer)
    q = m._quaternion
    q = np.squeeze(np.asarray(q))
    pitch, roll, yaw = Madgwick.quaternion_to_euler_angle(q[0], q[1], q[2], q[3], True)
    print(pitch, roll, yaw)

