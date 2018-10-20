import serial
import numpy as np

class Sensor:
    def __init__(self, port, baudrate=115200):
        # config
        self._port = port
        self._baudrate = baudrate
        self._ser = serial.Serial(self._port, self._baudrate)
        # data
        self._accelerometer = []
        self._gyroscope = []
        self._magnetometer = []
        # average data
        self._accel_avg = np.array([0, 0, 0])
        self._gyro_avg = np.array([0, 0, 0])
        self._mag_avg = np.array([0, 0, 0])
    @property
    def accelerometer(self):
        return self._accelerometer

    @property
    def gyroscope(self):
        return self._gyroscope

    @property
    def magnetometer(self):
        return self._magnetometer

    @accelerometer.setter
    def accelerometer(self, value):
        self._accelerometer = value

    @gyroscope.setter
    def gyroscope(self, value):
        self._gyroscope = value
    
    @magnetometer.setter
    def magnetometer(self, value):
        self._magnetometer = value
        
    def update_values(self):
        data = self._ser.readline()
        string_data = data.decode()
        ax, ay, az, gx, gy, gz, mx, my, mz = string_data.split(":")
        ax = float(ax)
        ay = float(ay)
        az = float(az)
        gx = float(gx)
        gy = float(gy)
        gz = float(gz)
        mx = float(mx)
        my = float(my)
        mz = float(mz)
        self.accelerometer = np.array([ax, ay, az])
        self.gyroscope = np.array([gx, gy, gz])
        self.magnetometer = np.array([mx, my, mz])
    
    def update_values_with_average(self, samples=5):
        for i in range(samples):
            self.update_values()
            self._accel_avg = self._accel_avg + self.accelerometer
            self._gyro_avg = self._gyro_avg + self.gyroscope
            self._mag_avg = self._mag_avg + self.magnetometer
        self._accel_avg = self._accel_avg / samples
        self._gyro_avg = self._gyro_avg / samples
        self._mag_avg = self._mag_avg / samples
        self.accelerometer = self._accel_avg
        self.gyroscope = self._gyro_avg
        self.magnetometer = self._mag_avg
