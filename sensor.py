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
        # average fifos
        self._fifo_accel = []
        self._fifo_gyro = []
        self._fifo_mag = []
    
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
        """
        This method updates all the values of the sensor object through serial port
        """
        data = self._ser.readline()
        string_data = data.decode()
        try:
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
        except:
            print("Oops, usb problems!")
            ax = self.accelerometer[0]
            ay = self.accelerometer[1]
            az = self.accelerometer[2]
            gx = self.gyroscope[0]
            gy = self.gyroscope[1]
            gz = self.gyroscope[2]
            mx = self.magnetometer[0]
            my = self.magnetometer[1]
            mz = self.magnetometer[2]
        
        self.accelerometer = np.array([ax, ay, az])
        self.gyroscope = np.array([gx, gy, gz])
        self.magnetometer = np.array([mx, my, mz])
    
    def update_values_with_moving_average(self, size=5):
        """
        This method implements moving average to smooth sensor data
        """
        self.update_values()
        self._fifo_accel.append(self.accelerometer)
        self._fifo_gyro.append(self.gyroscope)
        self._fifo_mag.append(self.magnetometer)

        if len(self._fifo_accel) > size:
            self._fifo_accel.pop(0)
            self._fifo_gyro.pop(0)
            self._fifo_mag.pop(0)

        for element in self._fifo_accel:
            self._accel_avg = self._accel_avg + element
        for element in self._fifo_gyro:
            self._gyro_avg = self._gyro_avg + element
        for element in self._fifo_mag:
            self._mag_avg = self._mag_avg + element

        self._accel_avg = self._accel_avg / len(self._fifo_accel)
        self._gyro_avg = self._gyro_avg / len(self._fifo_gyro)
        self._mag_avg = self._mag_avg / len(self._fifo_mag)