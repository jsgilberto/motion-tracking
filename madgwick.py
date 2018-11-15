import numpy as np
import math

class Madgwick:
    def __init__(self, sample_period = 1/100, quaternion = [1,0,0,0], beta = 1):
        self._sample_period = 1/100
        self._quaternion = [1, 0, 0, 0]
        self._quaternion_avg = np.array([1, 0, 0, 0])
        self._fifo_q = []
        self._beta = 1

    def quaternion_fifo(self, size=5):
        """
        This method smooths quaternion data
        """
        self._fifo_q.append(np.array(self._quaternion))

        if len(self._fifo_q) > size:
            self._fifo_q.pop(0)

        for element in self._fifo_q:
            self._quaternion_avg = self._quaternion_avg + element
        
        self._quaternion_avg = self._quaternion_avg / len(self._fifo_q)

    def update(self, accelerometer, gyroscope, magnetometer):
        """
        Implementation of madgwick's algorithm
        """
        q = self._quaternion
        
        # normalise data measurements
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        magnetometer = magnetometer / np.linalg.norm(magnetometer)

        # reference direction of Earth's magnetic field
        m = np.concatenate([[0], magnetometer])
        
        h = self.quaternion_product(q, self.quaternion_product(m, self.quaternion_conjugate(q)))
        b = [0, np.linalg.norm([h[1], h[2]]), 0, h[3]]

        # gradient descent algorithm corrective step
        F = [2 * (q[1] * q[3] - q[0] * q[2]) - accelerometer[0],
             2 * (q[0] * q[1] + q[2] * q[3]) - accelerometer[1],
             2 * (0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
             2 * b[1] * (0.5 - q[2]**2 - q[3]**2) + 2 * b[3]*(q[1] * q[3] - q[0] * q[2]) - magnetometer[0],
             2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]) - magnetometer[1],
             2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1]**2 - q[2]**2) - magnetometer[2]]
        
        J = np.matrix([
            [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
            [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
            [0, -4 * q[1], -4 * q[2], 0],
            [-2 * b[3] * q[2], 2 * b[3] * q[3], -4 * b[1] * q[2] - 2 * b[3] * q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
            [-2 * b[1] * q[3] + 2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3], -2*b[1]*q[0]+2*b[3]*q[2]],
            [2*b[1]*q[2], 2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2], 2*b[1]*q[1]]])
        
        step = (np.matmul(np.transpose(J), F))
        step = step / np.linalg.norm(step)
        
        # compute rate of change of quaternion
        q_dot = 0.5 * self.quaternion_product(q, [0, gyroscope[0], gyroscope[1], gyroscope[2]]) \
                    - self._beta * step
        
        # integrate to yield quaternion
        q = q + q_dot * self._sample_period
        self._quaternion = q / np.linalg.norm(q)
        self._quaternion = np.squeeze(np.asarray(self._quaternion))
        self.quaternion_fifo(15)

    def update_inaccurate(self, accelerometer, gyroscope):
        """
        This method does not use magnetometer data, so it's more inaccurate
        """
        q = self._quaternion

        # normalise accelerometer measurement
        accelerometer = accelerometer / np.linalg.norm(accelerometer)

        # gradient descent algorithm corrective step
        F = [2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
             2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
             2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]]

        J = np.matrix([[-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
                       [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
                       [0, -4*q[1], -4*q[2], 0]

        ])
        step = (np.matmul(np.transpose(J), F))
        step = step / np.linalg.norm(step)

        # compute rate of change of quaternion
        q_dot = 0.5 * self.quaternion_product(q, [0, gyroscope[0], gyroscope[1], gyroscope[2]]) \
                - self._beta * np.transpose(step)

        # integrate to yield quaternion
        q = q + q_dot * self._sample_period
        self._quaternion = q / np.linalg.norm(q)
        self._quaternion = np.squeeze(np.asarray(q))


    @staticmethod
    def quaternion_product(a, b):
        """
        Returns the product of two quaternions
        """
        a = np.array(a)
        b = np.array(b)
        v0 = a[0]
        w0 = b[0]
        v = a[-3:]
        w = b[-3:]
        z = v0 * w0 - np.inner(v, w) # esto es una constante
        z = np.array([z])
        vector = v0 * w + w0 * v + np.cross(v, w)
        return np.concatenate([z, vector])

    @staticmethod
    def quaternion_conjugate(a):
        """
        Returns the quaternion conjugate
        """
        return [a[0], -a[1], -a[2], -a[3]]
    
    @staticmethod
    def quaternion_to_euler_angle(w, x, y, z, degrees):
        """
        This method returns yaw, pitch and roll from a given quaternion
        """
        dqw = w
        dqx = x
        dqy = y
        dqz = z
        
        ysqr = dqy * dqy
        t0 = -2 * (ysqr + dqz * dqz) + 1
        t1 = 2 * (dqx * dqy - dqw * dqz)
        t2 = -2 * (dqx * dqz + dqw * dqy)
        t3 = 2 * (dqy * dqz - dqw * dqx)
        t4 = -2 * (dqx * dqx + ysqr) + 1

        # Keep t2 within range of asin (-1, 1)
        if t2 > 1:
            t2 = 1.0
        if t2 < -1:
            t2 = -1.0
    
        pitch = math.asin(t2) * 2
        roll = math.atan2(t3, t4)
        yaw = math.atan2(t1, t0)
        
        if degrees:
            pitch *= (180.0 / np.pi)
            roll *= (180.0 / np.pi)
            yaw *= (180.0 / np.pi)
            if pitch < 0:
                pitch = 360.0 + pitch
            if roll < 0:
                roll = 360.0 + roll
            if yaw < 0:
                yaw = 360.0 + yaw
        return pitch, roll, yaw
    
    @staticmethod
    def quat_to_rotmatrix(q):
        """
        Returns the rotation matrix from a given quaternion
        """
        qw, qx, qy, qz = q
        rot_mat = np.matrix(
           [[1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw,          0],
            [2*qx*qy + 2*qz*qw,         1 - 2*qx**2 - 2*qz**2,	2*qy*qz - 2*qx*qw,          0],
            [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,      1 - 2*qx**2 - 2*qy**2,      0],
            [0,                         0,                      0,                          1]])
        return rot_mat