from __future__ import division
import numpy as np
import warnings


class AHRS(object):

    def __init__(self, SamplePeriod=1. / 200, Kp=2., KpInit=0., Quaternion=np.array([1, 0, 0, 0]), Ki=0.,
                 InitPeriod=5.):
        """docstring for AHRS"""
        self.SamplePeriod = SamplePeriod
        self.Quaternion = Quaternion  # output quaternion describing the sensor relative to the Earth
        self.Kp = Kp  # proportional gain
        self.Ki = Ki  # integral gain
        self.KpInit = KpInit  # proportional gain used during initialisation
        self.InitPeriod = InitPeriod  # initialisation period in seconds

        self.q = np.array([1., 0, 0, 0])  # % internal quaternion describing the Earth relative to the sensor
        self.IntError = np.array([[0.], [0.], [0.]])  # % integral error
        self.KpRamped = None  # % internal proportional gain used to ramp during initialisation

    def UpdateIMU(self, Gyr, Acc):
        if np.linalg.norm(Acc) == 0:
            warnings.warn("Accelerometer magnitude is zero.  Algorithm update aborted.")
            return
        else:
            Acc = np.array(Acc / np.linalg.norm(Acc))

        # print Acc
        v = np.array([[2 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])],
                      [2 * (self.q[0] * self.q[1] + self.q[2] * self.q[3])],
                      [self.q[0] ** 2 - self.q[1] ** 2 - self.q[2] ** 2 + self.q[3] ** 2]])

        # print v

        error = np.cross(v, np.transpose([Acc]), axis=0)

        # print error
        self.IntError = self.IntError + error
        Ref = Gyr - np.transpose(self.Kp * error + self.Ki * self.IntError)

        pDot = np.multiply(0.5, self.quaternProd_single(self.q, [0, Ref[0, 0], Ref[0, 1], Ref[0, 2]]))
        self.q = self.q + pDot * self.SamplePeriod;  # % integrate rate of change of quaternion
        self.q = self.q / np.linalg.norm(self.q);
        self.Quaternion = self.quaternConj(self.q);

    def quaternProd_single(self, a, b):

        ab = [None] * 4
        ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
        ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
        ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
        ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
        return ab

    def quaternProd(self, a, b):
        ab = []
        # print a
        # print b
        ab = np.concatenate(
            (a[:, [0]] * b[:, [0]] - a[:, [1]] * b[:, [1]] - a[:, [2]] * b[:, [2]] - a[:, [3]] * b[:, [3]],
             a[:, [0]] * b[:, [1]] + a[:, [1]] * b[:, [0]] + a[:, [2]] * b[:, [3]] - a[:, [3]] * b[:, [2]],
             a[:, [0]] * b[:, [2]] - a[:, [1]] * b[:, [3]] + a[:, [2]] * b[:, [0]] + a[:, [3]] * b[:, [1]],
             a[:, [0]] * b[:, [3]] + a[:, [1]] * b[:, [2]] - a[:, [2]] * b[:, [1]] + a[:, [3]] * b[:, [0]]), axis=1)
        return ab

    def quaternConj(self, q):
        if q.ndim == 1:
            qConj = [q[0], -q[1], -q[2], -q[3]]
        else:
            qConj = np.concatenate((q[:, [0]], -q[:, [1]], -q[:, [2]], -q[:, [3]]), axis=1)
            # print qConj.shape
            # raw_input("wait")
        return qConj

    def quaternRotate(self, v, q):
        row, col = v.shape
        # print row,col
        # print 'shape:', np.zeros((row,1))
        # print 'q: ',  q
        temp = self.quaternProd(q, np.concatenate((np.zeros((row, 1)), v), axis=1))
        v0XYZ = self.quaternProd(temp, self.quaternConj(q))
        v = v0XYZ[:, 1:4]
        # print v.shape
        return v