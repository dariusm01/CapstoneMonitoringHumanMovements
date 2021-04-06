import numpy as np
from numpy.linalg import inv
import pandas as pd


def FirstOrderPNC(SD, dt):
    Qk = np.array([[(SD * dt ** 3) / 3, 0, 0, (SD * dt ** 2) / 2, 0, 0],
                   [0, (SD * dt ** 3) / 3, 0, 0, (SD * dt ** 2) / 2, 0],
                   [0, 0, (SD * dt ** 3) / 3, 0, 0, (SD * dt ** 2) / 2],
                   [(SD * dt ** 2) / 2, 0, 0, SD * dt, 0, 0],
                   [0, (SD * dt ** 2) / 2, 0, 0, SD * dt, 0],
                   [0, 0, (SD * dt ** 2) / 2, 0, 0, SD * dt]])

    return Qk


def ProcessCovariance(A, P):
    a = np.dot(A, P)  # A*P

    return np.dot(a, np.transpose(A))  # A*P*A^T


def KalmanGain(P, H, S):
    b = np.dot(P, np.transpose(H))  # P*H^T

    return np.dot(b, inv(S))  # (P*H^T)*(S^-1)


def CovarianceUpdate(P, K, H, R):
    # Joseph Form (numerically more stable)

    I = np.identity(H.shape[0])

    a = I - np.dot(K, H)  # (I - K*H)
    b = np.dot(a, P)  # (I - K*H)*Pkp
    c = np.dot(b, np.transpose(a))  # (I - K*H)*Pkp*(I - K*H)^T

    d = np.dot(K, R)  # (K*Rk)
    e = np.dot(d, np.transpose(K))  # (K*Rk*K^T);

    return c + e  # (I - K*H)*Pkp*(I - K*H).' + (K*Rk*K.');


def EulerRate(phi, theta, Gyro):
    EulerKinematic = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                               [0, np.cos(phi), -1 * np.sin(phi)],
                               [0, np.sin(phi) * (1 / (np.cos(theta))), np.cos(phi) * (1 / (np.cos(theta)))]],
                              dtype=float)

    EulerRates = np.dot(EulerKinematic, Gyro)

    phiDot = EulerRates[0]
    thetaDot = EulerRates[1]
    psiDot = EulerRates[2]

    return phiDot, thetaDot, psiDot


def AccelModel(phi, theta):
    Accel = np.array([[-np.sin(theta)],
                      [np.cos(theta) * np.sin(phi)],
                      [np.cos(theta) * np.cos(phi)]], dtype=float)

    return np.reshape(Accel, (3, 1))


def MeasurementJacobian(phi, theta):
    J = np.array([[0, -np.cos(theta), 0],
                  [np.cos(theta) * np.cos(phi), -np.sin(theta) * np.sin(phi), 0],
                  [-np.cos(theta) * np.sin(phi), -np.sin(theta) * np.cos(phi), 0]], dtype=float)

    return np.reshape(J, (3, 3))

def sensorData(fileName):
    # Importing the sensor data
    dataframe = pd.read_csv(fileName)

    # Converting the info from a pandas dataframe into a numpy array (better for linear algebra purposes)
    sensorArray = dataframe.to_numpy()

    return sensorArray
