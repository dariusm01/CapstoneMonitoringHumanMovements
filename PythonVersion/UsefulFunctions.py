import numpy as np
from numpy.linalg import inv

def FirstOrderPNC(SD,dt):
    Qk = np.array ([[(SD*dt**3)/3, 0 , 0, (SD*dt**2)/2,    0,     0],
                  [0, (SD*dt**3)/3,    0,     0,  (SD*dt**2)/2,    0],
                  [0,      0, (SD*dt**3)/3,   0,     0, (SD*dt**2)/2],
                  [(SD*dt**2)/2,       0,     0,    SD*dt,    0,  0],
                  [0,   (SD*dt**2)/2,  0,     0,    SD*dt,    0],
                  [0,       0,   (SD*dt**2)/2,  0,    0,    SD*dt]])

    return Qk


def ProcessCovariance(A,P):
    a = np.dot(A,P) # A*P

    return np.dot(a,np.transpose(A)) # A*P*A^T

def KalmanGain(P,H,S):
    b = np.dot(P,np.transpose(H)) # P*H^T

    return np.dot(b, inv(S)) # (P*H^T)*(S^-1)

def CovarianceUpdate(P,K,H,R):

    # Joseph Form (numerically more stable)

    I = np.identity(H.shape[0])

    a = I - np.dot(K,H) # (I - K*H)
    b = np.dot(a,P) # (I - K*H)*Pkp
    c = np.dot(b, np.transpose(a)) # (I - K*H)*Pkp*(I - K*H)^T

    d = np.dot(K,R) # (K*Rk)
    e = np.dot(d,np.transpose(K)) # (K*Rk*K^T);

    return c + e # (I - K*H)*Pkp*(I - K*H).' + (K*Rk*K.');


