import pandas
import numpy as np
import math
from UsefulFunctions import FirstOrderPNC, ProcessCovariance, KalmanGain, CovarianceUpdate
import matplotlib
import matplotlib.pyplot as plt


# Importing the sensor data
dataframe = pandas.read_csv('KalmanFilter/SensorInfo.csv')

# Converting the info from a pandas dataframe into a numpy array (better for linear algebra purposes)
sensorArray = dataframe.to_numpy()

# Assigning the data to their respective variables
GyroX = sensorArray[:,0]
GyroY = sensorArray[:,1]
GyroZ = sensorArray[:,2]
AccelX = sensorArray[:,3]
AccelY = sensorArray[:,4]
AccelZ = sensorArray[:,5]

# Same process for the Euler angles and their respective rates
angleData = pandas.read_csv('KalmanFilter/EulerAngles.csv')

EulerAngleArray = angleData.to_numpy()

rateData = pandas.read_csv('KalmanFilter/EulerRates.csv')

EulerRateArray = rateData.to_numpy()

# Assigning the data to their respective variables
phi = EulerAngleArray[:,0]
theta = EulerAngleArray[:,1]
psi = EulerAngleArray[:,2]

phi_dot = EulerRateArray[:,0]
theta_dot = EulerRateArray[:,1]
psi_dot = EulerRateArray[:,2]


# Gyro Noise Specs:
# Total RMS Noise = 0.1 °/s rms
# Rate Noise spectral density = 0.01 °/s /√Hz

# Accelerometer Noise Specs
# Noise power spectral density (low noise mode) = 300 µg/√Hz

dt = 1/50
TF = 50

# Prediction 

# x = Fx + w

F = np.array([[1, 0, 0, dt, 0, 0], 
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]])

# State Matrix
Xk_1 =  np.zeros((F.shape[0], 1))

# Covariance Matrix 
Pk_1 = np.identity(F.shape[0])
Pk_1 *= 500


# Noise
AccelSpectralDensity = 300e-6*np.sqrt(dt)

GyroSpectralDensity = 0.01*np.sqrt(dt)

Wk = 0

Qk = FirstOrderPNC(GyroSpectralDensity,dt)


# Measurement noise
Rk = np.identity(Pk_1.shape[0])
Rk*= 0.3

# Measurement Matrix
H = np.identity(Pk_1.shape[0])

# Values we want to plot 
PhiKalman = []
ThetaKalman = []
PsiKalman = []
PhiDotKalman = []
ThetaDotKalman = []
PsiDotKalman = []


# Sensor was not moving so everything should be close to 0
TrueVal = 0

# Need the standard deviation and residuals to evaluate the filter mathematically
PosPhiSTD = []
PosThetaSTD = []

SpeedPhiSTD = []
SpeedThetaSTD = []

ResidualPhi = []
ResidualTheta = []

ResidualPhiDot = []
ResidualThetaDot = []

time = []
sec = 0

for i in range(TF):

    # Prior 
    Xkp = np.dot(F,Xk_1) + Wk

    # Process Noise Covariance
    Pkp = ProcessCovariance(F,Pk_1) + Qk

    # Innovation Covariance
    Sk = ProcessCovariance(H,Pk_1) + Rk

    # Measurement
    zk = np.array([[AccelX[i]],[AccelY[i]],[AccelZ[i]],[GyroX[i]],[GyroY[i]],[GyroZ[i]]],dtype=float)

    # Measurement Model Accelerometer
    
    # |ax|     |    -sin(θ)    |
    # |ay|  =  |  cos(θ)sin(φ) |
    # |az|     |  cos(θ)cos(φ) |
 
    # Measurement Model Gyroscope
    #                   .   .
    # |p|     |         φ - ψsin(θ)      |
    #           .         .
    # |q|  =  | θcos(φ) + ψcos(θ)sin(φ)  |
    #           .               .
    # |r|     | ψcos(θ)cos(φ) - θsin(φ)  |

    h_of_x = np.array([[ -math.sin(Xkp[1])   ],
                         [  math.cos(Xkp[1]) * math.sin(Xkp[0])   ],
                         [  math.cos(Xkp[1]) * math.cos(Xkp[0])   ],
                         [  Xkp[3] - (Xkp[5] * math.sin(Xkp[1]))  ],
                         [  (Xkp[4] * math.cos(Xkp[0])) + Xkp[5] * math.cos(Xkp[1]) * math.sin(Xkp[0])  ],
                         [  (Xkp[5] * math.cos(Xkp[1]) * math.cos(Xkp[0])) - Xkp[4] * math.sin(Xkp[0])  ]],dtype=float)

    # Measurement Jacobian (partial derivatives)
    H = np.array([[0, -math.cos(Xkp[1]), 0, 0, 0, 0],
        [math.cos(Xkp[0])*math.cos(Xkp[1]), -math.sin(Xkp[0])*math.sin(Xkp[1]), 0, 0, 0, 0],
        [-math.cos(Xkp[1])*math.sin(Xkp[0]), -math.cos(Xkp[0])*math.sin(Xkp[1]), 0, 0, 0, 0],
        [0, -Xkp[5]*math.cos(Xkp[1]), 0, 1, 0, -math.sin(Xkp[1])],
        [Xkp[5]*math.cos(Xkp[0])*math.cos(Xkp[1]), -Xkp[5]*math.sin(Xkp[0])*math.sin(Xkp[1]), -Xkp[4]*math.sin(Xkp[2]), 0, math.cos(Xkp[2]), math.cos(Xkp[1])*math.sin(Xkp[2])],
        [(-Xkp[4]*math.cos(Xkp[0])) - Xkp[5]*math.cos(Xkp[1])*math.sin(Xkp[0]), -Xkp[5]*math.cos(Xkp[0])*math.sin(Xkp[1]),0,0,-math.sin(Xkp[0]),math.cos(Xkp[0])*math.cos(Xkp[1])]],dtype=float)

    # Innovation (Resdiual)
    yk = zk - h_of_x

    # Kalman Gain
    K = KalmanGain(Pkp,H,Sk)

    # Posterior
    Xk = Xkp + np.dot(K,yk)

    # Covariance Update
    Pk = CovarianceUpdate(Pkp,K,H,Rk)

    # Redefining For Next Iteration
    Xk_1 = Xk
    Pk_1 = Pk

    sec+=1
    
    # Store for plotting
    time.append(sec)

    PhiKalman.append(Xk[0].item())
    ThetaKalman.append(Xk[1].item())
    PsiKalman.append(Xk[2].item())
    PhiDotKalman.append(Xk[3].item())
    ThetaDotKalman.append(Xk[4].item())
    PsiDotKalman.append(Xk[5].item())


# Plotting

plt.figure(1)
plt.plot(time, PhiKalman, label="Kalman Filter")
plt.plot(time, phi_dot, label = "Dead-Reckoning")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Roll Angle (X-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()

plt.figure(2)
plt.plot(time, ThetaKalman, label="Kalman Filter")
plt.plot(time, theta, label = "Dead-Reckoning")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Pitch Angle (Y-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()

plt.figure(3)
plt.plot(time, PhiDotKalman, label="Kalman Filter")
plt.plot(time, phi_dot, label = "Measured")
plt.ylabel('Rate (rad/s)')
plt.xlabel('Time (sec)')
plt.legend()
plt.grid()
plt.show()

plt.figure(4)
plt.plot(time, ThetaDotKalman, label="Kalman Filter")
plt.plot(time, theta_dot, label = "Measured")
plt.ylabel('Rate (rad/s)')
plt.xlabel('Time (sec)')
plt.legend()
plt.grid()
plt.show()

plt.figure(5)
plt.plot(time, PsiDotKalman, label="Kalman Filter")
plt.plot(time, psi_dot, label = "Measured")
plt.ylabel('Rate (rad/s)')
plt.xlabel('Time (sec)')
plt.legend()
plt.grid()
plt.show()