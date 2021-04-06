import pandas
import numpy as np
from UsefulFunctions import FirstOrderPNC, ProcessCovariance, KalmanGain, CovarianceUpdate, EulerRate, AccelModel, MeasurementJacobian
import matplotlib
import matplotlib.pyplot as plt


# Importing the sensor data
dataframe = pandas.read_csv('SensorInfo.csv')

# Converting the info from a pandas dataframe into a numpy array (better for linear algebra purposes)
sensorArray = dataframe.to_numpy()

# Assigning the data to their respective variables
# GyroX = sensorArray[:,0]
# GyroY = sensorArray[:,1]
# GyroZ = sensorArray[:,2]
# AccelX = sensorArray[:,3]
# AccelY = sensorArray[:,4]
# AccelZ = sensorArray[:,5]

# To NED Frame
GyroX =  sensorArray[:,1]
GyroY =  sensorArray[:,0]
GyroZ =  -1*sensorArray[:,2]

AccelX = sensorArray[:,4]
AccelY = sensorArray[:,3]
AccelZ = -1*sensorArray[:,5]

# Gyro Noise Specs:
# Total RMS Noise = 0.1 °/s rms
# Rate Noise spectral density = 0.01 °/s /√Hz

# Accelerometer Noise Specs
# Noise power spectral density (low noise mode) = 300 µg/√Hz

dt = 1/100
TF = 50

# Prediction 

# x = Fx + w

F = np.array([[1, 0, 0], 
              [0, 1, 0],
              [0, 0, 1]])

G = np.array([[dt, 0, 0], 
              [0, dt, 0],
              [0, 0, dt]])
            

# State Matrix
Xk_1 =  np.zeros((F.shape[0], 1))

u = Xk_1

Phi = Xk_1[0]
Theta = Xk_1[1]
Psi = Xk_1[2]

# Covariance Matrix 
Pk_1 = np.identity(F.shape[0])
Pk_1 *= 500


# Noise
AccelSpectralDensity = 300e-6*np.sqrt(dt)

GyroSpectralDensity = 0.01*np.sqrt(dt)

Wk = 0

Qk = np.identity(F.shape[0])
Qk *= GyroSpectralDensity 


# Measurement noise
Rk = np.identity(Pk_1.shape[0])
Rk*= 0.3

# Measurement Matrix
H = np.identity(Pk_1.shape[0])

# Values we want to plot 
PhiKalman = []
ThetaKalman = []
PsiKalman = []

time = []
sec = 0

for i in range(TF):

    # Converting Gyro to Euler Rates
    Gyro = np.array([[GyroX[i]],
                     [GyroY[i]], 
                     [GyroZ[i]]],dtype=float)

    phiDot, thetaDot, psiDot = EulerRate(Phi,Theta, Gyro)

    # Inputs to the system
    u[0] = phiDot
    u[1] = thetaDot
    u[2] = psiDot

    # Prior 
    Xkp = np.dot(F,Xk_1) + np.dot(G,u)+ Wk

    # Process Noise Covariance
    Pkp = ProcessCovariance(F,Pk_1) + Qk

    # Innovation Covariance
    Sk = ProcessCovariance(H,Pkp) + Rk

    # Measurement
    zk = np.array([[AccelX[i]],
                   [AccelY[i]],
                   [AccelZ[i]]],dtype=float)

    # Measurement Model Accelerometer
    
    # |ax|     |    -sin(θ)    |
    # |ay|  =  |  cos(θ)sin(φ) |
    # |az|     |  cos(θ)cos(φ) |

    h_of_x = AccelModel(Xkp[0],Xkp[1])

    # Measurement Jacobian (partial derivatives)
    H = MeasurementJacobian(Xkp[0],Xkp[1])

    # Innovation (Resdiual)
    yk = zk - h_of_x

    # Kalman Gain
    K = KalmanGain(Pkp,H,Sk)

    # Posterior
    Xk = Xkp + np.dot(K,yk)

    Phi = Xk[0]
    Theta = Xk[1]
    Psi = Xk[2]

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


# Plotting

plt.figure(1)
plt.plot(time, PhiKalman, label="Kalman Filter")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Roll Angle (X-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()

plt.figure(2)
plt.plot(time, ThetaKalman, label="Kalman Filter")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Pitch Angle (Y-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()

plt.figure(3)
plt.plot(time, PsiKalman, label="Kalman Filter")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Yaw Angle (Z-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()
