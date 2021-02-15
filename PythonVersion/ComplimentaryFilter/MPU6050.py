import pandas
import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt

# Importing the sensor data
dataframe = pandas.read_csv('KalmanFilter/SensorInfo.csv')

# Converting the info from a pandas dataframe into a numpy array (better for linear algebra purposes)
sensorArray = dataframe.to_numpy()

# Assigning the data to their respective variables
AccelX = sensorArray[:,3]
AccelY = sensorArray[:,4]
AccelZ = sensorArray[:,5]

# Same process for the Euler angles 
angleData = pandas.read_csv('KalmanFilter/EulerAngles.csv')

EulerAngleArray = angleData.to_numpy()

# Assigning the data to their respective variables
phi = EulerAngleArray[:,0]
theta = EulerAngleArray[:,1]
psi = EulerAngleArray[:,2]

# As well as their rates
rateData = pandas.read_csv('KalmanFilter/EulerRates.csv')

EulerRateArray = rateData.to_numpy()

phi_dot = EulerRateArray[:,0]
theta_dot = EulerRateArray[:,1]
psi_dot = EulerRateArray[:,2]

# Values we want to plot 

PhiAngleComplimentary = []
ThetaAngleComplimentary  = []

AccelAngleX = []
AccelAngleY = []

time = []

# The percentage of the gyro information you want to use
alpha = 0.98

dt = 1/50
TF = 50
sec = 0

# initializing values
Phi = 0
Theta = 0

# Complimentary Filter

for i in range(TF):
    
    PhiAccel = math.atan2(AccelY[i], (math.sqrt((AccelX[i]**2) +  (AccelZ[i]**2))))

    ThetaAccel = math.atan2(-AccelX[i], (math.sqrt((AccelY[i]**2) +  (AccelZ[i]**2))))

    # Essentially using a high-pass filter on the rate gyro values and a low pass filter on the angles from the accelerometer
    newAngleX = alpha*(phi_dot[i]*dt+Phi) + (1-alpha)*PhiAccel
    
    newAngleY = alpha*(theta_dot[i]*dt+Theta) + (1-alpha)*ThetaAccel

    # Store for plotting
    PhiAngleComplimentary.append(newAngleX)
    ThetaAngleComplimentary.append(newAngleY)

    AccelAngleX.append(PhiAccel)
    AccelAngleY.append(ThetaAccel)

    # Redefining for next time step
    sec+=1
    Phi = newAngleX
    Theta = newAngleY

    time.append(sec)

# Plotting

plt.figure(1)
plt.plot(time, PhiAngleComplimentary, label="Complimentary Filter")
plt.plot(time, phi_dot, label = "Gyro Dead-Reckoning")
plt.plot(time, AccelAngleX, label = "Acclerometer Angles")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Roll Angle (X-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()

plt.figure(2)
plt.plot(time, ThetaAngleComplimentary, label="Complimentary Filter")
plt.plot(time, theta_dot, label = "Gyro Dead-Reckoning")
plt.plot(time, AccelAngleY, label = "Acclerometer Angles")
plt.ylabel('Angle (radians)')
plt.xlabel('Time (sec)')
plt.title("Pitch Angle (Y-Axis Rotation)")
plt.legend()
plt.grid()
plt.show()