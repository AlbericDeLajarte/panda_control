import numpy as np
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter

data = np.loadtxt("data_friction.txt")
data[1:15, :] = np.rad2deg(data[1:15, :])

time = data[0].T
position = data[1:8, :]
velocity = data[8:15, :]
commandTorque = data[15:22, :]
jointTorque = data[22:29, :]
mass = data[29, :]

fc = 0.01
nPoly = 3
window = int(((nPoly+1)/fc + 4.6)/3.2)


smoothVelocity = np.gradient(position[0, :], time) 
smoothVelocity = savgol_filter(smoothVelocity, window, nPoly)

smoothTorque = mass*np.gradient(np.deg2rad(smoothVelocity), time) 
smoothTorque = savgol_filter(smoothTorque, window, nPoly)

# smoothJointTorque = savgol_filter(jointTorque[0, :], window, nPoly)

data = np.loadtxt("data_friction.txt")
smoothJointTorque = jointTorque[0, :]
alpha = 0.02
for i in range(len(smoothJointTorque)-1):
    smoothJointTorque[i+1] = alpha*smoothJointTorque[i+1] + (1-alpha)*smoothJointTorque[i]

friction = commandTorque[0, :] - smoothJointTorque


fig, axs = plt.subplots(2, 1)

axs[0].plot(time, position[0, :], label="Position [°]")
axs[0].plot(time, velocity[0, :], label="Speed [°/s]")
axs[0].plot(time, smoothVelocity, label="Smoothed speed [°/s]")
axs[0].legend()
axs[0].set(xlabel="Time [s]")

axs[1].plot(time, smoothTorque, label="Accel torque")
axs[1].plot(time, commandTorque[0, :], label="Command torque")
axs[1].plot(time, data[22, :], label="Measured torque")
axs[1].plot(time, smoothJointTorque, label="Smoothed measured torque")
# axs[1].plot(time, friction, label = "Friction")
axs[1].legend()
axs[1].set(ylabel="N.m")
axs[1].set(xlabel="Time [s]")


fig = plt.figure()
plt.scatter(smoothVelocity, friction, marker="+")
plt.ylabel("friction torque [N.m]")
plt.xlabel("speed [°/s]")



plt.show()
