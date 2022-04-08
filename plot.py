import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("data.txt")
data[1:, :] = np.rad2deg(data[1:, :])

time = data[0].T
reference = data[8:15, :]
state = data[1:8, :]
error = state-reference

# Plot reference and state
fig, axs = plt.subplots(4, 2)
fig.suptitle("Position trajectory tracking, joint state and reference, K = 40")

for i, ax in enumerate(axs.ravel()[:-1]):
    print(i)
    ax.plot(time, state[i, :])
    ax.plot(time, reference[i, :], ":")
    ax.set(ylabel="joint {} [degree]".format(i))


# Plot error
fig, axs = plt.subplots(4, 2)
fig.suptitle("Position trajectory tracking, joint position error, K = 40")

for i, ax in enumerate(axs.ravel()[:-1]):
    print(i)
    ax.plot(time, error[i, :])
    ax.set(ylabel="joint {} [degree]".format(i))

print(np.sqrt(np.mean((error)**2, 1)))




import roboticstoolbox as rtb

robot = rtb.models.DH.Panda()
target = robot.fkine(np.deg2rad(reference[:, -1]))
position = robot.fkine(np.deg2rad(state[:, -1]))

print("Cartesian error", 1000*(position-target)[0:3, 3], "[mm]")


plt.show()
