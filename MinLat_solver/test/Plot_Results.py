import matplotlib.pyplot as plt
import numpy as np


def plotSensor(x, y, r, fig):
	circle = plt.Circle((x, y), r, color='b', fill=False)
	fig.plot(x, y, 'o', color='y', markersize=3)
	fig.add_patch(circle)


# Create a plot
ax = plt.gca()
# Clear anything on the plot
ax.cla()

# Set the X and Y axis
ax.set_xlim((-50, 450))
ax.set_ylim((-50, 450))

# Array to hold sensor data
sensorArray = []
# Array to hold path data
pathArray = []

# Get sensor data from file
with open('test2.txt') as f:
	n = [int(x) for x in next(f).split()] # read first line3
	print(n)
	for i in range(n[0]):
		sensorArray.append([float(x) for x in next(f).split()])
	depot = [float(x) for x in next(f).split()]

print(n)
print(sensorArray)
print(depot)

# Plot each sensor
for node in sensorArray:
	plotSensor(node[0], node[1], node[2], ax)

# Get path data from file
with open('output_path.txt') as f:
	for line in f:
		pathArray.append([float(x) for x in line.split()])

print(pathArray)

# Plot the paths
for i in range(len(pathArray)-1):
	ax.plot([pathArray[i][0], pathArray[i+1][0]], [pathArray[i][1], pathArray[i+1][1]], 'go--', linewidth=1, markersize=3)

# Plot depot
ax.plot(depot[0], depot[1], 'o', color='r', markersize=5)

plt.show()
