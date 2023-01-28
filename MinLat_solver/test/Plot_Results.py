import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rc


def plotSensor(x, y, r, fig):
	circle = plt.Circle((x, y), r, color='b', fill=False)
	fig.plot(x, y, 'o', color='y', markersize=3)
	fig.add_patch(circle)


# Create a plot
ax = plt.gca()
# Clear anything on the plot
ax.cla()

# Set the X and Y axis
ax.set_xlim((-50, 1050))
ax.set_ylim((-50, 1050))

rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
rc('text', usetex=True)

# Add title and axis names
# csfont = {'fontname':'Comic Sans MS'}
plt.xlabel('x-coordinate [m]')
plt.ylabel('y-coordinate [m]')

# Array to hold sensor data
sensorArray = []
# Array to hold path data
pathArray = []

# Get sensor data from file, paper figures are Experiment2/plot_20_7.txt
with open('output_graph.txt') as f:
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
	ax.plot([pathArray[i][0], pathArray[i+1][0]], [pathArray[i][1], pathArray[i+1][1]], 'go--', linewidth=2, markersize=5)

# Plot depot
ax.plot(depot[0], depot[1], 'o', color='r', markersize=5)

plt.show()
