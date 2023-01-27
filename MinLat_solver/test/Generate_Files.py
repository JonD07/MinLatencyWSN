import random
import math

INC_NODES = False
INC_ALPHA = True

# Parameters
NUM_PLOTS = 25
R_MAX = 50
R_MIN = 20
# Increasing nodes
START_COUNT = 5
END_COUNT = 100
Node_Increment = 5
ALPHA = 0.00015
# Increasing alpha
NUM_NODES = 50
START_DENSE = 50
END_DENSE = 200
DENSE_INC = 10

FILE_PATH = "Experiment1/"

# What increases?
if INC_NODES:
	# Loop over the number of sensors to use (n)
	for n in range(START_COUNT, (END_COUNT + Node_Increment), Node_Increment):
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Find the max distance a sensor can from the origin
			MAX_COORD = math.sqrt(n/ALPHA)
			# Open the file
			file_name = f"{FILE_PATH}plot_{n}_{i}.txt"
			with open(file_name, 'w') as file:
				file.write(f"{n}\n")
				# Pick base station
				x_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				y_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				for l in range(n):
					# Pick random coordinates
					x = MAX_COORD * random.random()
					y = MAX_COORD * random.random()
					r = (R_MAX - R_MIN) * random.random() + R_MIN
					# Verify we did not pick something over the base station
					while (x_b + r) > x > (x_b - r) and (y_b + r) > y > (y_b - r):
						x = MAX_COORD * random.random()
						y = MAX_COORD * random.random()
						r = (R_MAX - R_MIN) * random.random() + R_MIN
					# Write the results to file
					file.write(f"{x} {y} {r} 1\n")
				# Record BS position
				file.write(f"{x_b} {y_b}\n")

if INC_ALPHA:
	# Loop over the number of sensors to use (n)
	for den in range(START_DENSE, (END_DENSE + DENSE_INC), DENSE_INC):
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Determine alpha
			alpha = den/(1000**2)
			# Find the max distance a sensor can from the origin
			MAX_COORD = math.sqrt(den/alpha)
			# Open the file
			file_name = f"{FILE_PATH}plot_{den}_{i}.txt"
			with open(file_name, 'w') as file:
				file.write(f"{NUM_NODES}\n")
				# Pick base station
				x_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				y_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				for l in range(NUM_NODES):
					# Pick random coordinates
					x = MAX_COORD * random.random()
					y = MAX_COORD * random.random()
					r = (R_MAX - R_MIN) * random.random() + R_MIN
					# Verify we did not pick something over the base station
					while (x_b + r) > x > (x_b - r) and (y_b + r) > y > (y_b - r):
						x = MAX_COORD * random.random()
						y = MAX_COORD * random.random()
						r = (R_MAX - R_MIN) * random.random() + R_MIN
					# Write the results to file
					file.write(f"{x} {y} {r} 1\n")
				# Record BS position
				file.write(f"{x_b} {y_b}\n")
