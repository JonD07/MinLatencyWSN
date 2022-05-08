import random

NUM_PLOTS = 10
NUM_SENSORS = 10
FILE_PATH = "Experiment/"

MAX_X = 20
MAX_Y = 20
R_MAX = 1.5
R_MIN = 0.8

for i in range(NUM_PLOTS):
	file_name = f"{FILE_PATH}plot_{i}.txt"
	with open(file_name, 'w') as file:
		file.write(f"{NUM_SENSORS}\n")
		x_b = MAX_X * random.random()
		y_b = MAX_Y * random.random()
		for l in range(NUM_SENSORS):
			x = MAX_X * random.random()
			y = MAX_Y * random.random()
			r = (R_MAX - R_MIN) * random.random() + R_MIN
			while (x_b + r) > x > (x_b - r) and (y_b + r) > y > (y_b - r):
				x = MAX_X * random.random()
				y = MAX_Y * random.random()
				r = (R_MAX - R_MIN) * random.random() + R_MIN
			file.write(f"{x} {y} {r} 1\n")
		file.write(f"{x_b} {y_b}\n")

