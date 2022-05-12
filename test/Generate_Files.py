import random

NUM_PLOTS = 3
START_COUNT = 5
END_COUNT = 50
Increment = 5

FILE_PATH = "Experiment2/"

MAX_X = 400
MAX_Y = 400
R_MAX = 50
R_MIN = 20

for n in range(START_COUNT, (END_COUNT + Increment), Increment):
	for i in range(NUM_PLOTS):
		file_name = f"{FILE_PATH}plot_{n}_{i}.txt"
		with open(file_name, 'w') as file:
			file.write(f"{n}\n")
			x_b = MAX_X * random.random()
			y_b = MAX_Y * random.random()
			for l in range(n):
				x = MAX_X * random.random()
				y = MAX_Y * random.random()
				r = (R_MAX - R_MIN) * random.random() + R_MIN
				while (x_b + r) > x > (x_b - r) and (y_b + r) > y > (y_b - r):
					x = MAX_X * random.random()
					y = MAX_Y * random.random()
					r = (R_MAX - R_MIN) * random.random() + R_MIN
				file.write(f"{x} {y} {r} 1\n")
			file.write(f"{x_b} {y_b}\n")

