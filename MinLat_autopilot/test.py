import time
import subprocess as sp
from commands import CollectData
import numpy as np

if __name__ == '__main__':
	# Create thread for comms process
	# | Start comms process, wait for response
	# | If return on comms process was successful, set success-flag
	# |  else, leave success-flag unset
	# | Set complete-flag, rejoin
	# On update calls, do nothing

	# Collect data using NS3
	# child = sp.Popen(["/home/jonathan/Research/Tools/ns-allinone-3.36.1/ns-3.36.1/ns3", "run", "scratch/drone-to-sensor"])
	# child = sp.Popen(["/home/jonathan/Research/Tools/ns-allinone-3.36.1/ns-3.36.1/ns3", "run", "scratch/drone-to-sensor", "--", "--distance=100", "--payload=10000000", "--txpower=5", "--delay=true"])
	# child.communicate()[0]
	# rc = child.returncode

	# print("Done! ", rc)
	stuff = {-1: 0}
	stuff[1] = 10
	stuff[7] = 3

	print(stuff)
	print(stuff[7])

	arr = np.random.normal(9, 5, 1)
	print(arr[0])
