import time
import missions
import commands
from dronekit import connect, VehicleMode, LocationGlobalRelative

# 
# To run the simulation, open a terminal in ardupilot/ArduCopter and run:
# sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd
# 

## For simulation
#-- Define the function for takeoff
def guided_and_arm():
	print("Set Mode to GUIDED")
	vehicle.mode = VehicleMode("GUIDED")
	print("Arming motors")
	vehicle.armed = True
	while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
		print(" Getting ready to take off ...")
		time.sleep(1)


def init():
	# We are not running the simulation
	missions.setSimulation(True)
	commands.setSimulation(True)

	global state
	global vehicle

	print("Connect to simulation vehicle")
	vehicle = connect('localhost:14550', wait_ready=True)

	# Get some vehicle attributes (state)
	print("Contacted vehicle!")

	# Pass the vehicle to rest of ICCS autopilot
	missions.pass_vehicle(vehicle)
	commands.pass_vehicle(vehicle)

	# Get Vehicle Home location - will be `None` until first set by autopilot
	while not vehicle.home_location:
		cmds = vehicle.commands
		cmds.download()
		cmds.wait_ready()
		if not vehicle.home_location:
			print(" Waiting for home location ...")

	# We have a home location
	print(" Home location: %s" % vehicle.home_location)

	# Program-specific constants and configuration
	vehicle.airspeed = 11 # m/s
	vehicle.groundspeed = 11 # m/s

	state = {
		'mission': None,
		'is_disabled': False
	}


def start_next_mission(mission=None):
	if issubclass(type(state['mission']), missions.Mission):
		print("Terminating mission", state['mission'].name)
		state['mission'].dispose()
		while state['mission'].thread.is_alive():
			time.sleep(0.05)
	state['mission'] = mission
	print("Starting mission", state['mission'].name)
	# For simulation, set GUIDED mode and arm before starting a mission
	guided_and_arm()
	state['mission'].start()


if __name__ == '__main__':
	init()
	# start_next_mission(mission=missions.CollectWSNData())
	start_next_mission(mission=missions.CollectWSNData())
	print("Done!!")
