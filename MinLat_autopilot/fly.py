from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time
import subprocess as sb

import missions
import commands
#import config_copter


def init():
	# We are not running the simulation
	missions.setSimulation(False)
	commands.setSimulation(False)

	global state
	global vehicle

	print "Initializing vehicle"
	vehicle = connect('tcp:127.0.0.1:14550', wait_ready=True)
	missions.pass_vehicle(vehicle) # Passes vehicle to missions file
	commands.pass_vehicle(vehicle) # Passes vehicle to commands file
	print "Made contact with Navio"

	# Program-specific constants and configuration
	vehicle.airspeed = 11 # m/s
	vehicle.groundspeed = 11 # m/s

	state = {
		'mission': None,
		'channels': vehicle.channels,
		'is_disabled': False
	}

	vehicle.add_attribute_listener('channels', start)


def start(self, name, channels):
	## Check if we have comms with controller
	if channels['3'] < 988:
		## No comms
		# Check conditions to start or resume a mission
		if vehicle.armed:
			if not state['is_disabled']:
				print "Disabling missions..."
				state['mission'].dispose()
				state['is_disabled'] = True
		else:
			if state['mission'] != "PREARM":
				if issubclass(type(state['mission']), missions.Mission):
					state['mission'].dispose()
				print "Waiting to become armable..."
				state['mission'] = "PREARM"
		state['channels'] = channels.copy()
		return
	else:
		if vehicle.armed:
			if state['is_disabled']:
				print "Re-enabling missions..."
				state['is_disabled'] = False

	## Make sure that the controller is turned on, throttle should be >= 999 when turned on
	if state['channels']['3'] >= 988:
		## Select a mission based on switches changing position

		# If Switch A changes
		if abs(channels['5'] - state['channels']['5']) > 100:
			# Go into manual mode (this will launch the drone if on the ground)
			start_next_mission(mission=missions.Manual())

		# If Switch D changes
		elif abs(channels['8'] - state['channels']['8']) > 100:            
			sb.Popen(["/home/pi/adhoc-routing-framework/field_test/main", missions.getAodv_hop(), sb.check_output('hostname -I', shell = True).strip(), missions.getDroneNumber(0), missions.getOrbitSet()])
			print "Calling Orbit"
			print "Ad-hoc in mode "
			print missions.getAodv_hop()
			start_next_mission(mission=missions.RunOrbit())

		# Ready to start mission
		elif not state['mission'] == "READY" and (state['mission'] == None or state['mission'] == "PREARM"): 
			print "Ready for takeoff! Select mission..."
			state['mission'] = "READY"
			
		#If the third switch is flipped, start test mission.
		if abs(channels['7'] - state['channels']['7']) > 100:
			state['mission'].dispose()
			vehicle.mode = VehicleMode('STABILIZE')

	# Hold onto previous channel values
	state['channels'] = channels.copy()


def start_next_mission(mission=None):
	if issubclass(type(state['mission']), missions.Mission):
		print "Terminating mission", state['mission'].name
		state['mission'].dispose()
		while state['mission'].thread.is_alive():
			time.sleep(0.05)
	state['mission'] = mission
	print "Starting mission", state['mission'].name
	state['mission'].start()


if __name__ == '__main__':
	init()
	while True:
		time.sleep(1)
