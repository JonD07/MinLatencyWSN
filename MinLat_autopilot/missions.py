import abc
from dronekit import VehicleMode
import threading
import queue
import commands
import subprocess as sb
from collections import deque



def setSimulation(sim):
	global running_sim
	running_sim = sim
	if running_sim:
		import numpy as np


#Methods get the Drone number and the orbit set from text files on the drone in order to prevent
#a lot of overhead when updating scripts.
def getDroneNumber(drone_number = -1):
	if running_sim:
		return 0
	else:
		file1 = open("/home/pi/smallsat-autopilot/ip_map.txt","r+")
		if(drone_number == -1):
			for aline in file1:
				values = aline.split()
				if(values[0] == sb.check_output('hostname -I', shell = True).strip()):
					return values[1].strip()
		else:
			for aline in file1:
				values = aline.split()
				if((len(values) > 1) and (values[1] == str(drone_number))):
					print(values[0].strip())
					return values[0].strip()
		file1.close()
		print("ERROR: Number not found")
		return "ERROR: Number not found"


def getOrbitSet():
	return 2


def getAodv_hop():
	file1 = open("/home/pi/smallsat-autopilot/orbit_set.txt","r+")
	file1.readline()
	return file1.readline().strip()


def pass_vehicle(passed_vehicle):
	global vehicle
	vehicle = passed_vehicle


class Mission(object):
	__metaclass__ = abc.ABCMeta
	terminate = False
	name = "Name not set"
	thread = None
	q = deque()

	@abc.abstractmethod
	def __init__(self):
		pass

	def start(self):
		self.thread = threading.Thread(target=self.update_wrapper, name=self.name)
		self.thread.start()
		if running_sim:
			# Wait here for the thread to re-join
			self.thread.join()

	# Wrapper function for updating mission if not terminated
	def update_wrapper(self):
		self.command.init()
		while not self.terminate:
			if vehicle.mode == VehicleMode('LAND'):
				self.dispose()
			else:
				self.update()

	# Periodically called to check command status/is-done
	def update(self):
		# Verify we haven't gone into land mode
		if vehicle.mode == VehicleMode('LAND'):
			self.dispose()
		# Check current command
		elif self.command.is_done():
			# Current command finished, check if there is another command
			if self.q:
				# Run next command
				self.command = self.q.popleft()
				self.command.init()
			else:
				# deque is empty
				self.dispose()
		# Current command isn't complete, call update on command
		else:
			#  Command not complete, call update
			self.command.update()

	def dispose(self):
		self.terminate = True

class Manual(Mission):
	name = "MANUAL"

	def __init__(self):
		self.is_loiter = False
		self.command_idle = commands.Idle('LOITER')
		self.command_idle.can_idle = False
		self.command = commands.GainAlt(2)
		self.q.append(self.command_idle)

	def update(self):
		if vehicle.channels['3'] > 1500 and not self.is_loiter:
			self.is_loiter = True
			self.command_idle.can_idle = True
		super(Manual, self).update()

class PathTest(Mission):
	name = "PATH_TEST"

	def __init__(self):
		self.command = commands.GainAlt(5)
		self.q.append(commands.WaypointTime(0, 0, 10, 5))
		self.q.append(commands.WaypointTime(0, -10, 10, 10))
		self.q.append(commands.WaypointTime(-5, -10, 10, 5))
		self.q.append(commands.WaypointTime(-5, 0, 10, 10))
		self.q.append(commands.WaypointTime(0, 0, 5, 5))
		self.q.append(commands.WaypointDist(0, -10, 10))
		self.q.append(commands.WaypointDist(-5, -10, 10))
		self.q.append(commands.WaypointDist(-5, 0, 10))
		self.q.append(commands.WaypointDist(0, 0, 5))

class StartPosition(Mission):
	name = "START_POSITION"

	def __init__(self):
		# Get first way-point
		drone_number = str(int(getDroneNumber()) - 1 )
		file1 = open("/home/pi/smallsat-autopilot/Trans_Orbits/Orbits/" + getOrbitSet() + "/orbit_files/sc" + drone_number + "_wp_orbit.txt","r+")
		values = file1.readline().split()
		file1.close()
		# Set first way-point as the current command
		self.command = commands.WaypointDist(float(values[0]), float(values[1]), float(values[2]))

class RunOrbit(Mission):
	name = "RUN_ORBIT"

	def __init__(self):
		# Get all way-point from orbit file
		drone_number = str(int(getDroneNumber()) - 1 )
		file1 = open("/home/pi/smallsat-autopilot/Trans_Orbits/Orbits/" + getOrbitSet() + "/orbit_files/sc" + drone_number + "_wp_orbit.txt","r+")
		for aline in file1:
			values = aline.split()
			self.q.append(commands.WaypointTime(float(values[0]), float(values[1]), float(values[2]), float(values[3])))
		file1.close()
		self.q.append(commands.Idle('LOITER'))
		# Add first way-point as current command
		self.command = self.q.popleft()

class CollectWSNData(Mission):
	name = "WSN_DATA"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2

	def __init__(self):
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Get list of commands from file
		file1 = open("/home/pi/MinLatencyWSN/MinLat_autopilot/Missions/2/drone_0_0.pln","r+")
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				# TODO: Add a normal-distribution for the nodes range
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1])))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Successfully collected data from " + str(self.command.node_ID))
				else:
					print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					# Done collecting data at this point, start recovery
					while self.missed_q:
						print("Added move-collect command")
						# Add moving-collecting from this node to the command queue
						self.q.appendleft(commands.MoveAndCollectData(self.missed_q.pop(), self.mission_alt))
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectData):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass

		super(CollectWSNData, self).update()
