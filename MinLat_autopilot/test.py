import time
from commands import CollectData

if __name__ == '__main__':
	# Create thread for comms process
	# | Start comms process, wait for response
	# | If return on comms process was successful, set success-flag
	# |  else, leave success-flag unset
	# | Set complete-flag, rejoin
	# On update calls, do nothing

	for i in range(3):
		command = CollectData(i)
		print "Initializing command"
		command.init()

		while not command.is_done():
			command.update()
			time.sleep(0.1)
		
		if command.collection_success():
			print "Successfully collected data"
		else:
			print "Collection Failed!"

	print "Done!"