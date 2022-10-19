import subprocess as sb

#Methods get the Drone number and the orbit set from text files on the drone in order to prevent
#a lot of overhead when updating scripts.
def getDroneNumber():
  file1 = open("/home/pi/smallsat-autopilot/ip_map.txt","r+")
  for aline in file1:
    values = aline.split()
    if(values[0] + " " == sb.getoutput('hostname -I')):
      return values[1]
  file1.close()
  return "ERROR: Number not found"

def getOrbitSet():
  file1 = open("/home/pi/orbit_set.txt","r+")
  return file1.readline()


print(getDroneNumber())
print(getOrbitSet())
