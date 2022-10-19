from VoltageReader import ReadVoltage
from GPSreader import ReadGPS

gps = ReadGPS()
voltage = ReadVoltage()

print(voltage.measureContinuous())
#print(gps.singlePrint())
