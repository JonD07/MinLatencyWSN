import Navio.adc
import Navio.util

#Class is designed to only print out the voltage of each adc.
class ReadVoltage:
  
    def __init__ (self):
    #This checks for a running ardupilot instance. This program cannot be run simultaniously to the autopilot, I'm not sure why.
       # navio.util.check_apm() 
        self.adc = Navio.adc.ADC()
  
    def printVoltage(self):
        #From https://github.com/emlid/Navio2/blob/master/Python/ADC.py
        results = [0] * self.adc.channel_count
        s = ''
        for i in range (0, self.adc.channel_count):
            results[i] = self.adc.read(i)
            s += 'A{0}: {1:6.4f}V '.format(i, results[i] / 1000)
        print(s)
    
    
    def measureContinuous(self):
        while(True):
            self.printVoltage()
