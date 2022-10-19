#Based on https://github.com/emlid/Navio2/blob/master/Python/GPS.py
import Navio.ublox

class ReadGPS:
  def __init__(self):
    
    self.ubl = Navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    self.ubl.configure_poll_port()
    self.ubl.configure_poll(Navio.ublox.CLASS_CFG, Navio.ublox.MSG_CFG_USB)
    #self.ubl.configure_poll(Navio.ublox.CLASS_MON, Navio.ublox.MSG_MON_HW) #Not sure if we should use this line or the previous one, I need to test it.

    self.ubl.configure_port(port=Navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    self.ubl.configure_port(port=Navio.ublox.PORT_USB, inMask=1, outMask=1)
    self.ubl.configure_port(port=Navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    self.ubl.configure_poll_port()
    self.ubl.configure_poll_port(Navio.ublox.PORT_SERIAL1)
    self.ubl.configure_poll_port(Navio.ublox.PORT_SERIAL2)
    self.ubl.configure_poll_port(Navio.ublox.PORT_USB)
    self.ubl.configure_solution_rate(rate_ms=1000)

    self.ubl.set_preferred_dynamic_model(None)
    self.ubl.set_preferred_usePPP(None)

    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_POSLLH, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_PVT, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_STATUS, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_SOL, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_VELNED, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_SVINFO, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_VELECEF, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_POSECEF, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_RXM, Navio.ublox.MSG_RXM_RAW, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_RXM, Navio.ublox.MSG_RXM_SFRB, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_RXM, Navio.ublox.MSG_RXM_SVSI, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_RXM, Navio.ublox.MSG_RXM_ALM, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_RXM, Navio.ublox.MSG_RXM_EPH, 1)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_TIMEGPS, 5)
    self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_CLOCK, 5)
    #self.ubl.configure_message_rate(Navio.ublox.CLASS_NAV, Navio.ublox.MSG_NAV_DGPS, 5)
   
  def singlePrint(self):
    msg = self.ubl.receive_message()
    if msg is None:
        if opts.reopen:
          self.ubl.close()
          self.ubl = Navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
        else:
          print(empty)
          return
    #print(msg.name())
    if msg.name() == "NAV_POSLLH":
        outstr = str(msg).split(",")[1:]
        outstr = "".join(outstr)
        print(outstr)
    if msg.name() == "NAV_STATUS":
        outstr = str(msg).split(",")[1:2]
        outstr = "".join(outstr)
        print(outstr)
        #print(str(msg))
       
  def continuousPrint(self):
    while true :
      singlePrint()
