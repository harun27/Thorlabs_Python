###
# Only for these example files to make the import possible. You might need to change this:
import sys
sys.path.insert(0, "..")
###

from Thorlabs.devices import *

port1 = "COM5"
port2 = "COM4"

baud = 115200
dest = 1
source = 0x50
chan_ident = 1

device1 = LTS300(portn=port1, baud=baud, dest=dest, source=source,  chan_ident=chan_ident)
device1.initialize()
device1.update()

device2 = LTS300(portn=port2, baud=baud, dest=dest, source=source,  chan_ident=chan_ident)
device2.initialize()
device2.update()

device1.set_home_vel(100)
device1.set_vel(100)
device1.set_acc(100)

device2.set_home_vel(100)
device2.set_vel(100)
device2.set_acc(100)

device1.move_home()
device2.move_home()