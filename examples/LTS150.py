###
# Only for these example files to make the import possible. You might need to change this:
import sys
sys.path.insert(0, "..")
###

from Thorlabs.devices import *

port = "COM4"
baud = 115200
dest = 1
source = 0x50
chan_ident = 1

device1 = LTS150(portn=port, baud=baud, dest=dest, source=source,  chan_ident=chan_ident)
device1.initialize()
device1.update()

device1.set_home_vel(100)
device1.set_vel(100)
device1.set_acc(100)

device1.move_home()