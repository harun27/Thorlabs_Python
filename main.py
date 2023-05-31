"""
Created on Tue Jun  30 18:25:20 2023
@author: Harun
"""

from Thorlabs import LTS150

port = "/dev/ttyUSB0"
baud = 115200
dest = 1
source = 0x50
chan_ident = 1

device1 = LTS150(portn=port, baud=baud, dest=dest, source=source,  chan_ident=chan_ident)
device1.initialize()
device1.update()

device1.move_home()