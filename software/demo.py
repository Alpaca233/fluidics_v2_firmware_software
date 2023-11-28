from controller import *
from _def import *

teensy = Microcontroller('13995310')
ctrler = FluidController(teensy, log_measurements=True)

while True:
    