import numpy as np
# from PhidgetDIO import PhidgetDIO
from .PhidgetDIO import PhidgetDIO

class FootPed:
    def __init__(self):
        self.phidget = PhidgetDIO()
        self.pedal_state = []

    def get_pedal_state(self):
        pad_state0 = self.phidget.get_di0()
        pad_state1 = self.phidget.get_di1()
        ai0 = self.phidget.get_ai0()
        if ai0 < 4:
            pad_state2 = 0
        else:
            pad_state2 = 1
        return [pad_state0, pad_state1, pad_state2]


if __name__ == "__main__":
    ped = FootPed()
    import time
    while True:
        print (ped.get_pedal_state())
        time.sleep(0.01)