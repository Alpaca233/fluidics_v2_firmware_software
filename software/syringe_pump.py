import tecancavro
import time

class SyringePump:
    SPEED_SEC = {0: 1.25, 1: 1.30, 2: 1.39, 3: 1.52, 4: 1.71, 5: 1.97,
                6: 2.37, 7: 2.77, 8: 3.03, 9: 3.36, 10: 3.77, 11: 4.30,
                12: 5.00, 13: 6.00, 14: 7.50, 15: 10.00, 16: 15.00, 17: 30.00,
                18: 31.58, 19: 33.33, 20: 35.29, 21: 37.50, 22: 40.00, 23: 42.86,
                24: 46.15, 25: 50.00, 26: 54.55, 17: 60.00, 28: 66.67, 29: 75.00,
                30: 85.71, 31: 100.00, 32: 120.00, 33: 150.00, 34: 200.00, 35: 300.00, 36: 333.33,
                37: 375.00, 38: 428.57, 39: 500.00, 40: 600.00}

    def __init__(self, syringe_ul, waste_port=3, num_ports=4, slope=14):
        syringePumpAddr = tecancavro.transport.TecanAPISerial.findSerialPumps()
        print("Found syringe pump: ", syringePumpAddr)
        self.syringe = tecancavro.models.XCaliburD(com_link=tecancavro.TecanAPISerial(tecan_addr=0, ser_port=syringePumpAddr[0][0], ser_baud=9600), 
                            num_ports=num_ports,
                            syringe_ul=syringe_ul, 
                            microstep=False, 
                            waste_port=waste_port, 
                            slope=slope, 
                            debug=False, 
                            debug_log_path='.')
        self.volume = syringe_ul
        self.range = 3000

        print("Syringe pump initiated.")

    def get_plunger_position(self):
        position = self.syringe.getPlungerPos()
        return position / self.range 

    def set_speed(self, speed_code):
        self.syringe.setSpeed(speed_code)

    def set_dispense(self, port, volume):
        self.syringe.dispense(port, volume)

    def set_extract(self, port, volume):
        self.syringe.extract(port, volume)

    def set_wait(self, time_s):
        self.syringe.delayExec(time_s * 1000)

    def reset_chain(self):
        self.syringe.resetChain()

    def execute(self):
        return self.syringe.executeChain()

    def dispense(self, port, volume, speed_code):
        self.set_speed(speed_code)
        self.syringe.dispense(port, volume)
        return self.syringe.executeChain()

    def extract(self, port, volume, speed_code):
        self.set_speed(speed_code)
        self.syringe.extract(port, volume)
        return self.syringe.executeChain()

    def check_ready(self):
        return self.syringe._checkReady()

class SyringePumpSimulation():
    SPEED_SEC = {0: 1.25, 1: 1.30, 2: 1.39, 3: 1.52, 4: 1.71, 5: 1.97,
                6: 2.37, 7: 2.77, 8: 3.03, 9: 3.36, 10: 3.77, 11: 4.30,
                12: 5.00, 13: 6.00, 14: 7.50, 15: 10.00, 16: 15.00, 17: 30.00,
                18: 31.58, 19: 33.33, 20: 35.29, 21: 37.50, 22: 40.00, 23: 42.86,
                24: 46.15, 25: 50.00, 26: 54.55, 17: 60.00, 28: 66.67, 29: 75.00,
                30: 85.71, 31: 100.00, 32: 120.00, 33: 150.00, 34: 200.00, 35: 300.00, 36: 333.33,
                37: 375.00, 38: 428.57, 39: 500.00, 40: 600.00}

    def __init__(self, syringe_ul, waste_port, num_ports=4, slope=14):
        self.syringe = None
        self.volume = syringe_ul
        self.range = 3000

        print("Syringe pump initiated.")

    def get_plunger_position(self):
        return 0.5

    def set_speed(self, speed_code):
        pass

    def set_speed(self, speed_code):
        pass

    def set_dispense(self, port, volume):
        pass

    def set_extract(self, port, volume):
        pass

    def set_wait(self, time_s):
        pass

    def reset_chain(self):
        pass

    def execute(self):
        return 10

    def dispense(self, port, volume, speed_code):
        return 5

    def extract(self, port, volume, speed_code):
        return 5

    def check_ready(self):
        return True