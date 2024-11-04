import tecancavro
import time
import threading

class SyringePump:
    '''
    SPEED_SEC = {0: 1.25, 1: 1.30, 2: 1.39, 3: 1.52, 4: 1.71, 5: 1.97,
                6: 2.37, 7: 2.77, 8: 3.03, 9: 3.36, 10: 3.77, 11: 4.30,
                12: 5.00, 13: 6.00, 14: 7.50, 15: 10.00, 16: 15.00, 17: 30.00,
                18: 31.58, 19: 33.33, 20: 35.29, 21: 37.50, 22: 40.00, 23: 42.86,
                24: 46.15, 25: 50.00, 26: 54.55, 17: 60.00, 28: 66.67, 29: 75.00,
                30: 85.71, 31: 100.00, 32: 120.00, 33: 150.00, 34: 200.00, 35: 300.00, 36: 333.33,
                37: 375.00, 38: 428.57, 39: 500.00, 40: 600.00}
    '''
    SPEED_SEC_MAPPING = [1.25, 1.30, 1.39, 1.52, 1.71, 1.97, 2.37, 2.77, 3.03, 3.36, 3.77, 
                        4.30, 5.00, 6.00, 7.50, 10.00, 15.00, 30.00, 31.58, 33.33, 35.29,
                        37.50, 40.00, 42.86, 46.15, 50.00, 54.55, 60.00, 66.67, 75.00, 85.71,
                        100.00, 120.00, 150.00, 200.00, 300.00, 333.33, 375.00, 428.57, 500.00, 600.00]
                        # Maps to speed code 0-40

    def __init__(self, syringe_ul, speed_code_limit, waste_port=3, num_ports=4, slope=14):
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
        self.speed_code_limit = speed_code_limit
        self.range = 3000

        self.is_busy = False

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

    def execute(self, block_pump=False):
        self.is_busy = True
        t = self.syringe.executeChain()
        if block_pump:
            self.syringe.waitReady()
        else:
            self.wait_for_stop(t)

    def get_time_to_finish(self):
        return self.syringe.exec_time

    def dispense(self, port, volume, speed_code):
        self.syringe.resetChain()
        self.set_speed(speed_code)
        self.syringe.dispense(port, volume)
        return self.get_time_to_finish()

    def extract(self, port, volume, speed_code):
        self.syringe.resetChain()
        self.set_speed(speed_code)
        self.syringe.extract(port, volume)
        return self.get_time_to_finish()

    def wait_for_stop(self, t=0):
        time.sleep(t)
        while True:
            if self.syringe._checkReady():
                self.is_busy = False
                break
            time.sleep(0.1)

    def get_flow_rate(self, speed_code):
        return round(self.volume * 60 / (self.SPEED_SEC_MAPPING[speed_code] * 1000), 2)

    def flow_rate_to_speed_code(target_flow_rate, syringe_pump):
        """
        Map any flow rate to the closest speed code of the syringe pump
        
        :param flow_rate: ul/min
        :param syringe_pump: SyringePump instance 
        :return: speed code (int)
        """
        target_time = syringe_pump.volume * 60 / target_flow_rate

        left = 0
        right = len(syringe_pump.SPEED_SEC_MAPPING) - 1
        
        # If target is beyond the range, return the closest endpoint
        if target_time <= syringe_pump.SPEED_SEC_MAPPING[self.speed_code_limit]:
            return self.speed_code_limit
        if target_time >= syringe_pump.SPEED_SEC_MAPPING[-1]:
            return len(syringe_pump.SPEED_SEC_MAPPING) - 1
            
        # Binary search
        while left < right:
            if right - left == 1:
                if abs(syringe_pump.SPEED_SEC_MAPPING[left] - target_time) <= abs(syringe_pump.SPEED_SEC_MAPPING[right] - target_time):
                    return left
                return right
                
            mid = (left + right) // 2
            mid_value = syringe_pump.SPEED_SEC_MAPPING[mid]
            
            if mid_value == target_time:
                return mid
            elif mid_value > target_time:
                right = mid
            else:
                left = mid
        

        return left

    def close(self):
        self.plunger_position_updating_event.set()

class SyringePumpSimulation():
    '''
    SPEED_SEC = {0: 1.25, 1: 1.30, 2: 1.39, 3: 1.52, 4: 1.71, 5: 1.97,
                6: 2.37, 7: 2.77, 8: 3.03, 9: 3.36, 10: 3.77, 11: 4.30,
                12: 5.00, 13: 6.00, 14: 7.50, 15: 10.00, 16: 15.00, 17: 30.00,
                18: 31.58, 19: 33.33, 20: 35.29, 21: 37.50, 22: 40.00, 23: 42.86,
                24: 46.15, 25: 50.00, 26: 54.55, 17: 60.00, 28: 66.67, 29: 75.00,
                30: 85.71, 31: 100.00, 32: 120.00, 33: 150.00, 34: 200.00, 35: 300.00, 36: 333.33,
                37: 375.00, 38: 428.57, 39: 500.00, 40: 600.00}
    '''
    SPEED_SEC_MAPPING = [1.25, 1.30, 1.39, 1.52, 1.71, 1.97, 2.37, 2.77, 3.03, 3.36, 3.77, 
                        4.30, 5.00, 6.00, 7.50, 10.00, 15.00, 30.00, 31.58, 33.33, 35.29,
                        37.50, 40.00, 42.86, 46.15, 50.00, 54.55, 60.00, 66.67, 75.00, 85.71,
                        100.00, 120.00, 150.00, 200.00, 300.00, 333.33, 375.00, 428.57, 500.00, 600.00]
                        # Maps to speed code 0-40

    def __init__(self, syringe_ul, speed_code_limit, waste_port, num_ports=4, slope=14):
        self.syringe = None
        self.volume = syringe_ul
        self.range = 3000
        self.is_busy = False

        print("Simulated syringe pump.")

    def get_plunger_position(self):
        return 0.5

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

    def execute(self, block_pump=False):
        self.is_busy = True
        self.wait_for_stop(5)

    def get_time_to_finish(self):
        return 5

    def dispense(self, port, volume, speed_code):
        return 5

    def extract(self, port, volume, speed_code):
        return 5

    def wait_for_stop(self, t=0):
        time.sleep(t)
        self.is_busy = False
        return

    def get_flow_rate(self, speed_code):
        return round(self.volume * 60 / (self.SPEED_SEC_MAPPING[speed_code] * 1000), 2)

    def flow_rate_to_speed_code(target_flow_rate, syringe_pump):
        return 20

    def close(self):
        pass