from controller import FluidController
from _def import CMD_SET, MCU_CONSTANTS, COMMAND_STATUS, VALVE_POSITIONS
from controller import SERIAL_NUMBER_DEBUGGING

def wait_for_completion():
    status = fc.get_mcu_status()
    while (status == COMMAND_STATUS.IN_PROGRESS) or (status is None):
        status = fc.get_mcu_status()

fc = FluidController(SERIAL_NUMBER_DEBUGGING, log_measurements=True, debug=False)
fc.begin()
fc.send_command_blocking(CMD_SET.CLEAR)
fc.send_command_blocking(CMD_SET.INITIALIZE_BUBBLE_SENSORS)
fc.send_command_blocking(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 0)
fc.send_command_blocking(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 1)
fc.send_command_blocking(CMD_SET.INITIALIZE_FLOW_SENSOR, 1, MCU_CONSTANTS.SLF3X_WATER, True)
fc.send_command_blocking(CMD_SET.INITIALIZE_VALVES)
fc.send_command_blocking(CMD_SET.INITIALIZE_ROTARY, 0, 10)
fc.send_command_blocking(CMD_SET.INITIALIZE_ROTARY, 1, 10)
fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 0, 1)
fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 1, 1)
fc.send_command_blocking(CMD_SET.INITIALIZE_DISC_PUMP, MCU_CONSTANTS.TTP_MAX_PW)
fc.send_command_blocking(CMD_SET.SET_PUMP_PWR_OPEN_LOOP, MCU_CONSTANTS.TTP_MAX_PW)
fc.delay(10)
fc.send_command_blocking(CMD_SET.SET_SOLENOID_VALVES, 0b001010)
fc.delay(10)
fc.send_command_blocking(CMD_SET.SET_SOLENOID_VALVES, 0)
fc.delay(10)



# Reset at end
fc.send_command_blocking(CMD_SET.CLEAR)