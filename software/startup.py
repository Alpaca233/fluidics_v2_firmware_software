from controller import FluidController
from _def import CMD_SET, MCU_CONSTANTS, COMMAND_STATUS, VALVE_POSITIONS
from controller import SERIAL_NUMBER_DEBUGGING

PRESSURE_TEST = 5.0
VACUUM_TEST = -2.0

fc = FluidController(SERIAL_NUMBER_DEBUGGING, log_measurements=True, debug=False)

def initialize_device(fc):
    fc.begin()
    fc.send_command_blocking(CMD_SET.CLEAR)
    fc.send_command_blocking(CMD_SET.INITIALIZE_BUBBLE_SENSORS)
    fc.send_command_blocking(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 0)
    fc.send_command_blocking(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 1)
    fc.send_command_blocking(CMD_SET.INITIALIZE_FLOW_SENSOR, 1, MCU_CONSTANTS.SLF3X_WATER, True)
    fc.send_command_blocking(CMD_SET.INITIALIZE_VALVES)
    fc.send_command_blocking(CMD_SET.INITIALIZE_ROTARY, 0, 10)
    fc.send_command_blocking(CMD_SET.INITIALIZE_ROTARY, 1, 10)

    # Take 2 tries to init the disc pump
    result = fc.send_command_blocking(CMD_SET.INITIALIZE_DISC_PUMP, MCU_CONSTANTS.TTP_MAX_PW)
    if result == COMMAND_STATUS.CMD_EXECUTION_ERROR:
        result = fc.send_command_blocking(CMD_SET.INITIALIZE_DISC_PUMP, MCU_CONSTANTS.TTP_MAX_PW)
    if result == COMMAND_STATUS.CMD_EXECUTION_ERROR:
        print("Disc pump failed to initialize")
    return

def pressure_vacuum_test(fc):
    # PRESSURE AND VACUUM TEST
    fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 0, 2)
    fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 1, 2)
    fc.send_command_blocking(CMD_SET.SET_PUMP_PWR_OPEN_LOOP, MCU_CONSTANTS.TTP_MAX_PW)
    fc.send_command_blocking(CMD_SET.DELAY_MS, 500)
    fc.send_command_blocking(CMD_SET.SET_SOLENOID_VALVES, VALVE_POSITIONS.TEST_PRESSURE)
    while fc.recorded_data["pressures"][1] < PRESSURE_TEST:
        fc.get_mcu_status()
    fc.send_command_blocking(CMD_SET.SET_SOLENOID_VALVES, VALVE_POSITIONS.TEST_VACUUM)
    while fc.recorded_data["pressures"][0] > VACUUM_TEST:
        fc.get_mcu_status()
    fc.send_command_blocking(CMD_SET.SET_PUMP_PWR_OPEN_LOOP, 0)
    return

initialize_device(fc)
# OPEN LOOP FLUID WITHDRAWAL
fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 0, 2)
print("Clearing lines")
fc.send_command_blocking(CMD_SET.CLEAR_LINES, int((2/3) * MCU_CONSTANTS.TTP_MAX_PW), 3000, 35000)
fc.send_command_blocking(CMD_SET.SET_SOLENOID_VALVES, VALVE_POSITIONS.FLUID_STOP_FLOW)
print("Venting vacuum bottle")
fc.send_command_blocking(CMD_SET.VENT_VB0, -0.7, 20001)
print("Starting load")
fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 0, 1)
fc.send_command_blocking(CMD_SET.LOAD_FLUID_TO_SENSOR, int((1/4) * MCU_CONSTANTS.TTP_MAX_PW), 30000)
print("Vent vacuum bottle")
fc.send_command_blocking(CMD_SET.VENT_VB0, -0.5, 20001)
print("Load 1 uL of fluid open-loop")
fc.send_command_blocking(CMD_SET.LOAD_FLUID_VOLUME, MCU_CONSTANTS.OPEN_LOOP_CTRL, int((1/5) * MCU_CONSTANTS.TTP_MAX_PW), 30000, 1)
fc.send_command_blocking(CMD_SET.DELAY_MS, 100)

# Reset at end
print("resetting")
fc.send_command_blocking(CMD_SET.SET_ROTARY_VALVE, 0, 2)
fc.send_command_blocking(CMD_SET.CLEAR_LINES, int((1/3) * MCU_CONSTANTS.TTP_MAX_PW), 3000, 35000)
fc.send_command_blocking(CMD_SET.CLEAR)