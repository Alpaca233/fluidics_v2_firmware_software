from controller import FluidController
from _def import CMD_SET, MCU_CONSTANTS
from controller import SERIAL_NUMBER_DEBUGGING
import time

fc = FluidController(SERIAL_NUMBER_DEBUGGING, log_measurements=True, debug=True)
fc.begin()
# Get initial measurements
print("START")
fc.send_command(CMD_SET.CLEAR)
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()
# INITIALIZE FLUID SENSORS
print("INIT FLUID")
fc.send_command(CMD_SET.INITIALIZE_BUBBLE_SENSORS)
# Get measurements
tstop = time.time() + 4
while time.time() <= tstop:
    response = fc.get_mcu_status()

# INITIALIZE PRESSURE SENSORS
print("INIT PRESSURE")
fc.send_command(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 0)
fc.send_command(CMD_SET.INITIALIZE_PRESSURE_SENSOR, 1)
# Get measurements
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()

# INITIALIZE FLOW SENSOR
print("INIT FLOWRATE")
fc.send_command(CMD_SET.INITIALIZE_FLOW_SENSOR, 1, MCU_CONSTANTS.SLF3X_WATER, True)
# Get measurements
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()

# INITIALIZE SOLENOID VALVE AND BLINK VALVE 4
# Send command
print("INIT SOLENOID VALVES")
fc.send_command(CMD_SET.INITIALIZE_VALVES)
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()
# Send command
print("INIT P4 HIGH")
fc.send_command(CMD_SET.SET_SOLENOID_VALVE, True, 4)
# Get measurements
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()

# Send command
print("INIT P4 LOW")
fc.send_command(CMD_SET.SET_SOLENOID_VALVE, False, 4)
# Get measurements
tstop = time.time() + 1
while time.time() <= tstop:
    response = fc.get_mcu_status()

# INITIALIZE ROTARY VALVE, MOVE TO 5, MOVE BACK TO 1
# Send command
print("INIT ROTARY VALVES")
fc.send_command(CMD_SET.INITIALIZE_ROTARY, 0, 10)
fc.send_command(CMD_SET.INITIALIZE_ROTARY, 1, 10)
fc.send_command(CMD_SET.SET_ROTARY_VALVE, 1, 5)
# Get measurements
tstop = time.time() + 3
while time.time() <= tstop:
    response = fc.get_mcu_status()

# Send command
fc.send_command(CMD_SET.SET_ROTARY_VALVE, 1, 1)
# Get measurements
tstop = time.time() + 3
while time.time() <= tstop:
    response = fc.get_mcu_status()


# # INITIALIZE DISC PUMP, SET MAX PWR, SET 0 PWR
# # Send command
# print("INIT DISC PUMP")
# fc.send_command(CMD_SET.INITIALIZE_DISC_PUMP, MCU_CONSTANTS.TTP_MAX_PW)
# # Get measurements  
# tstop = time.time() + 1
# while time.time() <= tstop:
#     response = fc.get_mcu_status()
# # Send command
# print("SET MAX PWR")
# fc.send_command(CMD_SET.SET_PUMP_PWR_OPEN_LOOP, MCU_CONSTANTS.TTP_MAX_PW)
# # Get measurements  
# tstop = time.time() + 2
# while time.time() <= tstop:
#     response = fc.get_mcu_status()
# # Send command
# print("SET 0 PWR")
# fc.send_command(CMD_SET.SET_PUMP_PWR_OPEN_LOOP, 0)
# tstop = time.time() + 2
# while time.time() <= tstop:
#     response = fc.get_mcu_status()

# END
print("DONE")