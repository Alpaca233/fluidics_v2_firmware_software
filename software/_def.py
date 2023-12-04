import numpy as np

MCU_CMD_LENGTH = 15
MCU_MSG_LENGTH = 30

MEDIUM_WATER = 0x08
MEDIUM_IPA = 0x15
MEDIA = [MEDIUM_IPA, MEDIUM_WATER]

# MCU - COMPUTER
T_DIFF_COMPUTER_MCU_MISMATCH_FAULT_THRESHOLD_SECONDS = 3

class MCU_CONSTANTS:
    # pressure sensor SSCMRRV015PD2A3
    _output_min = 1638; # 10% of 2^14
    _output_max = 14745; # 90% of 2^14
    _p_min = -15; # psi
    _p_max = 15; # psi
    # SLF3X params
    VOLUME_UL_MAX = 5000
    SCALE_FACTOR_FLOW = 10 # Scale Factor for flow rate measurement, ul/min, SLF3S-0600F
    SLF3X_MAX_VAL_uL_MIN = 3520
    SLF3X_WATER = 0x08
    SLF3X_IPA = 0x15
    # PID params
    KP_MAX =  128
    KI_MAX =  128
    KD_MAX =  128
    ILIM_MAX = np.iinfo(np.uint32).max
    # Disc pump params
    TTP_MAX_PW = 1000

class CMD_SET:
  CLEAR                        = 0
  INITIALIZE_DISC_PUMP         = 1
  INITIALIZE_PRESSURE_SENSOR   = 2
  INITIALIZE_FLOW_SENSOR       = 3
  INITIALIZE_BUBBLE_SENSORS    = 4
  INITIALIZE_VALVES            = 5
  INITIALIZE_ROTARY            = 6
  INITIALIZE_BANG_BANG_PARAMS  = 7
  INITIALIZE_PID_PARAMS        = 8
  SET_SOLENOID_VALVES          = 9
  SET_SOLENOID_VALVE           = 10
  SET_ROTARY_VALVE             = 11
  SET_PUMP_PWR_OPEN_LOOP       = 12
