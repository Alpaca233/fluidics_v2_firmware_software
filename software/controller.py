from cobs import cobs
import serial
from _def import *
import serial.tools.list_ports
from datetime import datetime
import os
from pathlib import Path
import numpy as np
from time import time

SERIAL_NUMBER_DEBUGGING = '13995310'

def print_message(msg):
    '''
    Print message with timestamp prepended
    '''
    print(datetime.now().strftime('%m/%d %H:%M:%S') + ' : '  + msg )
    return
def split_byte(byte_in):
    '''
    Split single byte int two nibbles
    '''
    return ((byte_in >> 4), (byte_in & 0x0F))
def uint_to_bytes(uint, n_bytes):
    '''
    Split unsigned integer into n bytes. Returns error if number of bytes is too small
    '''
    out = []
    n_bytes_req = (np.ceil(np.log2(max(uint, 1))/8))
    assert n_bytes_req <= n_bytes, f"Overflow error, need at least {n_bytes_req} bytes"
    for i in range(n_bytes-1, -1, -1):
        shift_amt = i * 8
        shifted = uint >> shift_amt
        out.append(np.uint8(shifted & 0xFF))
    return out

# Define basic input/output from the microcontroller
class Microcontroller():
    def __init__(self, serial_number, use_cobs = True):
        '''
        Arguments:
            string serial_number: serial number of target microcontroller
            bool use_cobs: set True to enable Consistent Overhead Byte Stuffing encoding, default value True
        '''
        self.serial = None
        self.serial_number = serial_number
        self.use_cobs = use_cobs

        # Parameters for fixed-length messages 
        self.tx_buffer_length = MCU_CMD_LENGTH
        self.rx_buffer_length = MCU_MSG_LENGTH

        self.read_buffer = []
        return
    
    def __del__(self):
        '''
        Close the serial port
        '''
        self.serial.close()
        return
    
    def begin(self):
        '''
        Find a Serial device that matches the serial number and connect to it.
        '''
        self.read_buffer = []
        controller_ports = [ p.device for p in serial.tools.list_ports.comports() if self.serial_number == p.serial_number]
        if not controller_ports:
            raise IOError("No Controller Found")
        self.serial = serial.Serial(controller_ports[0],2000000)
        print_message('Teensy connected')
        return
    
    def send_mcu_command(self, cmd):
        '''
        Format using COBS (for PacketSerial library) if necessary and write
        Arguments:
            uint8[] cmd: list of uint8 or equivalent
        Output:
            Writes data over Serial
        Returns:
            None
        '''
        cmd = bytearray(cmd)
        if self.use_cobs:
            cmd = bytearray(cobs.encode(cmd))
            cmd.append(0)
        
        self.serial.write(cmd)
        return
    
    def read_received_packet_nowait(self):
        '''
        If there is serial data available, return it as an array of bytes.
        If we are using COBS, decode it first
        Arguments:
            None
        Inputs:
            Reads data over Serial
        Returns:
            uint8_t data[] or None: either the decoded data (variable length) or None
        '''
        if self.serial.in_waiting == 0:
            return None
        if self.use_cobs:
            # Read data into read_buffer until we hit an end-of-packet (0x00)
            while self.serial.in_waiting:
                byte_in = ord(self.serial.read())
                if byte_in != 0:
                    self.read_buffer.append(byte_in)
                else:
                    break

            # If the last byte isn't 0, we didn't read a full packet yet
            if byte_in != 0:
                return None
            # If it is 0, we have a full packet to decode. Clear the read buffer
            else:
                output = cobs.decode(bytearray(self.read_buffer))
                self.read_buffer = []

        # If we aren't using COBS, use fixed-length command rx
        else:
            # If we have too many bytes in the buffer, throw out the excess
            buffer_len = self.serial.in_waiting
            if buffer_len > self.rx_buffer_length:
                for _ in range(buffer_len - self.rx_buffer_length):
                    self.serial.read()
            # If we have too few bytes, return None
            elif buffer_len < self.rx_buffer_length:
                return None
            
            # Read the buffer
            output = []
            for _ in range(self.rx_buffer_length):
                output.append(ord(self.serial.read()))
        
        return output
    

class FluidController(Microcontroller):
    def __init__(self, serial_number, use_cobs = True, log_measurements = False, debug = False):
        '''
        Initialize logging and microcontroller connection. This class inherits from Microcontroller
        Arguments:
            String serial_number: serial number of the microcontroller
            bool use_cobs: use Consistent Overhead Byte Stuffing for Serial I/O
            bool log_measurements: save logs to a CSV
            bool debug: print debug info
        '''
        self.log_measurements = log_measurements

        if(self.log_measurements):
            self.measurement_file = open(os.path.join(Path.home(),"Downloads","Fluidic Controller Logged Measurement_" + datetime.now().strftime('%Y-%m-%d %H-%M-%S.%f') + ".csv"), "w+")
            self.counter_measurement_file_flush = 0

        self.cmd_uid = 0
        self.cmd_sent = CMD_SET.CLEAR
        self.timestamp_last_mismatch = None
        self.mcu_state = None
        self.debug = debug

        self.serial_number = serial_number
        self.use_cobs = use_cobs

        return

    def __del__(self):
        '''Close the logfile if it's being used'''
        if self.log_measurements:
            self.measurement_file.close()
        return
    
    def add_uid_to_cmd(self, cmd):
        '''Break cmd_uid into two bytes and overwrite the first two bytes of the command array with the uid'''
        cmd[0] = self.cmd_uid >> 8
        cmd[1] = self.cmd_uid & 0xFF
        return cmd
    
    def get_mcu_status(self):
        '''
        Read a fixed-length packet from the microcontroller. If there is data available, unpack it. If in debug mode, print out the data. If we are aving logs, write to disc
        '''
        msg = self.read_received_packet_nowait()
        if msg is None:
            return None
        assert (len(msg) == MCU_MSG_LENGTH), f"Expected message of len {MCU_CMD_LENGTH}, got len {len(msg)}"

        '''
        #########################################################
        #########   MCU -> Computer message structure   #########
        #########################################################
        byte 0-1    : computer -> MCU CMD counter (UID)
        byte 2      : cmd from host computer (error checking through check sum => no need to transmit back the parameters associated with the command)
        byte 3      : status of the command (see _def.py)
        byte 4      : MCU internal program being executed (see _def.py)
        byte 5      : Bubble sensor state (high and low nibble)
        byte 6-10   : Selector valves status (1,2,3,4,5)
        byte 11-12  : state of valve D1-D16
        byte 13-14  : pump power
        byte 15-16  : pressure sensor 1 reading
        byte 17-18  : pressure sensor 2 reading
        byte 19-20  : pressure sensor 3 reading
        byte 21-22  : pressure sensor 4 reading
        byte 23-24  : flow sensor 1 reading
        byte 25-26  : flow sensor 2 reading
        byte 27     : elapsed time since the start of the last internal program (in seconds)
        byte 28-29  : total volume (ul), range: 0 - 5000

        '''

        MCU_received_command_UID = (msg[0] << 8) + msg[1] 
        MCU_received_command = msg[2]
        MCU_command_execution_status = msg[3]
        MCU_interal_program = msg[4]

        bubble_sensor_1_state, bubble_sensor_2_state = split_byte(msg[5])

        selector_valve_1_pos = msg[6]
        selector_valve_2_pos = msg[7]
        selector_valve_3_pos = msg[8]
        selector_valve_4_pos = msg[9]
        selector_valve_5_pos = msg[10]

        solenoid_valves = np.int16((int(msg[11])<<8) + msg[12])

        measurement_pump_power = float((int(msg[13])<<8)+msg[14])/0xFFFF

        _pressure_1_raw = (int(msg[15])<<8) + msg[16]
        _pressure_2_raw = (int(msg[17])<<8) + msg[18]
        _pressure_3_raw = (int(msg[19])<<8) + msg[20]
        _pressure_4_raw = (int(msg[21])<<8) + msg[22]
        pressure_1 = (_pressure_1_raw - MCU_CONSTANTS._output_min) * (MCU_CONSTANTS._p_max - MCU_CONSTANTS._p_min) / (MCU_CONSTANTS._output_max - MCU_CONSTANTS._output_min) + MCU_CONSTANTS._p_min
        pressure_2 = (_pressure_2_raw - MCU_CONSTANTS._output_min) * (MCU_CONSTANTS._p_max - MCU_CONSTANTS._p_min) / (MCU_CONSTANTS._output_max - MCU_CONSTANTS._output_min) + MCU_CONSTANTS._p_min
        pressure_3 = (_pressure_3_raw - MCU_CONSTANTS._output_min) * (MCU_CONSTANTS._p_max - MCU_CONSTANTS._p_min) / (MCU_CONSTANTS._output_max - MCU_CONSTANTS._output_min) + MCU_CONSTANTS._p_min
        pressure_4 = (_pressure_4_raw - MCU_CONSTANTS._output_min) * (MCU_CONSTANTS._p_max - MCU_CONSTANTS._p_min) / (MCU_CONSTANTS._output_max - MCU_CONSTANTS._output_min) + MCU_CONSTANTS._p_min
        
        flow_1 = float(np.int16((int(msg[23])<<8)+msg[24]))/MCU_CONSTANTS.SCALE_FACTOR_FLOW
        flow_2 = float(np.int16((int(msg[25])<<8)+msg[26]))/MCU_CONSTANTS.SCALE_FACTOR_FLOW

        MCU_CMD_time_elapsed = msg[27]

        vol_ul = (float(np.int16((int(msg[28])<<8)+msg[29]))/0xFFFF)*MCU_CONSTANTS.VOLUME_UL_MAX

        # Write the data to file
        if self.log_measurements or self.debug:
            line = f"""{datetime.now().strftime('%m/%d %H:%M:%S')}, \
                       {MCU_received_command_UID}, \
                       {MCU_received_command}, \
                       {MCU_command_execution_status}, \
                       {MCU_interal_program}, \
                       {bubble_sensor_1_state}, \
                       {bubble_sensor_2_state}, \
                       {MCU_CMD_time_elapsed}, \
                       {selector_valve_1_pos}, \
                       {selector_valve_2_pos}, \
                       {selector_valve_3_pos}, \
                       {selector_valve_4_pos}, \
                       {selector_valve_5_pos}, \
                       {solenoid_valves:>016b}, \
                       {measurement_pump_power:.2f}, \
                       {pressure_1:.2f}, \
                       {pressure_2:.2f}, \
                       {pressure_3:.2f}, \
                       {pressure_4:.2f}, \
                       {flow_1:.2f}, \
                       {flow_2:.2f}, \
                       {vol_ul:.2f}\n"""
            if self.log_measurements:
                self.measurement_file.write(line)
                self.counter_measurement_file_flush += 1
                if self.counter_measurement_file_flush >= 500:
                    self.counter_measurement_file_flush = 0
                    self.measurement_file.flush()
            if self.debug:
                print(line)

        # Check for mismatch between received command and transmitted command
        if (MCU_received_command != self.cmd_sent) or (MCU_received_command_UID != self.cmd_uid):
            if self.timestamp_last_mismatch is None:
                self.timestamp_last_mismatch = time()
            else:
                dt = time() - self.timestamp_last_mismatch
                assert (dt < T_DIFF_COMPUTER_MCU_MISMATCH_FAULT_THRESHOLD_SECONDS), f"Command mismatch for {dt} seconds"
        else:
            self.timestamp_last_mismatch = None

        self.mcu_state = ()
        return MCU_command_execution_status
    
    def send_command(self, command, *args):
        '''
        Commands are formatted as UID, command, parameters (arb. length)
        Parameters are formatted differently depending on the command
        '''
        command_array = [0, 0] # Initialize with two empty cells for UID
        self.add_uid_to_cmd(command_array)
        self.cmd_uid += 1

        command_target = np.uint8(command)
        assert command_target == command, "Command is not uint8"
        command_array.append(command_target)

        if command == CMD_SET.CLEAR:
            self.cmd_uid = 0
            pass
        elif command == CMD_SET.INITIALIZE_DISC_PUMP:
            # For disc pump, we need pwr limit (2 bytes), source, mode, and stream config
            assert len(args) == 1, "Need power limit"
            pwr_lim = np.uint16(args[0])
            assert pwr_lim == args[0], "Power limit not a uint16"

            pwr_lim_hi, pwr_lim_lo = uint_to_bytes(pwr_lim, 2)

            command_array.append(pwr_lim_hi)
            command_array.append(pwr_lim_lo)
            pass
        elif command == CMD_SET.INITIALIZE_PRESSURE_SENSOR:
            # Need index of pressure sensor to init
            assert len(args) == 1, "Need sensor index"
            idx = np.uint8(args[0])
            assert idx == args[0], "Index is not a uint8"
            
            command_array.append(idx)
            pass
        elif command == CMD_SET.INITIALIZE_FLOW_SENSOR:
            # Need index, medium, and do_crc
            assert len(args) == 3, "Need I2C index, liquid medium, and whether we do CRC positional arguments"
            idx = np.uint8(args[0])
            assert idx == args[0], "Index not an int16"
            medium = np.uint8(args[1])
            assert medium in MCU_CONSTANTS.MEDIA, "Medium not recognized, must be MEDIUM_IPA or MEDIUM_WATER"
            do_crc = bool(args[2])
            assert do_crc == args[2], "do_crc setting is not a boolean"

            command_array.append(idx)
            command_array.append(medium)
            command_array.append(do_crc)
            pass
        elif command == CMD_SET.INITIALIZE_BUBBLE_SENSORS:
            # Need no additional info
            pass
        elif command == CMD_SET.INITIALIZE_VALVES:
            # Need no additional info
            pass
        elif command == CMD_SET.INITIALIZE_BANG_BANG_PARAMS:
            # Need loop type, low threshold, high threshold, min out, max out, and timestep
            assert len(args) == 6, "Need loop type, low/high thresholds, min/max outputs, and timestep (ms)"
            
            loop_type = np.uint8(args[0])
            assert loop_type in MCU_CONSTANTS.BB_LOOP_TYPES, "loop type is not a bang-bang type"

            t_lower_intermediate = int((args[1]/MCU_CONSTANTS.SLF3X_MAX_VAL_uL_MIN) * np.iinfo(np.uint16).max)
            t_lower = np.uint16(t_lower_intermediate)
            assert t_lower_intermediate == t_lower, "Error calculating lower bound"
            t_upper_intermediate = int((args[2]/MCU_CONSTANTS.SLF3X_MAX_VAL_uL_MIN) * np.iinfo(np.uint16).max)
            t_upper = np.uint16(t_upper_intermediate)
            assert t_upper_intermediate == t_upper, "Error calculating upper bound"

            o_lower_intermediate = int((args[3]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            o_lower = np.uint16(o_lower_intermediate)
            assert o_lower_intermediate == o_lower, "Error calculating lower output"
            o_upper_intermediate = int((args[4]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            o_upper = np.uint16(o_upper_intermediate)
            assert o_upper_intermediate == o_upper, "Error calculating upper output"

            timestep = np.uint32(args[5])
            assert timestep == args[5], "Timestep is not uint32"

            t_lower_hi, t_lower_lo = uint_to_bytes(t_lower, 2)
            t_upper_hi, t_upper_lo = uint_to_bytes(t_upper, 2)
            o_lower_hi, o_lower_lo = uint_to_bytes(o_lower, 2)
            o_upper_hi, o_upper_lo = uint_to_bytes(o_upper , 2)

            tstep_3, tstep_2, tstep_1, tstep_0 = uint_to_bytes(timestep, 4)

            command_array.append(loop_type)
            command_array.append(t_lower_hi)
            command_array.append(t_lower_lo)
            command_array.append(t_upper_hi)
            command_array.append(t_upper_lo)
            command_array.append(o_lower_hi)
            command_array.append(o_lower_lo)
            command_array.append(o_upper_hi)
            command_array.append(o_upper_lo)
            command_array.append(tstep_3)
            command_array.append(tstep_2)
            command_array.append(tstep_1)
            command_array.append(tstep_0)

            pass
        elif command == CMD_SET.INITIALIZE_PID_PARAMS:
            # Need loop type, Kp, Ki, Kd, intergral winding limit, min/max outputs, and timestep
            assert len(args) == 8, "Need loop type, Kp/Ki/Kd, winding limit, min/max outputs, and timestep (ms)"

            loop_type = np.uint8(args[0])
            assert loop_type in MCU_CONSTANTS.PID_LOOP_TYPES, "loop type is not PID type"

            kp_intermediate = int((args[1]/MCU_CONSTANTS.KP_MAX) * np.iinfo(np.uint16).max)
            kp = np.uint16(kp_intermediate)
            assert kp_intermediate == kp, "Error calculating Kp"

            ki_intermediate = int((args[2]/MCU_CONSTANTS.KI_MAX) * np.iinfo(np.uint16).max)
            ki = np.uint16(ki_intermediate)
            assert ki_intermediate == ki, "Error calculating Ki"

            kd_intermediate = int((args[3]/MCU_CONSTANTS.KD_MAX) * np.iinfo(np.uint16).max)
            kd = np.uint16(kd_intermediate)
            assert kd_intermediate == kd, "Error calculating Kd"

            ilim_intermediate = int((args[4]/MCU_CONSTANTS.ILIM_MAX) * np.iinfo(np.uint16).max)
            ilim = np.uint16(ilim_intermediate)
            assert ilim_intermediate == ilim, "Error calculating integral winding limit"
            

            o_lower_intermediate = int((args[5]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            o_lower = np.uint16(o_lower_intermediate)
            assert o_lower_intermediate == o_lower, "Error calculating lower output"
            o_upper_intermediate = int((args[6]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            o_upper = np.uint16(o_upper_intermediate)
            assert o_upper_intermediate == o_upper, "Error calculating upper output"

            timestep = np.uint32(args[7])
            assert timestep == args[7], "Timestep is not uint32"

            kp_hi, kp_lo = uint_to_bytes(kp, 2)
            ki_hi, ki_lo = uint_to_bytes(ki, 2)
            kd_hi, kd_lo = uint_to_bytes(kd, 2)
            ilim_hi, ilim_lo = uint_to_bytes(ilim, 2)
            o_lower_hi, o_lower_lo = uint_to_bytes(o_lower, 2)
            o_upper_hi, o_upper_lo = uint_to_bytes(o_upper , 2)

            tstep_3, tstep_2, tstep_1, tstep_0 = uint_to_bytes(timestep, 4)

            command_array.append(loop_type)
            command_array.append(kp_hi)
            command_array.append(kp_lo)
            command_array.append(ki_hi)
            command_array.append(ki_lo)
            command_array.append(kd_hi)
            command_array.append(kd_lo)
            command_array.append(ilim_hi)
            command_array.append(ilim_lo)
            command_array.append(o_lower_hi)
            command_array.append(o_lower_lo)
            command_array.append(o_upper_hi)
            command_array.append(o_upper_lo)
            command_array.append(tstep_3)
            command_array.append(tstep_2)
            command_array.append(tstep_1)
            command_array.append(tstep_0)
            pass
        elif command == CMD_SET.SET_SOLENOID_VALVES:
            # For setting all valves, need 16 bit setting
            assert len(args) == 1, "Need setting"
            setting = np.uint16(args[0])
            assert setting == args[0], "Setting not an int16"

            setting_hi, setting_lo = uint_to_bytes(setting, 2)
            command_array.append(setting_hi)
            command_array.append(setting_lo)
            pass
        elif command == CMD_SET.SET_SOLENOID_VALVE:
            # For setting an individual valve, need on/off bool and index
            assert len(args) == 2, "Need on/off bool and index"
            on_off = bool(args[0])
            assert on_off == args[0], "on_off setting is not a boolean"
            idx = np.uint8(args[1])
            assert idx == args[1], "idx is not uint8"

            command_array.append(on_off)
            command_array.append(idx)
            pass
        elif command == CMD_SET.SET_PUMP_PWR_OPEN_LOOP:
            # To set the pump power, we just need the power
            assert len(args) == 1, "Need power (milliwatts)"

            pwr_intermediate = np.uint16((args[0]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            pwr = np.uint16(pwr_intermediate)
            assert pwr_intermediate == pwr, "Error calculating power setting"

            pwr_hi, pwr_lo = uint_to_bytes(pwr, 2)
            command_array.append(pwr_hi)
            command_array.append(pwr_lo)
            pass
        elif command == CMD_SET.INITIALIZE_ROTARY:
            # Initialize rotary valve with index and max positions
            assert len(args) == 2, "Need index and number of positions"
            idx = np.uint8(args[0])
            assert idx == args[0], "index is not uint8"
            n_pos = np.uint8(args[1])
            assert n_pos == args[1], "Number of positions is not uint8"

            command_array.append(idx)
            command_array.append(n_pos)
            pass
        elif command == CMD_SET.SET_ROTARY_VALVE:
            # Set rotary valve position, we need index and target position
            assert len(args) == 2, "Need index and target position"
            idx = np.uint8(args[0])
            assert idx == args[0], "index is not uint8"
            target = np.uint8(args[1])
            assert target == args[1], "target position is not uint8"

            command_array.append(idx)
            command_array.append(target)
            pass
        elif command ==  CMD_SET.BEGIN_CLOSED_LOOP:
            # Begin one of the control loops. We need the type of loop to begin
            assert len(args) == 1, "Need loop type"
            loop_type = np.uint8(args[0])
            assert loop_type in MCU_CONSTANTS.LOOP_TYPES, "loop type not valid"
            
            command_array.append(loop_type)
            pass
        elif command == CMD_SET.STOP_CLOSED_LOOP:
            # Need no additional info
            pass
        elif command == CMD_SET.CLEAR_LINES:
            # Need open loop disc pump power, debounce time (ms), and timeout time(ms)
            assert len(args) == 3, "Need power, debounce time, and timeout time"

            o_power_intermediate = int((args[0]/MCU_CONSTANTS.TTP_MAX_PW) * np.iinfo(np.uint16).max)
            o_power = np.uint16(o_power_intermediate)
            assert o_power_intermediate == o_power, "Error calculating open loop power"
            
            t_debounce = np.uint16(args[1])
            assert t_debounce == args[1], "debounce time is not uint16"

            t_timeout = np.uint16(args[2])
            assert t_timeout == args[2], "timeout time is not uint16"

            pwr_hi, pwr_lo = uint_to_bytes(o_power, 2)
            db_hi, db_lo = uint_to_bytes(t_debounce, 2)
            tt_hi,tt_lo = uint_to_bytes(t_timeout, 2)
            command_array.append(db_hi)
            command_array.append(db_lo)
            command_array.append(tt_hi)
            command_array.append(tt_lo)
            command_array.append(pwr_hi)
            command_array.append(pwr_lo)

        else:
            # If we don't recognize the command, raise an error
            raise Exception("Command not recognized")

        self.send_mcu_command(command_array)