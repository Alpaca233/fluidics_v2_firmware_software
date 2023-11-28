from cobs import cobs
import serial
from _def import *
import serial.tools.list_ports
from datetime import datetime
import os
from pathlib import Path
import numpy as np
from time import time

# Print messages with timestamp
def print_message(msg):
    print(datetime.now().strftime('%m/%d %H:%M:%S') + ' : '  + msg )
    return
# Split a byte into two nibbles
def split_byte(byte_in):
    return ((byte_in >> 4), (byte_in & 0x0F))

# Define basic input/output from the microcontroller
class Microcontroller():
    def __init__(self, serial_number, use_cobs = True):
        self.serial = None
        self.serial_number = serial_number
        self.use_cobs = use_cobs

        # Parameters for fixed-length messages 
        self.tx_buffer_length = MCU_CMD_LENGTH
        self.rx_buffer_length = MCU_MSG_LENGTH

        self.read_buffer = []
        return
    
    def __del__(self):
        self.serial.close()
        return
    
    def begin(self):
         # Find a serial port that matches the serial number
        controller_ports = [ p.device for p in serial.tools.list_ports.comports() if self.serial_number == p.serial_number]
        if not controller_ports:
            raise IOError("No Controller Found")
        self.serial = serial.Serial(controller_ports[0],2000000)
        print_message('Teensy connected')
        return
    
    def send_command(self, cmd):
        # Format using COBS (for PacketSerial library)
        if self.use_cobs:
            cmd = cobs.encode(bytearray(cmd))
        
        self.serial.write(cmd)
        return
    
    def read_received_packet_nowait(self):
        if self.serial.in_waiting == 0:
            return None
        # Read data into read_buffer until we hit an end-of-packet (0x00)
        if self.use_cobs:
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
    

class FluidController():
    def __init__(self, microcontroller, log_measurements = False):
        self.microcontroller = microcontroller
        self.log_measurements = log_measurements

        if(self.log_measurements):
            self.measurement_file = open(os.path.join(Path.home(),"Downloads","Fluidic Controller Logged Measurement_" + datetime.now().strftime('%Y-%m-%d %H-%M-%S.%f') + ".csv"), "w+")
            self.counter_measurement_file_flush = 0

        self.cmd_uid = 0
        self.cmd_sent = CMD_SET.CLEAR
        self.timestamp_last_mismatch = None
        self.mcu_state = None

        return

    def __del__(self):
        if self.log_measurements:
            self.measurement_file.close()
        return
    
    def add_uid_to_cmd(self, cmd):
        cmd[0] = self.cmd_uid >> 8
        cmd[1] = self.cmd_uid & 0xFF
        return cmd
    
    def get_mcu_status(self):
        msg = self.microcontroller.read_received_packet_nowait()
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
        if self.log_measurements:
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
            self.measurement_file.write(line)
            self.counter_measurement_file_flush += 1
            if self.counter_measurement_file_flush >= 500:
                self.counter_measurement_file_flush = 0
                self.measurement_file.flush()

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
        # Commands are formatted as UID, command, parameters (arb. length)
        # Parameters are formatted differently depending on the command
        command_array = [0, 0] # Initialize with two empty cells for UID
        self.add_uid_to_cmd(command_array)
        self.cmd_uid += 1

        cmd = np.uint8(command)
        assert cmd == command, "Command is not uint8"
        command_array.append(cmd)

        if command == CMD_SET.CLEAR:
            self.cmd_uid = 0
            pass
        elif command == CMD_SET.INITIALIZE_DISC_PUMP:
            # For disc pump, we need pwr limit (2 bytes), source, mode, and stream config
            assert len(args) == 4, "Need power limit, source, mode, stream positional arguments"
            pwr_lim = np.int16(args[0])
            assert pwr_lim == args[0], "Power limit not an int16"
            src = np.uint8(args[1])
            assert src == args[1], "Source setting is not a uint8"
            mode = np.uint8(args[2])
            assert mode == args[2], "Mode setting is not a uint8"
            stream = np.uint8(args[3])
            assert stream == args[3], "Stream setting is not a uint8"

            pwr_lim_hi = pwr_lim >> 8
            pwr_lim_lo = pwr_lim & 0xFF
            command_array.append(pwr_lim_hi)
            command_array.append(pwr_lim_lo)
            command_array.append(src) #src
            command_array.append(mode) #mode
            command_array.append(stream) #stream
            pass
        elif command == CMD_SET.INITIALIZE_PRESSURE_SENSORS:
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
            assert medium in MEDIA, "Medium not recognized, must be MEDIUM_IPA or MEDIUM_WATER"
            do_crc = bool(args[2])
            assert do_crc == args[2], "do_crc setting is not a boolean"

            command_array.append(idx)
            command_array.append(medium)
            command_array.append(do_crc)
            pass
        elif command == CMD_SET.INITIALIZE_BUBBLE_SENSORS:
            # Need no additional info
            pass
        elif command == CMD_SET.INITIALIZE_BANG_BANG_PARAMS:
            # Need low threshold, high threshold, min out, max out, and timestep
            assert len(args) == 5, "Need low/high thresholds, min/max outputs, and timestep (ms)"
            timestep = np.uint32(args[4])
            assert timestep == args[4], "Timestep is not uint32"

            t_low = np.uint16((args[0]/MCU_CONSTANTS.SLF3X_MAX_VAL_uL_MIN) * np.iinfo(np.uint16).max)
            t_high= np.uint16((args[1]/MCU_CONSTANTS.SLF3X_MAX_VAL_uL_MIN) * np.iinfo(np.uint16).max)
            pass
        elif command == CMD_SET.INITIALIZE_PID_PARAMS:
            # 
            pass
        else:
            # If we don't recognize the command, raise an error
            raise Exception("Command not recognized")

        # TODO: implement rest of command-sepcific formatting

        self.microcontroller.send_command(command_array)

def list_devices():
    for p in serial.tools.list_ports.comports():
        print(p.__dict__)
        print('\n')