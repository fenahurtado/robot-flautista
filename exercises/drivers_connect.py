import socket
import sys

import pandas as pd
import sounddevice as sd
from scipy import signal
from librosa import yin, note_to_hz
from scipy.io.wavfile import write

#sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista')

from multiprocessing import Process, Event, Value, Pipe, Array, Manager
import lib.ethernet_ip.ethernetip as ethernetip
from utils.motor_route import *
from exercises.communication import CommunicationCenter
#from utils.driver_fingers import FingersDriver
import struct
import time
import numpy as np
import serial

INPUT_FUNCTION_BITS = {'General Purpose Input': 0, 'CW Limit': 1, 'CCW Limit': 2, 'Start Index Move': 3, 'Capture Encoder Value': 3, 'Stop Jog': 4, 'Stop Registration Move': 4, 'Emergency Stop': 5, 'Home': 6}

class Command:
    def __init__(self, preset_encoder=0, run_assembled_move=0, program_assembled=0, read_assembled_data=0, reset_errors=0, preset_motor_position=0, jog_ccw=0, jog_cw=0, find_home_ccw=0, find_home_cw=0, immediate_stop=0, resume_move=0, hold_move=0, relative_move=0, absolute_move=0, enable_driver=1, virtual_encoder_follower=0, general_purpose_output_state=0, virtual_position_follower=0, backplane_proximity_bit=0, clear_driver_fault=0, assembled_move_type=0, indexed_command=0, registration_move=0, enable_electronic_gearing_mode=0, save_assembled_move=0, reverse_blend_direction=0, hybrid_control_enable=0, encoder_registration_move=0, current_key=0, desired_command_word_2=0, desired_command_word_3=0, desired_command_word_4=0, desired_command_word_5=0, desired_command_word_6=0, desired_command_word_7=0, desired_command_word_8=0, desired_command_word_9=0, name=''):
        self.desired_mode_select_bit = 0
        self.desired_preset_encoder = preset_encoder
        self.desired_run_assembled_move = run_assembled_move
        self.desired_program_assembled = program_assembled
        self.desired_read_assembled_data = read_assembled_data
        self.desired_reset_errors = reset_errors
        self.desired_preset_motor_position = preset_motor_position
        self.desired_jog_ccw = jog_ccw
        self.desired_jog_cw = jog_cw
        self.desired_find_home_ccw = find_home_ccw
        self.desired_find_home_cw = find_home_cw
        self.desired_immediate_stop = immediate_stop
        self.desired_resume_move = resume_move
        self.desired_hold_move = hold_move
        self.desired_relative_move = relative_move
        self.desired_absolute_move = absolute_move

        self.desired_enable_driver = enable_driver
        self.desired_virtual_encoder_follower = virtual_encoder_follower
        self.desired_general_purpose_output_state = general_purpose_output_state
        self.desired_virtual_position_follower = virtual_position_follower
        self.desired_backplane_proximity_bit = backplane_proximity_bit
        self.desired_clear_driver_fault = clear_driver_fault
        self.desired_assembled_move_type = assembled_move_type
        self.desired_indexed_command = indexed_command
        self.desired_registration_move = registration_move
        self.desired_enable_electronic_gearing_mode = enable_electronic_gearing_mode
        self.desired_save_assembled_move = save_assembled_move
        self.desired_reverse_blend_direction = reverse_blend_direction
        self.desired_hybrid_control_enable = hybrid_control_enable
        self.desired_encoder_registration_move = encoder_registration_move
        self.desired_current_key = current_key

        self.desired_command_word_2 = desired_command_word_2        
        self.desired_command_word_3 = desired_command_word_3        
        self.desired_command_word_4 = desired_command_word_4
        self.desired_command_word_5 = desired_command_word_5
        self.desired_command_word_6 = desired_command_word_6
        self.desired_command_word_7 = desired_command_word_7
        self.desired_command_word_8 = desired_command_word_8
        self.desired_command_word_9 = desired_command_word_9

        self.name = name
    
    def get_ints_to_send(self):
        word0 = f'{self.desired_mode_select_bit}{self.desired_preset_encoder}{self.desired_run_assembled_move}{self.desired_read_assembled_data}{self.desired_program_assembled}{self.desired_reset_errors}{self.desired_preset_motor_position}{self.desired_jog_ccw}{self.desired_jog_cw}{self.desired_find_home_ccw}{self.desired_find_home_cw}{self.desired_immediate_stop}{self.desired_resume_move}{self.desired_hold_move}{self.desired_relative_move}{self.desired_absolute_move}'
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'{self.desired_enable_driver}{self.desired_virtual_encoder_follower}{self.desired_general_purpose_output_state}{self.desired_virtual_position_follower}{self.desired_backplane_proximity_bit}{self.desired_clear_driver_fault}{self.desired_assembled_move_type}{self.desired_indexed_command}{self.desired_registration_move}{self.desired_enable_electronic_gearing_mode}{self.desired_save_assembled_move}{self.desired_reverse_blend_direction}{self.desired_hybrid_control_enable}{self.desired_encoder_registration_move}' + format(self.desired_current_key, 'b').zfill(2)
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        return [word0, word1, self.desired_command_word_2, self.desired_command_word_3, self.desired_command_word_4, self.desired_command_word_5, self.desired_command_word_6, self.desired_command_word_7, self.desired_command_word_8, self.desired_command_word_9]

    def get_bytes_to_send(self):
        ints_to_send = self.get_ints_to_send()
        bytes_to_send  = b''
        for i in ints_to_send:
            bytes_to_send += struct.pack("h", i)
        return bytes_to_send

    def get_list_to_send(self):
        bytes_to_send = self.get_bytes_to_send()
        bits_to_send = ''.join(format(byte, '08b')[::-1] for byte in bytes_to_send)
        as_list = [i == '1' for i in bits_to_send]
        return as_list

class Setting:
    def __init__(self, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=50, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5):
        
        self.desired_mode_select_bit = 1
        self.desired_disable_anti_resonance_bit = disable_anti_resonance_bit
        self.desired_enable_stall_detection_bit = enable_stall_detection_bit
        self.desired_use_backplane_proximity_bit = use_backplane_proximity_bit
        self.desired_use_encoder_bit = use_encoder_bit
        self.desired_home_to_encoder_z_pulse = home_to_encoder_z_pulse
        self.desired_input_3_function_bits = input_3_function_bits
        self.desired_input_2_function_bits = input_2_function_bits
        self.desired_input_1_function_bits = input_1_function_bits

        self.desired_output_functionality_bit = output_functionality_bit
        self.desired_output_state_control_on_network_lost = output_state_control_on_network_lost
        self.desired_output_state_on_network_lost = output_state_on_network_lost
        self.desired_read_present_configuration = read_present_configuration
        self.desired_save_configuration = save_configuration
        self.desired_binary_input_format = binary_input_format
        self.desired_binary_output_format = binary_output_format
        self.desired_binary_endian = binary_endian
        self.desired_input_3_active_level = input_3_active_level
        self.desired_input_2_active_level = input_2_active_level
        self.desired_input_1_active_level = input_1_active_level        

        self.desired_starting_speed = starting_speed
        self.desired_motors_step_turn = motors_step_turn
        self.desired_hybrid_control_gain = hybrid_control_gain
        self.desired_encoder_pulses_turn = encoder_pulses_turn
        self.desired_idle_current_percentage = idle_current_percentage
        self.desired_motor_current = motor_current
        self.desired_current_loop_gain = current_loop_gain

        self.name = 'Settings configuration'

    def get_ints_to_send(self):
        word0 = f'{self.desired_mode_select_bit}{self.desired_disable_anti_resonance_bit}{self.desired_enable_stall_detection_bit}0{self.desired_use_backplane_proximity_bit}{self.desired_use_encoder_bit}{self.desired_home_to_encoder_z_pulse}' + format(self.desired_input_3_function_bits, 'b').zfill(3) + format(self.desired_input_2_function_bits, 'b').zfill(3) + format(self.desired_input_1_function_bits, 'b').zfill(3)
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'0{self.desired_output_functionality_bit}{self.desired_output_state_control_on_network_lost}{self.desired_output_state_on_network_lost}{self.desired_read_present_configuration}{self.desired_save_configuration}{self.desired_binary_input_format}{self.desired_binary_output_format}{self.desired_binary_endian}0000{self.desired_input_3_active_level}{self.desired_input_2_active_level}{self.desired_input_1_active_level}'
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        return [word0, word1, self.desired_starting_speed//1000, self.desired_starting_speed%1000, self.desired_motors_step_turn, self.desired_hybrid_control_gain, self.desired_encoder_pulses_turn, self.desired_idle_current_percentage, self.desired_motor_current, self.desired_current_loop_gain]

    def get_bytes_to_send(self):
        ints_to_send = self.get_ints_to_send()
        byte_to_send = b''
        for i in ints_to_send:
            byte_to_send += struct.pack("h", i)
        return byte_to_send

    def get_list_to_send(self):
        bytes_to_send = self.get_bytes_to_send()
        bits_to_send = ''.join(format(byte, '08b')[::-1] for byte in bytes_to_send)
        as_list = [i == '1' for i in bits_to_send]
        return as_list

class VirtualAxis(Process):
    def __init__(self, running, interval, t0, pipe_conn, verbose=False):
        Process.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = [(0,0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.pipe_conn = pipe_conn
        self.verbose = verbose
        self.pos = Value('i', 0)
        self.vel = Value('i', 0)
        
    def run(self):
        # while self.running.is_set():
        #     t = time.time() - self.t0
        #     self.pos = int(200 * np.sin(2*np.pi * self.f * t))
        #     self.vel = int(200 * 2*np.pi*self.f * np.cos(2*np.pi * self.f * t))
        #     time.sleep(0.01)
        while self.running.is_set():
            t = time.time() - self.t0
            self.pos.value, self.vel.value = self.get_ref(t)
            if self.verbose:
                print(t, self.pos.value, self.vel.value)
            self.update_ref(t)
            if self.pipe_conn.poll(self.interval):
                message = self.pipe_conn.recv()
                print("Message received in virtual axis:", message[0])
                if message[0] == "get_ref":
                    pos, vel = self.get_ref(message[1])
                elif message[0] == "update_ref":
                    self.update_ref(message[1])
                elif message[0] == "merge_ref":
                    self.merge_ref(message[1])
                elif message[0] == "stop":
                    self.stop()

    def get_ref(self, t):
        if self.ref[-1][0] > t:
            pos, vel = get_value_from_func_2d(t, self.ref)
        else:
            pos = self.ref[-1][1]
            vel = 0
        return pos, vel

    def update_ref(self, t):
        while self.ref[0][0] < t and len(self.ref) > 1:
            self.ref.pop(0)

    def merge_ref(self, new_ref):
        #print("Merging:", new_ref)
        t_change = new_ref[0][0]
        if self.ref[-1][0] < t_change:
            self.ref += new_ref
        else:
            i = 0
            while self.ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.ref)):
                self.ref.pop()
            self.ref += new_ref
    
    def stop(self):
        self.ref = [(0,self.pos,0)]

class AMCIDriver(Process):
    def __init__(self, hostname, running, musician_pipe, comm_pipe, comm_data, virtual_axis_pipe, t0, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=1, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=40, current_loop_gain=5, homing_slow_speed=200, verbose=False, virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True, virtual_axis_proportional_coef=1, Kp=0, Ki=5, Kd=0.01):
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = None
        self.musician_pipe = musician_pipe
        self.comm_pipe = comm_pipe
        self.comm_data = comm_data
        self.virtual_axis_pipe = virtual_axis_pipe
        self.t0 = t0
        self.connected = connected
        self.acc = virtual_axis_follow_acceleration
        self.dec = virtual_axis_follow_deceleration
        self.virtual_axis_proportional_coef = virtual_axis_proportional_coef
        self.home = home
        self.forced_break = False
        self.motor_current = motor_current
        self.verbose = verbose
        self.init_params()
        self.fast_ccw_limit_homing = False
        self.slow_ccw_limit_homing = False
        self.fast_cw_limit_homing = False
        self.slow_cw_limit_homing = False

        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = 0.01
        self.e0 = 0
        self.MV_I0 = 0
        self.MV = 0
        self.Ka = 0
        self.MV_low = 0
        self.MV_high = 0
        
        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 20*8, 20*8, 20, 20, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 100, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 150])
    
    def init_params(self):
        self.mode_select_bit = 0
        
        self.disable_anti_resonance_bit = 0
        self.enable_stall_detection_bit = 0
        self.use_backplane_proximity_bit = 0
        self.use_encoder_bit = 0
        self.home_to_encoder_z_pulse = 0
        self.input_3_function_bits = 0
        self.input_2_function_bits = 0
        self.input_1_function_bits = 0

        self.output_functionality_bit = 0
        self.output_state_control_on_network_lost = 0
        self.output_state_on_network_lost = 0
        self.read_present_configuration = 0
        self.save_configuration = 0
        self.binary_input_format = 0
        self.binary_output_format = 0
        self.binary_endian = 0
        self.input_3_active_level = 0
        self.input_2_active_level = 0
        self.input_1_active_level = 0

        self.starting_speed = 0
        self.motors_step_turn = 0
        self.hybrid_control_gain = 0
        self.encoder_pulses_turn = 0
        self.idle_current_percentage = 0
        #self.motor_current = 0
        self.current_loop_gain = 0


        self.module_ok = 0
        self.configuration_error = 0
        self.command_error = 0
        self.input_error = 0
        self.position_invalid = 0
        self.waiting_for_assembled_segment = 0
        self.in_assembled_mode = 0
        self.move_complete = 0
        self.decelerating = 0
        self.accelerating = 0
        self.at_home = 0
        self.stopped = 0
        self.in_hold_state = 0
        self.moving_ccw = 0
        self.moving_cw = 0

        self.driver_is_enabled = 0
        self.stall_detected = 0
        self.output_state = 0
        self.heartbeat_bit = 0
        self.limit_condition = 0
        self.invalid_jog_change = 0
        self.motion_lag = 0
        self.driver_fault = 0
        self.connection_was_lost = 0
        self.plc_in_prog_mode = 0
        self.temperature_above_90 = 0
        self.in_3_active = 0
        self.in_2_active = 0
        self.in_1_active = 0

        self.motor_position = Value("i", 0)
        self.encoder_position = Value("i", 0)
        self.pos_ref = Value("i", 0)
        self.captured_encoder_position = 0
        self.programed_motor_current = 0
        self.acceleration_jerk = 0


        self.desired_mode_select_bit = 0
        self.desired_disable_anti_resonance_bit = 0
        self.desired_enable_stall_detection_bit = 0
        self.desired_use_backplane_proximity_bit = 0
        self.desired_use_encoder_bit = 0
        self.desired_home_to_encoder_z_pulse = 0
        self.desired_input_3_function_bits = 0
        self.desired_input_2_function_bits = 0
        self.desired_input_1_function_bits = 0

        self.desired_output_functionality_bit = 0
        self.desired_output_state_control_on_network_lost = 0
        self.desired_output_state_on_network_lost = 0
        self.desired_read_present_configuration = 0
        self.desired_save_configuration = 0
        self.desired_binary_input_format = 0
        self.desired_binary_output_format = 0
        self.desired_binary_endian = 0
        self.desired_input_3_active_level = 0
        self.desired_input_2_active_level = 0
        self.desired_input_1_active_level = 0

        self.desired_starting_speed = 0
        self.desired_motors_step_turn = 0
        self.desired_hybrid_control_gain = 0
        self.desired_encoder_pulses_turn = 0
        self.desired_idle_current_percentage = 0
        self.desired_motor_current = 0
        self.desired_current_loop_gain = 0


        self.desired_preset_encoder = 0
        self.desired_run_assembled_move = 0
        self.desired_program_assembled = 0
        self.desired_read_assembled_data = 0
        self.desired_reset_errors = 0
        self.desired_preset_motor_position = 0
        self.desired_jog_ccw = 0
        self.desired_jog_cw = 0
        self.desired_find_home_ccw = 0
        self.desired_find_home_cw = 0
        self.desired_immediate_stop = 0
        self.desired_resume_move = 0
        self.desired_hold_move = 0
        self.desired_relative_move = 0
        self.desired_absolute_move = 0

        self.desired_enable_driver = 0
        self.desired_virtual_encoder_follower = 0
        self.desired_general_purpose_output_state = 0
        self.desired_virtual_position_follower = 0
        self.desired_backplane_proximity_bit = 0
        self.desired_clear_driver_fault = 0
        self.desired_assembled_move_type = 0
        self.desired_indexed_command = 0
        self.desired_registration_move = 0
        self.desired_enable_electronic_gearing_mode = 0
        self.desired_save_assembled_move = 0
        self.desired_reverse_blend_direction = 0
        self.desired_hybrid_control_enable = 0
        self.desired_encoder_registration_move = 0
        self.desired_current_key = 0

        self.desired_command_word_2 = 0
        self.desired_command_word_3 = 0
        self.desired_command_word_4 = 0
        self.desired_command_word_5 = 0
        self.desired_command_word_6 = 0
        self.desired_command_word_7 = 0
        self.desired_command_word_8 = 0
        self.desired_command_word_9 = 0

    def run(self):
        print("Running amci driver...")
        self.virtual_axis = VirtualAxis(self.running, 0.01, self.t0, self.virtual_axis_pipe, verbose=False)
        self.virtual_axis.start()
        print("Virtual axis started...")
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            time.sleep(0.1)
            
            self.send_data(self.initial_settings.get_bytes_to_send())
            time.sleep(0.1)
            data = self.read_input(explicit=True)
            self.process_incoming_data(data)
            

            return_to_command_mode = self.get_return_to_command_mode_command()
            self.send_data(return_to_command_mode.get_bytes_to_send())
            time.sleep(0.1)

            synchrostep_command = self.get_synchrostep_move_command(0, 0, speed=0, acceleration=self.acc, deceleration=self.dec, proportional_coefficient=self.virtual_axis_proportional_coef, network_delay=0, encoder=False)

            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 100, 150, 110, 50, 50, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH])

            while self.hostname not in " ".join(self.comm_data.keys()):
                pass
            
            stop = self.get_immediate_stop_command()
            self.comm_data[self.hostname + '_out'] = stop.get_list_to_send()
            time.sleep(0.1)

            if self.home:
                if self.initial_settings.desired_input_2_function_bits == INPUT_FUNCTION_BITS['CCW Limit']:
                    print("Buscando CCW")
                    self.ccw_find_home_to_limit()
                    self.fast_ccw_limit_homing = True
                    while self.fast_ccw_limit_homing or self.slow_ccw_limit_homing:
                        if self.verbose:
                            print('Still not homed...')
                        time.sleep(0.5)
                        #break
                        data = self.read_input()
                        self.process_incoming_data(data)
                    time.sleep(0.5)

                    c = self.get_preset_encoder_position_command(-566)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_position_command(-566)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_relative_move_command(566, programmed_speed=1000, acceleration=self.acc, deceleration=self.dec, motor_current=self.motor_current)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)
                    while True:
                        data = self.read_input()
                        self.process_incoming_data(data)
                        if self.move_complete:
                            break

                    c = self.get_reset_errors_command()
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_return_to_command_mode_command()
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    if self.verbose:
                        print('Homed')

                elif self.initial_settings.desired_input_2_function_bits == INPUT_FUNCTION_BITS['CW Limit']:
                    print("Buscando CW")
                    self.cw_find_home_to_limit()
                    self.fast_cw_limit_homing = True
                    while self.fast_cw_limit_homing or self.slow_cw_limit_homing:
                        if self.verbose:
                            print('Still not homed...')
                        time.sleep(0.5)
                        #break
                        data = self.read_input()
                        self.process_incoming_data(data)
                    time.sleep(0.5)
                    c = self.get_relative_move_command(-800, programmed_speed=4000, acceleration=self.acc, deceleration=self.dec, motor_current=self.motor_current)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(2)

                    c = self.get_reset_errors_command()
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_position_command(0)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_encoder_position_command(0)
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_return_to_command_mode_command()
                    self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                    time.sleep(0.1)

                    if self.verbose:
                        print('Homed')
            else:
                c = self.get_preset_position_command(0)
                self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                time.sleep(0.1)

            self.comm_data[self.hostname + '_out'] = synchrostep_command.get_list_to_send()

            while self.running.is_set():
                if self.forced_break:
                    break
                time.sleep(self.Ts)
                data = self.read_input(read_output=False)
                # if self.verbose:
                #     print(data)
                self.process_incoming_data(data)
                self.pos_ref.value = self.virtual_axis.pos.value
                corrected_pos, corrected_vel = self.pid_control(self.virtual_axis.pos.value, self.virtual_axis.vel.value)
                #print(corrected_pos, corrected_vel)
                if type(corrected_pos) == int and type(corrected_vel) == int:
                    try:
                        self.set_output(-corrected_pos, -corrected_vel)
                        #print(corrected_pos, corrected_vel, self.encoder_position, self.motor_position)
                    except:
                        print(f'Error en referencia: {self.virtual_axis.pos.value}, {self.virtual_axis.vel.value}')
            
            self.comm_pipe.send(["stopProduce", self.hostname, 100, 150, 110])

    def pid_control(self, ref_pos, ref_vel):
        SP = ref_pos
        CV = self.encoder_position.value
        e = SP-CV
        MV_P = self.Kp*e
        MV_I = self.MV_I0 + self.Ki*self.Ts*e - self.Ka*self.sat(self.MV,self.MV_low,self.MV_high)
        MV_D = self.Kd*(e-self.e0)/self.Ts
        #print(MV_P, MV_I, MV_D)
        self.MV = int(round(SP + MV_P + MV_I + MV_D, 0))
        self.e0 = e
        self.MV_I0 = MV_I

        return self.MV, ref_vel
    
    def sat(self, x, low, high):
        if x < low:
            return low
        elif x > high:
            return high
        return x
    
    def break_loop(self):
        self.forced_break = True

    def absolute_move(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=5):
        c = self.get_absolute_move_command(target_position, programmed_speed=programmed_speed, acceleration=acceleration, deceleration=deceleration, motor_current=motor_current, acceleration_jerk=acceleration_jerk)
        sen = self.send_data(c.get_bytes_to_send())
        time.sleep(0.1)

    def get_absolute_move_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=5):
        #print(target_position, programmed_speed)
        command = Command(absolute_move=1, name='Absolute Move')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_relative_move_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0):
        command = Command(relative_move=1, name='Relative Move')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def ccw_find_home_to_limit(self):
        self.fast_ccw_limit_homing = True
        ccw_jog = self.get_ccw_jog_command(programmed_speed=400, acceleration=5, motor_current=self.motor_current)
        self.comm_data[self.hostname + '_out'] = ccw_jog.get_list_to_send()
    
    def cw_find_home_to_limit(self):
        self.fast_cw_limit_homing = True
        print(self.motor_current)
        cw_jog = self.get_cw_jog_command(programmed_speed=4000, motor_current=self.motor_current)
        self.comm_data[self.hostname + '_out'] = cw_jog.get_list_to_send()

    def get_reset_errors_command(self):
        command = Command(reset_errors=1, enable_driver=1, clear_driver_fault=1, name='Reset Errors')
        command.desired_command_word_8 = self.motor_current
        return command

    def get_immediate_stop_command(self, motor_current=30):
        command = Command(immediate_stop=1, name='Immediate Stop')
        command.desired_command_word_8 = motor_current
        return command

    def get_ccw_jog_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(jog_ccw=1, name='CCW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_cw_jog_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(jog_cw=1, name='CW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_return_to_command_mode_command(self):
        command = Command(name='Return to Command Mode')
        return command

    def get_preset_position_command(self, position):
        command = Command(preset_motor_position=1, name='Preset Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        command.desired_command_word_8 = self.motor_current
        return command
    
    def get_preset_encoder_position_command(self, position):
        command = Command(preset_encoder=1, name='Preset Encoder Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        return command

    def get_ccw_find_home_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(find_home_ccw=1, name='CCW FFind Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_cw_find_home_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(find_home_cw=1, name='CW FFind Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def read_input(self, read_output=False, explicit=False):
        if not explicit:
            words = []
            for w in range(20):
                words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b = bytearray(20)
            struct.pack_into('20B', b, 0, *words)
        else:
            self.comm_pipe.send(["getAttrSingle", self.hostname, 0x04, 100, 0x03])
            while True:
                response = self.comm_pipe.recv()
                if response[0] == self.hostname:
                    status = response[1]
                    break
            b = status[1]
        
        [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9] = struct.unpack('<10H', b)
        #print(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9)
        
        if read_output:
            words2 = []
            for w in range(20):
                words2.append(int("".join(["1" if self.comm_data[self.hostname + '_out'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b2 = bytearray(20)
            struct.pack_into('20B', b2, 0, *words2)
            [o0, o1, o2, o3, o4, o5, o6, o7, o8, o9] = struct.unpack('<10H', b2)
            # if self.verbose:
            #     print(o0, o1, o2, o3, o4, o5, o6, o7, o8, o9)
        
        return [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9]

    def get_program_assembled_command(self):
        command = Command(program_assembled=1, name='Program Assembles')
        return command

    def get_assembled_segment_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0):
        command = Command(read_assembled_data=1, program_assembled=1, name='Write Assembled Segment')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_run_assembled_move_command(self, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0):
        command = Command(run_assembled_move=1, reverse_blend_direction=blend_direction, assembled_move_type=dwell_move, name='Run Assembled Move')
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = dwell_time
        return command

    def program_run_assembled_move(self, steps, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0):
        #self.programming_assembly = True
        c = self.get_program_assembled_command()
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)
        for step in steps:
            c = self.get_assembled_segment_command(target_position=step['pos'], programmed_speed=step['speed'], acceleration=step['acc'], deceleration=step['dec'], acceleration_jerk=step['jerk'], motor_current=motor_current)
            self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
            time.sleep(0.1)
            c = self.get_program_assembled_command()
            self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
            time.sleep(0.1)
        c = self.get_return_to_command_mode_command()
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)
        c = self.get_run_assembled_move_command(motor_current=motor_current, blend_direction=blend_direction, dwell_move=dwell_move, dwell_time=dwell_time)
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)

    def process_incoming_data(self, data):
        #print(data)
        word0 = format(data[0], 'b').zfill(16)
        word1 = format(data[1], 'b').zfill(16)
        mode = int(word0[0], 2)

        if mode != self.mode_select_bit:
            if mode:
                print('Changed to configuration mode')
                #print(data)
                command = self.get_return_to_command_mode_command()
                self.send_data(command.get_bytes_to_send())
                time.sleep(0.01)

        self.mode_select_bit = int(word0[0], 2)
        
        if self.mode_select_bit: ## configuration mode
            self.disable_anti_resonance_bit = int(word0[1], 2)
            self.enable_stall_detection_bit = int(word0[2], 2)
            self.use_backplane_proximity_bit = int(word0[4], 2)
            self.use_encoder_bit = int(word0[5], 2)
            self.home_to_encoder_z_pulse = int(word0[6], 2)
            self.input_3_function_bits = int(word0[7:10], 2)
            self.input_2_function_bits = int(word0[10:13], 2)
            self.input_1_function_bits = int(word0[13:16], 2)

            self.output_functionality_bit = int(word1[1], 2)
            self.output_state_control_on_network_lost = int(word1[2], 2)
            self.output_state_on_network_lost = int(word1[3], 2)
            self.read_present_configuration = int(word1[4], 2)
            self.save_configuration = int(word1[5], 2)
            self.binary_input_format = int(word1[6], 2)
            self.binary_output_format = int(word1[7], 2)
            self.binary_endian = int(word1[8], 2)
            self.input_3_active_level = int(word1[13], 2)
            self.input_2_active_level = int(word1[14], 2)
            self.input_1_active_level = int(word1[15], 2)

            self.starting_speed = data[2]*1000 + data[3]
            self.motors_step_turn = data[4]
            self.hybrid_control_gain = data[5]
            self.encoder_pulses_turn = data[6]
            self.idle_current_percentage = data[7]
            #self.motor_current = data[8]
            self.current_loop_gain = data[9]
        else: ## command mode
            module_ok = int(word0[1], 2)
            # if module_ok != self.module_ok:
            #     self.module_ok_signal.emit(module_ok)
            self.module_ok = module_ok
            configuration_error = int(word0[2], 2)
            if configuration_error and configuration_error != self.configuration_error:
                if self.verbose:
                    print('Configuration Error')
                # self.configuration_error_signal.emit(configuration_error)
            self.configuration_error = configuration_error
            command_error = int(word0[3], 2)
            if command_error and command_error != self.command_error:
                if self.verbose:
                    print('Command Error')
            #     self.command_error_signal.emit(command_error)
            self.command_error = command_error
            input_error = int(word0[4], 2)
            if input_error and input_error != self.input_error:
                if self.verbose:
                    print('Input Error')
                # self.input_error_signal.emit(input_error)
            self.input_error = input_error
            position_invalid = int(word0[5], 2)
            if position_invalid and position_invalid != self.position_invalid:
                if self.verbose:
                    print('Position Invalid')
            #     self.position_invalid_signal.emit(position_invalid)
            self.position_invalid = position_invalid
            waiting_for_assembled_segment = int(word0[6], 2)
            if waiting_for_assembled_segment != self.waiting_for_assembled_segment:
                if self.verbose:
                    print('Waiting for assembled segment changed')
            #     self.waiting_for_assembled_segment_signal.emit(waiting_for_assembled_segment)
            self.waiting_for_assembled_segment = waiting_for_assembled_segment
            in_assembled_mode = int(word0[7], 2)
            if in_assembled_mode != self.in_assembled_mode:
                if self.verbose:
                    print('In assembled mode changed')
            #     self.in_assembled_mode_signal.emit(in_assembled_mode)
            self.in_assembled_mode = in_assembled_mode
            move_complete = int(word0[8], 2)
            # if move_complete != self.move_complete:
            #     self.move_complete_signal.emit(move_complete)
                #self.request_write_return_to_command_mode()
            self.move_complete = move_complete
            decelerating = int(word0[9], 2)
            # if decelerating != self.decelerating:
            #     self.decelerating_signal.emit(decelerating)
            self.decelerating = decelerating
            accelerating = int(word0[10], 2)
            # if accelerating != self.accelerating:
            #     self.accelerating_signal.emit(accelerating)
            self.accelerating = accelerating
            at_home = int(word0[11], 2)
            # if at_home != self.at_home:
            #     self.at_home_signal.emit(at_home)
                # if at_home:
                #     print(11)
                #     self.request_write_configuration(self.actual_settings)
                #     self.request_write_preset_position(0)
            self.at_home = at_home
            stopped = int(word0[12], 2)
            # if stopped != self.stopped:
            #     self.stopped_signal.emit(stopped)
            self.stopped = stopped
            in_hold_state = int(word0[13], 2)
            # if in_hold_state != self.in_hold_state:
            #     self.in_hold_state_signal.emit(in_hold_state)
            self.in_hold_state = in_hold_state
            moving_ccw = int(word0[14], 2)
            # if moving_ccw != self.moving_ccw:
            #     self.moving_ccw_signal.emit(moving_ccw)
            self.moving_ccw = moving_ccw
            moving_cw = int(word0[15], 2)
            # if moving_cw != self.moving_cw:
            #     self.moving_cw_signal.emit(moving_cw)
            self.moving_cw = moving_cw

            driver_is_enabled = int(word1[0], 2)
            # if driver_is_enabled != self.driver_is_enabled:
            #     self.driver_is_enabled_signal.emit(driver_is_enabled)
            self.driver_is_enabled = driver_is_enabled
            stall_detected = int(word1[1], 2)
            if stall_detected and stall_detected != self.stall_detected:
                if self.verbose:
                    print('Stall detected')
            #     self.stall_detected_signal.emit(stall_detected)
            self.stall_detected = stall_detected
            output_state = int(word1[2], 2)
            # if output_state != self.output_state:
            #     self.output_state_signal.emit(output_state)
            self.output_state = output_state
            heartbeat_bit = int(word1[4], 2)
            # if heartbeat_bit != self.heartbeat_bit:
            #     self.heartbeat_bit_signal.emit(heartbeat_bit)
            self.heartbeat_bit = heartbeat_bit
            limit_condition = int(word1[5], 2)
            if limit_condition:
                if self.verbose:
                    print('Limit condition')
                #print('limit condition')
                if self.fast_ccw_limit_homing:
                    if self.verbose:
                        print('was moving fast ccw')
                    # self.request_write_reset_errors()
                    self.slow_ccw_limit_homing = True
                    self.fast_ccw_limit_homing = False
                    steps = [{'pos': 400, 'speed': 400, 'acc': 400, 'dec': 400, 'jerk': 0}, 
                                {'pos': -450, 'speed': 100, 'acc': 400, 'dec': 400, 'jerk': 0}]
                    self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100, motor_current=self.motor_current)

                elif self.slow_ccw_limit_homing:
                    if self.verbose:
                        print('was moving slow ccw')
                    self.slow_ccw_limit_homing = False
                    
                elif self.fast_cw_limit_homing:
                    if self.verbose:
                        print('was moving fast cw')
                    # self.request_write_reset_errors()
                    self.slow_cw_limit_homing = True
                    self.fast_cw_limit_homing = False
                    steps = [{'pos': -4000, 'speed': 2000, 'acc': 400, 'dec': 400, 'jerk': 0}, 
                                {'pos': 4500, 'speed': 800, 'acc': 400, 'dec': 400, 'jerk': 0}]
                    self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100, motor_current=self.motor_current)
                    
                elif self.slow_cw_limit_homing:
                    if self.verbose:
                        print('was moving slow cw')
                    self.slow_cw_limit_homing = False
                    #self.C1.outAssem = c2.get_list_to_send()
                    #time.sleep(0.1)
            # if limit_condition != self.limit_condition:
            #     self.limit_condition_signal.emit(limit_condition)
            self.limit_condition = limit_condition
            invalid_jog_change = int(word1[6], 2)
            if invalid_jog_change != self.invalid_jog_change:
                if self.verbose:
                    print('Invalid jog changed')
            #     self.invalid_jog_change_signal.emit(invalid_jog_change)
            self.invalid_jog_change = invalid_jog_change
            motion_lag = int(word1[7], 2)
            # if motion_lag != self.motion_lag:
            #     self.motion_lag_signal.emit(motion_lag)
            self.motion_lag = motion_lag
            driver_fault = int(word1[8], 2)
            if driver_fault != self.driver_fault:
                if self.verbose:
                    print('Driver fault changed')
            #     self.driver_fault_signal.emit(driver_fault)
            self.driver_fault = driver_fault
            connection_was_lost = int(word1[9], 2)
            if connection_was_lost != self.connection_was_lost:
                if self.verbose:
                    print('Conection was lost')
            #     self.connection_was_lost_signal.emit(connection_was_lost)
            self.connection_was_lost = connection_was_lost
            plc_in_prog_mode = int(word1[10], 2)
            # if plc_in_prog_mode != self.plc_in_prog_mode:
            #     self.plc_in_prog_mode_signal.emit(plc_in_prog_mode)
            self.plc_in_prog_mode = plc_in_prog_mode
            temperature_above_90 = int(word1[11], 2)
            if temperature_above_90 != self.temperature_above_90:
                if self.verbose:
                    print('Temperature above 90 changed')
            #     self.temperature_above_90_signal.emit(temperature_above_90)
            self.temperature_above_90 = temperature_above_90
            in_3_active = int(word1[13], 2)
            # if in_3_active != self.in_3_active:
            #     self.in_3_active_signal.emit(in_3_active)
            self.in_3_active = in_3_active
            in_2_active = int(word1[14], 2)
            # if in_2_active != self.in_2_active:
            #     self.in_2_active_signal.emit(in_2_active)
            self.in_2_active = in_2_active
            in_1_active = int(word1[15], 2)
            # if in_1_active != self.in_1_active:
            #     self.in_1_active_signal.emit(in_1_active)
            self.in_1_active = in_1_active

            pos1 = data[2]
            pos2 = data[3]
            if pos1 > 2**15:
                pos1 = pos1 - 2**16
            if pos2 > 2**15:
                pos2 = pos2 - 2**16
            motor_position = pos1*1000 + pos2
            # if motor_position != self.motor_position:
                # self.motor_position_signal.emit(motor_position)
            self.motor_position.value = -motor_position
            # if self.verbose:
            #     print(self.motor_position)
            pos1 = data[4]
            pos2 = data[5]
            if pos1 > 2**15:
                pos1 = pos1 - 2**16
            if pos2 > 2**15:
                pos2 = pos2 - 2**16
            encoder_position = pos1*1000 + pos2
            # if encoder_position != self.encoder_position:
                # self.encoder_position_signal.emit(encoder_position)
            self.encoder_position.value = -encoder_position
            captured_encoder_position = data[6]*1000 + data[7]
            # if captured_encoder_position != self.captured_encoder_position:
                # self.captured_encoder_position_signal.emit(captured_encoder_position)
            self.captured_encoder_position = captured_encoder_position
            programed_motor_current = data[8]
            # if programed_motor_current != self.programed_motor_current:
                # self.programed_motor_current_signal.emit(programed_motor_current)
            self.programed_motor_current = programed_motor_current
            acceleration_jerk = data[9]
            # if acceleration_jerk != self.acceleration_jerk:
                # self.acceleration_jerk_signal.emit(acceleration_jerk)
            self.acceleration_jerk = acceleration_jerk

    def set_output(self, pos_value, vel_value):
        if pos_value < 0:
            pos_value += 2*(2**31)
        pos_in_bits = "{0:b}".format(pos_value).zfill(32)
        
        if vel_value < 0:
            vel_value += 2*(2**31)
        speed_in_bits = "{0:b}".format(vel_value).zfill(32)

        words = [0, 0, 0, 0]
        words[0] = int(pos_in_bits[16:], 2)
        if words[0] >= 2**15:
            words[0] -= 2**16
        words[1] = int(pos_in_bits[:16], 2)
        if words[1] >= 2**15:
            words[1] -= 2**16
        words[2] = int(speed_in_bits[16:], 2)
        if words[2] >= 2**15:
            words[2] -= 2**16
        words[3] = int(speed_in_bits[:16], 2)
        if words[3] >= 2**15:
            words[3] -= 2**16
        
        b = bytearray(8)
        struct.pack_into('hhhh', b, 0, *words)

        #print(b[0])
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
            
        for i in range(8):
            for j in range(8):
                self.comm_data[self.hostname + '_out'][16*2+i*8+j] = int(words[i][j]) == 1

    def send_data(self, data):
        self.comm_pipe.send(["setAttrSingle", self.hostname, 0x04, 150, 0x03, data])
        #return self.C1.setAttrSingle(0x04, 150, 0x03, data)

    def get_synchrostep_move_command(self, position, direction, speed=200, acceleration=50, deceleration=50, proportional_coefficient=1, network_delay=0, encoder=False):
        command = Command(name='Synchrostep Move')
        if encoder:
            command.desired_virtual_encoder_follower = 1
            command.desired_virtual_position_follower = 0
            #command.desired_encoder_registration_move = 1
            command.desired_hybrid_control_enable = 1
        else:
            command.desired_virtual_encoder_follower = 0
            command.desired_virtual_position_follower = 1
            command.desired_hybrid_control_enable = 0
            command.desired_encoder_registration_move = 0
        if direction:
            command.desired_jog_cw = 1
            command.desired_jog_ccw = 0
        else:
            command.desired_jog_cw = 0
            command.desired_jog_ccw = 1

        if position < 0:
            position += 2*(2**31)
            pos_in_bits = "{0:b}".format(position).zfill(32)
        else:
            pos_in_bits = "{0:b}".format(position).zfill(32)
        
        if speed < 0:
            speed += 2*(2**31)
            speed_in_bits = "{0:b}".format(speed).zfill(32)
        else:
            speed_in_bits = "{0:b}".format(speed).zfill(32)

        command.desired_command_word_2 = int(pos_in_bits[16:], 2)
        if command.desired_command_word_2 >= 2**15:
            command.desired_command_word_2 -= 2**16
        command.desired_command_word_3 = int(pos_in_bits[:16], 2)
        if command.desired_command_word_3 >= 2**15:
            command.desired_command_word_3 -= 2**16
        command.desired_command_word_4 = int(speed_in_bits[16:], 2)
        if command.desired_command_word_4 >= 2**15:
            command.desired_command_word_4 -= 2**16
        command.desired_command_word_5 = int(speed_in_bits[:16], 2)
        if command.desired_command_word_5 >= 2**15:
            command.desired_command_word_5 -= 2**16
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = proportional_coefficient
        command.desired_command_word_9 = network_delay

        return command

class FlowControllerDriver(Process):
    def __init__(self, hostname, running, t0, mus_pipe, comm_pipe, comm_data, axis_pipe, connected=True, verbose=False): # 
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.t0 = t0
        self.mus_pipe = mus_pipe
        self.axis_pipe = axis_pipe
        self.comm_pipe = comm_pipe
        self.comm_data = comm_data
        self.connected = connected
        self.virtual_flow = None

        self.verbose = verbose
        self.mass_flow_reading = Value("d", 0.0)
        self.vol_flow_reading = Value("d", 0.0)
        self.temperature_reading = Value("d", 0.0)
        self.absolute_preasure_reading = Value("d", 0.0)
        self.mass_flow_set_point_reading = Value("d", 0.0)
        
        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 26*8, 4*8, 26, 4, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 101, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 100])

    def run(self):
        self.virtual_flow = VirtualFlow(self.running, 0.01, self.t0, self.axis_pipe, verbose=False)
        self.virtual_flow.start()
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            #time.sleep(0.1)
            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 101, 100, 0x6e, 50, 50, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH])

            while self.hostname not in " ".join(self.comm_data.keys()):
                pass

            while self.running.is_set():
                time.sleep(0.01)
                self.read_input(read_output=False)
                self.set_output(self.virtual_flow.flow.value)
            
            self.comm_pipe.send(["stopProduce", self.hostname, 101, 100, 0x6e])
    
    # def send_ref(self, value):
    #     b = bytearray(4)
    #     struct.pack_into('f', b, 0, value)
        
    #     return self.C1.setAttrSingle(0x04, 100, 0x03, b)
    
    def set_output(self, value):
        value = min(50, max(0, value))
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
        
        l = [0 for i in range(4*8)]
        for i in range(4):
            for j in range(8):
                l[i*8+j] = int(words[i][j]) == 1
        self.comm_data[self.hostname + '_out'] = l

    def read_input(self, read_output=False):
        words = []
        for w in range(26):
            words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(26)
        struct.pack_into('26B', b, 0, *words)
        [g, s, ap, ft, vf, mf, mfsp] = struct.unpack('<HIfffff', b)

        self.mass_flow_reading.value = mf
        self.vol_flow_reading.value = vf
        self.temperature_reading.value = ft
        self.absolute_preasure_reading.value = ap
        self.mass_flow_set_point_reading.value = mfsp
        #print(ft)
        # if s != 0:
        #     print(s)
        if read_output:
            words2 = []
            for w in range(4):
                words2.append(int("".join(["1" if self.comm_data[self.hostname + '_out'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            
            b2 = bytearray(4)
            struct.pack_into('4B', b2, 0, *words2)
            [ref] = struct.unpack('<f', b2)
            print(g, s, ap, ft, vf, mf, mfsp, ref)

    def change_controlled_var(self, value):
        pass

    def change_control_loop(self, value):
        pass

    def change_kp(self, value):
        pass

    def change_ki(self, value):
        pass

    def change_kd(self, value):
        pass

class VirtualFlow(Process):
    def __init__(self, running, interval, t0, pipe_conn, verbose=False):
        Process.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = [(0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.pipe_conn = pipe_conn
        self.verbose = verbose
        self.flow = Value("d", 0.0)
        self.vibrato_amp = Value("d", 0.0)
        self.vibrato_freq = Value("d", 0.0)
        
    def run(self):
        print("Virtual Flow running")
        # f = 1
        # while self.running.is_set():
        #     t = time.time() - self.t0
        #     self.flow = int(15 + 15 * np.sin(2*np.pi * f * t))
        #     time.sleep(0.01)
        while self.running.is_set():
            t = time.time() - self.t0
            self.flow.value = self.get_ref(t)
            if not (type(self.flow.value) == float or type(self.flow.value) == int or type(self.flow.value) == np.float64):
                print(type(self.flow.value), self.flow.value)
            if self.verbose:
                print(t, self.flow.value)
            self.update_ref(t)
            if self.pipe_conn.poll(self.interval):
                message = self.pipe_conn.recv()
                print("Message received in virtual flow:", message[0])
                if message[0] == "get_ref":
                    self.get_ref(message[1])
                elif message[0] == "update_ref":
                    self.update_ref(message[1])
                elif message[0] == "merge_ref":
                    self.vibrato_amp.value = message[2]
                    self.vibrato_freq.value = message[3]
                    self.merge_ref(message[1])
                elif message[0] == "stop":
                    self.stop()

    def get_ref(self, t):
        if self.ref[-1][0] > t:
            ramp = get_value_from_func(t, self.ref, approx=False)
            vibr = ramp * self.vibrato_amp.value * sin(t * 2*pi * self.vibrato_freq.value)
            flow = max(0,min(50, ramp+vibr))
        else:
            ramp = self.ref[-1][1]
            vibr = ramp * self.vibrato_amp.value * sin(t * 2*pi * self.vibrato_freq.value)
            flow = max(0,min(50, ramp+vibr))
        return flow

    def update_ref(self, t):
        while self.ref[0][0] < t and len(self.ref) > 1:
            self.ref.pop(0)

    def merge_ref(self, new_ref):
        t_change = new_ref[0][0]
        if self.ref[-1][0] < t_change:
            self.ref += new_ref
        else:
            i = 0
            while self.ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.ref)):
                self.ref.pop()
            self.ref += new_ref

    def stop(self):
        self.ref = [(0,0)]

class VirtualFingers(Process):
    def __init__(self, running, interval, t0, fingers_driver, pipe_end, verbose=False):
        Process.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = []
        self.t0 = t0
        self.verbose = verbose
        self.note = 'D3'
        self.interval = interval
        self.fingers_driver = fingers_driver
        self.pipe_end = pipe_end
        self.changeEvent = Event()
        
    def run(self):
        self.note_time = 0
        self.next_note_time = 0
        while self.running.is_set():
            if len(self.ref) > 0:
                self.next_note_time, self.note = self.ref.pop(0)
                sleep_time = self.next_note_time - self.note_time
                self.fingers_driver.request_finger_action(self.note)
                if self.verbose:
                    print(sleep_time, self.note)
            else:
                sleep_time = self.interval
            self.changeEvent.wait(timeout=sleep_time)
            self.changeEvent.clear()
            self.note_time = self.next_note_time
    
    def stop(self):
        self.ref = []
        self.changeEvent.set()

class PressureSensor(Process):
    def __init__(self, hostname, running, comm_pipe, comm_data, connected=False, verbose=False):
        Process.__init__(self)
        self.running = running
        self.comm_pipe = comm_pipe
        self.comm_data = comm_data
        self.connected = connected
        self.verbose = verbose
        self.pressure = Value("d", 0.0)
        self.hostname = hostname
        self.verbose = verbose
        
        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 0, 4*8, 10, 4, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 101, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 100])

    def run(self):
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            #time.sleep(0.1)
            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 101, 100, 0x6e, 50, 50, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_LOW])
            
            while self.hostname not in " ".join(self.comm_data.keys()):
                pass

            while self.running.is_set():
                time.sleep(0.01)
                self.read_input()
            
            self.comm_pipe.send(["stopProduce", self.hostname, 101, 100, 0x6e])

    def read_input(self):
        try:
            words = []
            for w in range(10):
                words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b = bytearray(10)
            struct.pack_into('10B', b, 0, *words)
            [g, s, ap] = struct.unpack('<HIf', b)

            self.pressure.value = ap
        except:
            print("Hubo un error en la lectura del input del sensor de presion")

flute_dict = {#'C3':  '00000 0000',
              #'C#3': '00000 0000',
              'D3':  '11110 1110',
              'D#3': '11110 1111',
              'E3':  '11110 1101',
              'F3':  '11110 1001',
              'F#3': '11110 0011',
              'G3':  '11110 0001',
              'G#3': '11111 0001',
              'A3':  '11100 0001',
              'A#3': '11000 1001',
              'B3':  '11000 0001',
              'C4':  '10000 0001',
              'C#4': '00000 0001',
              'D4':  '01110 1110',
              'D#4': '01110 1111',
              'E4':  '11110 1101',
              'F4':  '11110 1001',
              'F#4': '11110 0011',
              'G4':  '11110 0001',
              'G#4': '11111 0001',
              'A4':  '11100 0001',
              'A#4': '11000 1001',
              'B4':  '11000 0001',
              'C5':  '10000 0001',
              'C#5': '00000 0001',
              'D5':  '01110 0001',
              'D#5': '11111 1111',
              'E5':  '11100 1101',
              'F5':  '11010 1001',
              'F#5': '11010 0011',
              'G5':  '10110 0001',
              'G#5': '00111 0001',
              'A5':  '01100 1001',
              #'A#5': '00000 0000',
              #'B5':  '00000 0000',
              'C6':  '10111 1000'}

quena_dict = {'G3':  '00 1111111',
              'G#3': '00 0000000',
              'A3':  '00 1111110',
              'A#3': '00 0000000',
              'B3':  '00 1111100',
              'C4':  '00 1111000',
              'C#4': '00 0000000',
              'D4':  '00 1110000',
              'D#4': '00 0000000',
              'E4':  '00 1100000',
              'F4':  '00 0010000',
              'F#4': '00 1000000',
              'G4':  '00 0000110',
              'G#4': '00 0000000',
              'A4':  '00 1111110',
              'A#4': '00 0000000',
              'B4':  '00 1111100',
              'C5':  '00 1111000',
              'C#5': '00 0000000',
              'D5':  '00 1110000',
              'D#5': '00 0000000',
              'E5':  '00 1100000',
              'F5':  '00 1000000',
              'F#5': '00 0000000',
              'G5':  '00 0000110',
              'G#5': '00 0111111',
              'A5':  '00 0111110',
              'A#5': '00 0000000',
              'B5':  '00 1100010',
              'C6':  '00 1101111',
              'C#6': '00 1101001',
              'D6':  '00 1010000',
              'D#6': '00 0011110',
              'E6':  '00 0000000',
              'F6':  '00 0100000',
              'F#6': '00 1111100'}

test_dict = {'1': '000000001',
             '2': '000000010',
             '3': '000000100',
             '4': '000001000',
             '5': '000010000',
             '6': '000100000',
             '7': '001000000',
             '8': '010000000',
             '9': '100000000',
             '0': '000000000'}

instrument_dicts = {'flute': flute_dict,
                    'quena': quena_dict,
                    'test':  test_dict}

class FingersDriver(Process):
    def __init__(self, host, running, pipe_end, connected=True, instrument='flute', verbose=False):
        # Variables de threading
        Process.__init__(self)
        self.running = running
        self.pipe_end = pipe_end
        self.connected = connected
        self.verbose = verbose
        
        # Variables de músico
        self.instrument = instrument
        self.note_dict = instrument_dicts[instrument]
        self.state = '000000000'

        # Configura evento de cambio
        self.changeEvent = Event()
        self.changeEvent.clear()

        # Configura comunicación serial
        if self.connected:
            self.serial_port = serial.Serial(host, 115200, timeout=1)

    def run(self):
        if self.connected:
            while self.running.is_set():
                self.changeEvent.wait(timeout=1)
                if self.changeEvent.is_set():

                    # Ejecuta acción de dedos de bajo nivel
                    try:
                        self.serial_port.write(self.state)
                    except:
                        print("Arduino disconnected.")
                    # Limpia el flag de cambio
                    self.changeEvent.clear()

            # Finaliza el thread
            self.stop()
            self.serial_port.close()
            print('Fingers Driver thread killed')

    def stop(self):
        # Suelta todas las llaves
        self.serial_port.write(b'\0\0')

    def request_finger_action(self, req_note: str):
        """
        Función para llamar desde FingersController
        :param req_note: string indicando la nota que se desea.
        """

        # Modifica el estado de servos interno según un diccionario
        if req_note in instrument_dicts[self.instrument].keys():
            servo = translate_fingers_to_servo(instrument_dicts[self.instrument][req_note])
            self.state = int(servo.replace(' ', ''), 2).to_bytes(2, byteorder='big')

            # Levanta el flag para generar un cambio en el Thread principal
            self.changeEvent.set()
        else:
            print(f'Key error: {req_note} not in dict')

def translate_fingers_to_servo(note_bits):
    """
    Intercambia las llaves 4 y 5 por disposición geométrica.
    - llave nueva 4 <-- llave antigua 5.
    - llave nueva 5 <-- llave antigua 4.
    :param note_bits:
    :return:
    """
    servo_bits = list(note_bits)
    servo_bits[3] = note_bits[4]
    servo_bits[4] = note_bits[3]

    return ''.join(servo_bits)

class Microphone(Process):
    def __init__(self, running, end_pipe, connected=False, device=1, verbose=False):
        Process.__init__(self)
        self.running = running
        self.connected = connected
        self.verbose = verbose
        self.device = device
        self.end_pipe = end_pipe
        self.pitch = Value('d', 0.0)
        self.sr = 44100
        self.max_num_points = int(self.sr*0.1)
        self.last_mic_data = np.array([])
        self.last = []
        self.flt = signal.remez(121, [0, 50, 240, int(self.sr/2)], [0, 1], fs=self.sr)
        self.A = [1] +  [0 for i in range(77-1)]
        fo = 12800
        l  = 0.995
        self.B2  = [1, -2*np.cos(2*np.pi*fo/self.sr), 1]
        self.A2  = [1, -2*l*np.cos(2*np.pi*fo/self.sr), l**2]
        self.saving = False
        self.data = np.array([])
        self.print_i = 0

    def micCallback(self, indata, frames, time, status):
        if status:
            print('Status:', status)
        #senal_filtrada1 = signal.lfilter(self.flt, self.A, indata)
        #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)
        self.last_mic_data = np.hstack((self.last_mic_data, np.transpose(indata)[0]))
        self.last_mic_data = self.last_mic_data[-self.max_num_points:]
        if self.saving:
            self.data = np.hstack((self.data, np.transpose(indata)[0]))

    def start_saving(self):
        #print("Grabando...")
        self.data = np.array([])
        self.saving = True
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        print(self.data.size)
        write(file_name, self.sr, self.data)
        #self.data.to_csv(file_name)

    def run(self):
        if self.connected:
            #print(sd.query_devices())
            with sd.InputStream(samplerate=self.sr, channels=1, callback=self.micCallback, device=self.device, latency='high'):#,  blocksize=300000): #, latency='high'
                while self.running.is_set():
                    if self.end_pipe.poll(0.05):
                        message = self.end_pipe.recv()
                        if message[0] == 'start_saving':
                            self.start_saving()
                        elif message[0] == 'pause_saving':
                            self.pause_saving()
                        elif message[0] == 'resume_saving':
                            self.resume_saving()
                        elif message[0] == 'finish_saving':
                            self.finish_saving(message[1])
                    #pitches, harmonic_rates, argmins, times = compute_yin(self.last_mic_data, self.sr, f0_max=2000)#, w_len=int(len(self.last_mic_data)-1), harmo_thresh=0.1,f0_max=self.sr/2, w_step=int(len(self.last_mic_data)-1)) 
                    #senal_filtrada1 = signal.lfilter(self.flt, self.A, self.last_mic_data)
                    #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)

                    pitches = yin(self.last_mic_data, sr=self.sr, fmin=note_to_hz('C2'), frame_length=4410, fmax=note_to_hz('C7'), trough_threshold=0.00001) #, trough_threshold=0.0001)]
                    # fmax = note_to_hz('C7')
                    #print(pitches[-1])
                    #print(1/(self.last_mic_data.shape[0]*(1/self.sr)), pitches[-1])
                    #compute_yin() NUT = 1
                    # N = 44100*1
                    # T = 1/44100
                    # U = 1
                    #print(pitches, len(self.last_mic_data))
                    self.pitch.value = pitches[-1]
                    #print(self.pitch)
                    #self.last_mic_data = np.array([])
                    #self.callback.doc.add_next_tick_callback(partial(self.callback.update2, self.last))

            
            print("Mic thread killed")

class Musician(Process):
    def __init__(self, host, connections, running, end_pipe, data, interval=0.01, home=True, x_connect=True, z_connect=True, alpha_connect=True, flow_connect=True, fingers_connect=True, pressure_sensor_connect=True, mic_connect=True):
        Process.__init__(self) # Initialize the threading superclass
        self.t0 = time.time()
        self.host = host
        self.running = running
        self.end_pipe = end_pipe
        self.interval = interval
        self.x_connect = x_connect
        self.z_connect = z_connect
        self.alpha_connect = alpha_connect
        self.flow_connect = flow_connect
        self.fingers_connect = fingers_connect
        self.pressure_sensor_connect = pressure_sensor_connect
        self.mic_connect = mic_connect
        self.connections = connections
        self.home = home
        self.instrument = 'flute'
        self.data = data
        
        self.loaded_route_x = []
        self.loaded_route_z = []
        self.loaded_route_alpha = []
        self.loaded_route_flow = []
        self.loaded_route_notes = []
    
    def run(self):
        print("Running musician...")
        
        # self.x_virtual_axis = VirtualAxis(self.running, self.interval, self.t0, x_virtual_axis_end_conn, verbose=False)

        
        # self.z_virtual_axis = VirtualAxis(self.running, self.interval, self.t0, z_virtual_axis_end_conn, verbose=False)

        
        # self.alpha_virtual_axis = VirtualAxis(self.running, self.interval, self.t0, alpha_virtual_axis_end_conn, verbose=False)

        if self.x_connect or self.z_connect or self.alpha_connect or self.flow_connect or self.pressure_sensor_connect:
            communication_connect = True
        else:
            communication_connect = False

        print("Connecting communications...")
        self.comm_event = Event()
        self.comm_event.set()
        self.comm_pipe, comm_pipe2 = Pipe()
        self.communications = CommunicationCenter(self.host, self.comm_event, comm_pipe2, self.data, connect=communication_connect, verbose=True)
        self.communications.start()
        print("Communication started...\nConnecting Drivers...")

        self.x_driver_conn, x_driver_end_conn = Pipe()
        self.x_virtual_axis_conn, x_virtual_axis_end_conn = Pipe()

        self.x_driver = AMCIDriver(self.connections[0], self.running, x_driver_end_conn, self.comm_pipe, self.data, x_virtual_axis_end_conn, self.t0, connected=self.x_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=self.home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=5, Kd=0.01)
        
        self.z_driver_conn, z_driver_end_conn = Pipe()
        self.z_virtual_axis_conn, z_virtual_axis_end_conn = Pipe()

        self.z_driver = AMCIDriver(self.connections[1], self.running, z_driver_end_conn, self.comm_pipe, self.data, z_virtual_axis_end_conn, self.t0, connected=self.z_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=self.home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=5, Kd=0.01)
        
        self.alpha_driver_conn, alpha_driver_end_conn = Pipe()
        self.alpha_virtual_axis_conn, alpha_virtual_axis_end_conn = Pipe()
        
        self.alpha_driver = AMCIDriver(self.connections[2], self.running, alpha_driver_end_conn, self.comm_pipe, self.data, alpha_virtual_axis_end_conn, self.t0, connected=self.alpha_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=self.home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=1, Kd=0.01)

        self.flow_driver_conn, flow_driver_end_conn = Pipe()
        self.virtual_flow_conn, virtual_flow_end_conn = Pipe()
        self.flow_driver = FlowControllerDriver(self.connections[3], self.running, self.t0, flow_driver_end_conn, self.comm_pipe, self.data, virtual_flow_end_conn, connected=self.flow_connect, verbose=False)

        self.fingers_driver_conn, fingers_driver_end_conn = Pipe()
        try:
            self.fingers_driver = FingersDriver('COM3', self.running, fingers_driver_end_conn, connected=self.fingers_connect, verbose=False)
        except:
            try:
                self.fingers_driver = FingersDriver('/dev/ttyACM1', self.running, fingers_driver_end_conn, connected=self.fingers_connect, verbose=False)
            except:
                raise Exception('Finger Driver not connected')
        self.virtual_fingers_driver_conn, virtual_fingers_driver_end_conn = Pipe()
        self.virtual_fingers = VirtualFingers(self.running, 0.05, self.t0, self.fingers_driver, virtual_fingers_driver_end_conn, verbose=True)

        self.preasure_sensor_conn, preasure_sensor_end_conn = Pipe()
        self.preasure_sensor = PressureSensor(self.connections[4], self.running, self.comm_pipe, self.data, connected=self.pressure_sensor_connect, verbose=True)
        
        self.mic_conn, mic_end_conn = Pipe()
        self.microphone = Microphone(self.running, mic_end_conn, connected=self.mic_connect, verbose=False)

        print("Drivers created...\nCreating memory...")

        self.memory_conn, memory_end_conn = Pipe()
        self.memory = Memory(self.running, self.x_driver, self.z_driver, self.alpha_driver, self.flow_driver, self.preasure_sensor, self.microphone, memory_end_conn, self.data, windowWidth=200, interval=0.05)

        print("Memory created...\nStarting...")

        self.memory.start()
        #self.virtual_flow.start()
        # self.fingers_driver.start()
        # self.virtual_fingers.start()
        
        self.x_driver.start()
        # self.z_driver.start()
        # self.alpha_driver.start()
        self.flow_driver.start()
        # self.fingers_driver.start()
        self.preasure_sensor.start()

        self.microphone.start()

        self.end_pipe.send(['instances created'])
        print("Pierre started listening...")

        while self.running.is_set():
            if self.end_pipe.poll():
                message = self.end_pipe.recv()
                print("Message recived:", message[0])
                if message[0] == "get_memory_data":
                    #s = self.get_memory_data()
                    #print(s)
                    # s = [self.memory.times, self.memory.radius, self.memory.theta, self.memory.offset, self.memory.x, self.memory.z, self.memory.alpha, self.memory.mouth_pressure, self.memory.mass_flow, self.memory.flow_ref, self.memory.volume_flow, self.memory.temperature, self.memory.frequency, self.memory.x_ref, self.memory.z_ref, self.memory.alpha_ref]
                    #print(s)get_data
                    self.memory_conn.send(["get_data"])
                    data = self.memory_conn.recv()[0]
                    print(data)
                    self.end_pipe.send([data])
                    # d = self.get_memory_data()
                    # self.end_pipe.send([d])
                elif message[0] == "get_ref_state":
                    s = self.get_ref_state()
                    print(s)
                    self.end_pipe.send([s])
                elif message[0] == "execute_fingers_action":
                    self.execute_fingers_action(message[1], through_action=message[2])
                elif message[0] == "move_to":
                    self.move_to(message[1], T=message[2], only_x=message[3], only_z=message[4], only_alpha=message[5], only_flow=message[6])
                elif message[0] == "reset_x_controller":
                    self.reset_x_controller()
                elif message[0] == "reset_z_controller":
                    self.reset_z_controller()
                elif message[0] == "reset_alpha_controller":
                    self.reset_alpha_controller()
                elif  message[0] == "start_loaded_script":
                    self.start_loaded_script()
                elif  message[0] == "execute_score":
                    self.execute_score(message[1])
                elif message[0] == "stop":
                    self.stop()
                    self.memory_conn.send(["stop_recording"])
                elif message[0] == "memory.save_recorded_data":
                    self.memory_conn.send(["save_recorded_data", message[1], message[2]])
                elif message[0] == "flow_driver.change_controlled_var":
                    self.flow_driver_conn.send(["change_controlled_var", message[1]])
                elif message[0] == "flow_driver.change_control_loop":
                    self.flow_driver_conn.send(["change_control_loop", message[1]])
                elif message[0] == "flow_driver.change_kp":
                    self.flow_driver_conn.send(["change_kp", message[1]])
                elif message[0] == "flow_driver.change_ki":
                    self.flow_driver_conn.send(["change_ki", message[1]])
                elif message[0] == "flow_driver.change_kd":
                    self.flow_driver_conn.send(["change_kd", message[1]])
                elif message[0] == "set_instrument":
                    self.set_instrument(message[1])
        
        # if not self.EIP is None:
        #     self.EIP.stopIO()
        time.sleep(0.5)
        self.comm_event.clear()

    def set_instrument(self, instrument):
        self.instrument = instrument

    def move_to(self, desired_state, T=None, only_x=False, only_z=False, only_alpha=False, only_flow=False):
        my_state = State(0, 0, 0, 0)
        my_state.x = encoder_units_to_mm(self.x_driver.encoder_position.value)
        my_state.z = encoder_units_to_mm(self.z_driver.encoder_position.value)
        my_state.alpha = encoder_units_to_angle(self.alpha_driver.encoder_position.value)
        my_state.flow = self.flow_driver.mass_flow_reading.value
        # print(my_state, desired_state)
        route = get_route(my_state, desired_state, T=T, acc=50, dec=50)

        move_t0 = time.time() - self.t0
        route_x = []
        route_z = []
        route_alpha = []
        route_flow = []
        for i in range(len(route['t'])):
            route_x.append((route['t'][i] + move_t0, route['x'][i], route['x_vel'][i]))
            route_z.append((route['t'][i] + move_t0, route['z'][i], route['z_vel'][i]))
            route_alpha.append((route['t'][i] + move_t0, route['alpha'][i], route['alpha_vel'][i]))
            route_flow.append((route['t'][i] + move_t0, round(route['flow'][i], 2)))
        # print(route_flow)
        # print(route_x)
        # print(route_z)
        # print(route_alpha)
        # print(route_flow)
        if not only_z and not only_alpha and not only_flow:
            #self.x_virtual_axis.merge_ref(route_x)
            self.x_virtual_axis_conn.send(["merge_ref", route_x])
        if not only_x and not only_alpha and not only_flow:
            # self.z_virtual_axis.merge_ref(route_z)
            self.z_virtual_axis_conn.send(["merge_ref", route_z])
        if not only_x and not only_z and not only_flow:
            # self.alpha_virtual_axis.merge_ref(route_alpha)
            self.alpha_virtual_axis_conn.send(["merge_ref", route_alpha])
        
        self.virtual_flow_conn.send(["merge_ref", route_flow, desired_state.vibrato_amp, desired_state.vibrato_freq])

        return route['t'][-1]

    def execute_score(self, path, go_back=True):
        my_state = State(0, 0, 0, 0)
        my_state.x = x_units_to_mm(self.x_driver.encoder_position.value)
        my_state.z = z_units_to_mm(self.z_driver.encoder_position.value)
        my_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position.value)
        my_state.flow = self.flow_driver.mass_flow_reading.value

        route = get_route_complete(path, go_back=go_back)
        initial_state = State(0, 0, 0, 0)
        initial_state.x = x_units_to_mm(route['x'][0])
        initial_state.z = z_units_to_mm(route['z'][0])
        initial_state.alpha = alpha_units_to_angle(route['alpha'][0])
        print(initial_state)
        self.move_to(initial_state)

        self.loaded_route_x = []
        self.loaded_route_z = []
        self.loaded_route_alpha = []
        self.loaded_route_flow = []
        self.loaded_route_notes = route['notes']
        self.virtual_fingers.note_time = 0
        self.virtual_fingers.next_note_time = 0
        #print(self.loaded_route_notes)
        for i in range(len(route['t'])):
            self.loaded_route_x.append([route['t'][i], route['x'][i], route['x_vel'][i]])
            self.loaded_route_z.append([route['t'][i], route['z'][i], route['z_vel'][i]])
            self.loaded_route_alpha.append([route['t'][i], route['alpha'][i], route['alpha_vel'][i]])
            self.loaded_route_flow.append([route['t_flow'][i], route['flow'][i]])

        #print(self.loaded_route_flow)

        # self.x_virtual_axis.merge_ref(route_x)
        # self.z_virtual_axis.merge_ref(route_z)
        # self.alpha_virtual_axis.merge_ref(route_alpha)
        # self.virtual_flow.merge_ref(route_flow)

    def start_loaded_script(self):
        x = self.x_driver.motor_position
        z = self.z_driver.motor_position
        alpha = self.alpha_driver.motor_position

        if x - self.loaded_route_x[0][1] > 10 or z - self.loaded_route_z[0][1] > 10 or alpha - self.loaded_route_alpha[0][1] > 50:
            print('Not quite there yet...')
            return

        t_start = time.time() - self.t0
        for i in range(len(self.loaded_route_flow)):
            self.loaded_route_x[i][0] += t_start
            self.loaded_route_z[i][0] += t_start
            self.loaded_route_alpha[i][0] += t_start
            self.loaded_route_flow[i][0] += t_start
        self.x_virtual_axis_conn.send(["merge_ref", self.loaded_route_x])
        self.z_virtual_axis_conn.send(["merge_ref", self.loaded_route_z])
        self.alpha_virtual_axis_conn.send(["merge_ref", self.loaded_route_alpha])
        self.virtual_flow_conn.send(["merge_ref", self.loaded_route_flow, 0, 0])
        self.virtual_fingers.ref = self.loaded_route_notes
        self.virtual_fingers.changeEvent.set()
        self.memory.start_saving()

    def stop(self):
        self.x_virtual_axis_conn.send(["stop"])
        self.z_virtual_axis_conn.send(["stop"])
        self.alpha_virtual_axis_conn.send(["stop"])
        self.virtual_flow_conn.send(["stop"])
        self.virtual_fingers.stop()

    def get_ref_state(self):
        s = State(0,0,0,0)
        s.x = x_units_to_mm(self.x_driver.pos_ref.value)
        s.z = z_units_to_mm(self.z_driver.pos_ref.value)
        s.alpha = alpha_units_to_angle(self.alpha_driver.pos_ref.value)
        s.flow = self.flow_driver.mass_flow_set_point_reading.value
        return s

    def move_to_alpha(self, value):
        self.alpha_driver.homing_move_target = value

    # def reset_x_controller(self):
    #     self.x_driver.break_loop()
    #     time.sleep(0.1)
    #     self.x_driver = AMCIDriver(self.EIP, self.connections[0], self.running, self.x_virtual_axis, connected=self.x_connect, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=100, virtual_axis_follow_deceleration=100, home=self.home)
    #     self.x_driver.start()

    # def reset_z_controller(self):
    #     self.z_driver.break_loop()
    #     time.sleep(0.1)
    #     self.z_driver = AMCIDriver(self.EIP, self.connections[1], self.running, self.z_virtual_axis, connected=self.z_connect, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=100, virtual_axis_follow_deceleration=100, home=self.home)
    #     self.z_driver.start()

    # def reset_alpha_controller(self):
    #     self.alpha_driver.break_loop()
    #     time.sleep(0.1)
    #     self.alpha_driver = AMCIDriver(self.EIP, self.connections[2], self.running, self.alpha_virtual_axis, connected=self.alpha_connect, starting_speed=1, verbose=False, motors_step_turn=10000, virtual_axis_follow_acceleration=1000, virtual_axis_follow_deceleration=1000, home=self.home, virtual_axis_proportional_coef=10)
    #     self.alpha_driver.start()

    def execute_fingers_action(self, action, through_action=True):
        """
        Ejecuta una acción de los dedos
        """
        #print(action['data']['note'])
        if through_action:
            self.fingers_driver.request_finger_action(action['data']['note'])
        else:
            self.fingers_driver.request_finger_action(action)

    def get_instrument(self):
        return self.instrument

    def print_info(self):
        my_state = State(0, 0, 0, 0)
        my_state.x = x_units_to_mm(self.x_driver.motor_position)
        my_state.z = z_units_to_mm(self.z_driver.motor_position)
        my_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
        my_state.flow = self.flow_driver.mass_flow_reading.value
        print(my_state)

class Memory(Process):
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, running, x_driver, z_driver, alpha_driver, flow_controller, pressure_sensor, microphone, pipe_end, data, windowWidth=200, interval=0.05):
        Process.__init__(self) # Initialize the threading superclass
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.flow_controller = flow_controller
        self.pressure_sensor = pressure_sensor
        self.microphone = microphone
        self.pipe_end = pipe_end
        self.windowWidth = windowWidth
        self.ref_state = State(0,0,0,0)
        self.real_state = State(0,0,0,0)

        self.ref_state.x = x_units_to_mm(self.x_driver.pos_ref.value)
        self.ref_state.z = z_units_to_mm(self.z_driver.pos_ref.value)
        self.ref_state.alpha = alpha_units_to_angle(self.alpha_driver.pos_ref.value)
        self.ref_state.flow = self.flow_controller.mass_flow_set_point_reading.value
        self.real_state.x = x_units_to_mm(self.x_driver.encoder_position.value)
        self.real_state.z = z_units_to_mm(self.z_driver.encoder_position.value)
        self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position.value)
        self.real_state.flow = self.flow_controller.mass_flow_reading.value

        self.data = data
        self.data['flow_ref'] = linspace(0,0,200)
        self.data['x_ref'] = linspace(0,0,200)
        self.data['z_ref'] = linspace(0,0,200)
        self.data['alpha_ref'] = linspace(0,0,200)
        self.data['x'] = linspace(0,0,200)
        self.data['z'] = linspace(0,0,200)
        self.data['alpha'] = linspace(0,0,200)
        self.data['radius'] = linspace(0,0,200)
        self.data['theta'] = linspace(0,0,200)
        self.data['offset'] = linspace(0,0,200)
        self.data['radius_ref'] = linspace(0,0,200)
        self.data['theta_ref'] = linspace(0,0,200)
        self.data['offset_ref'] = linspace(0,0,200)
        self.data['mouth_pressure'] = linspace(0,0,200)
        self.data['volume_flow'] = linspace(0,0,200)
        self.data['mass_flow'] = linspace(0,0,200)
        self.data['temperature'] = linspace(0,0,200)
        self.data['frequency'] = linspace(0,0,200)
        self.data['times'] = linspace(0,0,200)
        
        self.t0 = time.time()
        self.first_entry = False
        self.t1 = 0

        self.data_frame = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])

        self.interval = interval
        self.running = running
        # self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self.update)
        # self.timer.start(interval)
        
    # def start(self):
    #     self.timer.start(self.interval)

    def run(self):
        while self.running.is_set():
            #self.flow_ref[:-1] = self.flow_ref[1:]                      # shift data in the temporal mean 1 sample left
            #self.flow_ref[-1] = self.flowController.values['set_point']
            
            self.ref_state.x = x_units_to_mm(self.x_driver.pos_ref.value)
            self.ref_state.z = z_units_to_mm(self.z_driver.pos_ref.value)
            self.ref_state.alpha = alpha_units_to_angle(self.alpha_driver.pos_ref.value)
            self.ref_state.flow = self.flow_controller.mass_flow_set_point_reading.value
            self.real_state.x = encoder_units_to_mm(self.x_driver.encoder_position.value)
            self.real_state.z = z_units_to_mm(self.z_driver.encoder_position.value)
            self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position.value)
            self.real_state.flow = self.flow_controller.mass_flow_reading.value

            
            self.data['x_ref'] = np.hstack([self.data['x_ref'][1:], self.ref_state.x])
            self.data['z_ref'] = np.hstack([self.data['z_ref'][1:], self.ref_state.z])
            self.data['alpha_ref'] = np.hstack([self.data['alpha_ref'][1:], self.ref_state.alpha])
            self.data['flow_ref'] = np.hstack([self.data['flow_ref'][1:], self.flow_controller.mass_flow_set_point_reading.value])
            self.data['x'] = np.hstack([self.data['x'][1:], self.real_state.x])
            self.data['z'] = np.hstack([self.data['z'][1:], self.real_state.z])
            self.data['alpha'] = np.hstack([self.data['alpha'][1:], self.real_state.alpha])
            self.data['volume_flow'] = np.hstack([self.data['volume_flow'][1:], self.flow_controller.vol_flow_reading.value])

            self.data['radius'] = np.hstack([self.data['radius'][1:], self.real_state.r])
            self.data['theta'] = np.hstack([self.data['theta'][1:], self.real_state.theta])
            self.data['offset'] = np.hstack([self.data['offset'][1:], self.real_state.o])
            self.data['radius_ref'] = np.hstack([self.data['radius_ref'][1:], self.ref_state.r])
            self.data['theta_ref'] = np.hstack([self.data['theta_ref'][1:], self.ref_state.theta])
            self.data['offset_ref'] = np.hstack([self.data['offset_ref'][1:], self.ref_state.o])
            self.data['mouth_pressure'] = np.hstack([self.data['mouth_pressure'][1:], self.pressure_sensor.pressure.value])
            self.data['mass_flow'] = np.hstack([self.data['mass_flow'][1:], self.flow_controller.mass_flow_reading.value])
            self.data['temperature'] = np.hstack([self.data['temperature'][1:], self.flow_controller.temperature_reading.value])
            self.data['frequency'] = np.hstack([self.data['frequency'][1:], self.microphone.pitch.value])
            self.data['times'] = np.hstack([self.data['times'][1:], time.time() - self.t0])  
            if self.saving:
                if self.first_entry:
                    self.t1 = self.data['times'][-1]
                    self.first_entry = False
                # t = self.times[-1] - self.t1
                # new_data = pd.DataFrame({'time': [t for i in range(18)], 'signal':['frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'], 'value': [self.x_ref[-1], self.x[-1], self.z_ref[-1], self.z[-1], self.alpha_ref[-1], self.alpha[-1], self.vel_state_x, self.vel_state_z, self.vel_state_alpha]})
                # self.data = self.data.append(new_data, ignore_index = True)

                new_data = pd.DataFrame([[self.times[-1] - self.t1, self.frequency[-1], self.temperature[-1], self.mass_flow[-1], self.volume_flow[-1], self.mouth_pressure[-1], self.offset[-1], self.theta[-1], self.radius[-1], self.offset_ref[-1], self.theta_ref[-1], self.radius_ref[-1], self.alpha[-1], self.z[-1], self.x[-1], self.alpha_ref[-1], self.z_ref[-1], self.x_ref[-1], self.alpha_ref[-1]]], columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
                self.data_frame = pd.concat([self.data_frame, new_data], ignore_index=True)
            
            if self.pipe_end.poll(self.interval):
                message = self.pipe_end.recv()
                print("Message received in memory:", message[0])
                if message[0] == "get_data":
                    self.pipe_end.send([self.data])
                # if message[0] == "get_radius":
                #     self.pipe_end.send([self.radius])
                # if message[0] == "get_theta":
                #     self.pipe_end.send([self.theta])
                # if message[0] == "get_offset":
                #     self.pipe_end.send([self.offset])
                # if message[0] == "get_x":
                #     self.pipe_end.send([self.x])
                # if message[0] == "get_z":
                #     self.pipe_end.send([self.z])
                # if message[0] == "get_alpha":
                #     self.pipe_end.send([self.alpha])
                # if message[0] == "get_mouth_pressure":
                #     self.pipe_end.send([self.mouth_pressure])
                # if message[0] == "get_mass_flow":
                #     self.pipe_end.send([self.mass_flow])
                # if message[0] == "get_flow_ref":
                #     self.pipe_end.send([self.flow_ref])
                # if message[0] == "get_volume_flow":
                #     self.pipe_end.send([self.volume_flow])
                # if message[0] == "get_temperature":
                #     self.pipe_end.send([self.temperature])
                # if message[0] == "get_frequency":
                #     self.pipe_end.send([self.frequency])
                # if message[0] == "get_x_ref":
                #     self.pipe_end.send([self.x_ref])
                # if message[0] == "get_z_ref":
                #     self.pipe_end.send([self.z_ref])
                # if message[0] == "get_alpha_ref":
                #     self.pipe_end.send([self.alpha_ref])
    
    def start_saving(self):
        self.first_entry = True
        self.data_frame = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
        # self.data = pd.DataFrame(columns=['time', 'signal', 'value'])
        self.saving = True
        self.microphone.start_saving()
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        self.data_frame.to_csv(file_name)

    def save_recorded_data(self, filename1, filename2):
        self.finish_saving(filename1)
        self.microphone.finish_saving(filename2)

    def stop_recording(self):
        self.saving = False
        self.microphone.saving = False

if __name__ == "__main__":
    pass

# if __name__ == "__main__":
#     print(sd.query_devices())
#     event = Event()
#     event.set()
#     mic_conn, child_conn = Pipe()
#     mic = Microphone(event, child_conn, connected=True, verbose=True)
#     mic.start()

#     while True:
#         t = input()
#         if t == "q":
#             event.clear()
#             break
#         elif t == "r":
#             mic_conn.send(['start_saving'])
#         elif t == "s":
#             mic_conn.send(['finish_saving', 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista/exercises/data/escala2.wav'])

    # host = "192.168.2.10"
    # connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]
    # event = threading.Event()
    # event.set()

    # t0 = time.time()
    # pierre = Musician(host, connections, event)
    # pierre.start()

    # while True:
    #     t = input()
    #     if t == "q":
    #         event.clear()
    #         break
    #     elif t == "m":
    #         desired_state = State(0,0,0,0)
    #         desired_state.x = 0
    #         desired_state.z = 0
    #         desired_state.alpha = 10
    #         pierre.move_to(desired_state, T=1.5)
    #     elif t == 'i':
    #         pierre.print_info()
    #     else:
    #         # print(C1.inAssem)
    #         # print(C2.inAssem)
    #         pass
    