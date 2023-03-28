import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
from socket import timeout
import time
import threading
import random
from functools import partial
from time import sleep
from turtle import speed
from cpppo.server.enip import poll
from cpppo.server.enip.get_attribute import proxy_simple as device
from PyQt5 import QtCore
from numpy import sign
from eeip.eeip.eipclient import *
from utils.motor_route import *
import matplotlib.pyplot as plt

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

class AMCIDriver(threading.Thread):
    def __init__(self, host, running, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5, homing_slow_speed=200, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.connected = connected
        self.init_params()

        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)
        self.actual_settings = self.initial_settings

        self.verbose = verbose
        self.running = running
        self.changeEvent = threading.Event()
        self.explicit_poll = []

        self.eeipclient = EEIPClient()
        #Ip-Address of the Ethernet-IP Device (In this case Allen-Bradley 1734-AENT Point I/O)
        #A Session has to be registered before any communication can be established
        if self.connected:
            self.eeipclient.register_session(host)

        self.write_configuration(self.initial_settings)

        # 0x20, 0x04,0x24, 0x6E, 0x2C, 0x96, 0x2C, 0x64
        #Parameters from Originator -> Target 
        self.eeipclient.configuration_assembly_instance_id = 0x6e

        self.eeipclient.o_t_instance_id = 0x96
        self.eeipclient.o_t_length = 20
        self.eeipclient.o_t_requested_packet_rate = 10000  #Packet rate 100ms (default 500ms)
        self.eeipclient.o_t_realtime_format = RealTimeFormat.HEADER32BIT
        self.eeipclient.o_t_owner_redundant = False
        self.eeipclient.o_t_variable_length = False
        self.eeipclient.o_t_connection_type = ConnectionType.POINT_TO_POINT
        #self.eeipclient.o_t_priority = Priority.HIGH

        #Parameters from Target -> Originator
        self.eeipclient.t_o_instance_id = 0x64
        self.eeipclient.t_o_length = 20
        self.eeipclient.t_o_requested_packet_rate = 10000  #Packet rate 100ms (default 500ms)
        self.eeipclient.t_o_realtime_format = RealTimeFormat.MODELESS
        self.eeipclient.t_o_owner_redundant = False
        self.eeipclient.t_o_variable_length = False
        self.eeipclient.t_o_connection_type = ConnectionType.POINT_TO_POINT

        command = Command(virtual_position_follower=1, name='Synchrostep Move')
        command.desired_jog_cw = 1
        command.desired_jog_ccw = 0
        command.desired_command_word_6 = 20
        command.desired_command_word_7 = 20
        command.desired_command_word_8 = 1
        command.desired_command_word_9 = 0

        word0 = f'{command.desired_mode_select_bit}{command.desired_preset_encoder}{command.desired_run_assembled_move}{command.desired_read_assembled_data}{command.desired_program_assembled}{command.desired_reset_errors}{command.desired_preset_motor_position}{command.desired_jog_ccw}{command.desired_jog_cw}{command.desired_find_home_ccw}{command.desired_find_home_cw}{command.desired_immediate_stop}{command.desired_resume_move}{command.desired_hold_move}{command.desired_relative_move}{command.desired_absolute_move}'
        #print(word0)
        word0 = int(word0, 2)
        #print(word0)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'{command.desired_enable_driver}{command.desired_virtual_encoder_follower}{command.desired_general_purpose_output_state}{command.desired_virtual_position_follower}{command.desired_backplane_proximity_bit}{command.desired_clear_driver_fault}{command.desired_assembled_move_type}{command.desired_indexed_command}{command.desired_registration_move}{command.desired_enable_electronic_gearing_mode}{command.desired_save_assembled_move}{command.desired_reverse_blend_direction}{command.desired_hybrid_control_enable}{command.desired_encoder_registration_move}' + format(command.desired_current_key, 'b').zfill(2)
        #print(word1)
        word1 = int(word1, 2)
        #print(word1)
        if word1 >= 2**15:
            word1 -= 2**16

        # self.eeipclient.o_t_iodata[0] = word0
        # self.eeipclient.o_t_iodata[1] = word1
        # self.eeipclient.o_t_iodata[2] = 0
        # self.eeipclient.o_t_iodata[3] = 0
        # self.eeipclient.o_t_iodata[4] = 0
        # self.eeipclient.o_t_iodata[5] = 0
        # self.eeipclient.o_t_iodata[6] = command.desired_command_word_6
        # self.eeipclient.o_t_iodata[7] = command.desired_command_word_7
        # self.eeipclient.o_t_iodata[8] = command.desired_command_word_8
        # self.eeipclient.o_t_iodata[9] = command.desired_command_word_9


    def run(self):
        if self.connected:
            self.eeipclient.forward_open()
        while self.running.is_set():
            self.changeEvent.wait(timeout=0.05)
            while len(self.explicit_poll):
                values = self.explicit_poll.pop(0)
                if self.connected:
                    print(values)
                    self.eeipclient.set_attribute_single(0x04, 0x96, 0x03, [0 for i in range(20)])

            self.process_incoming_data()
            # print('State of the first Input byte: {0}'.format(self.eeipclient.t_o_iodata[8]))
            # print('State of the second Input byte: {0}'.format(self.eeipclient.t_o_iodata[9]))

            self.changeEvent.clear()

        #Close the Session (First stop implicit Messaging then unregister the session)
        if self.connected:
            self.eeipclient.forward_close()
            self.eeipclient.unregister_session()
        
        print('AMCI Driver Thread killed')

    def change_ref(self, position, speed):
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

        desired_command_word_2 = int(pos_in_bits[16:], 2)
        if desired_command_word_2 >= 2**15:
            desired_command_word_2 -= 2**16
        desired_command_word_3 = int(pos_in_bits[:16], 2)
        if desired_command_word_3 >= 2**15:
            desired_command_word_3 -= 2**16
        desired_command_word_4 = int(speed_in_bits[16:], 2)
        if desired_command_word_4 >= 2**15:
            desired_command_word_4 -= 2**16
        desired_command_word_5 = int(speed_in_bits[:16], 2)
        if desired_command_word_5 >= 2**15:
            desired_command_word_5 -= 2**16

        self.eeipclient.o_t_iodata[2] = desired_command_word_2
        self.eeipclient.o_t_iodata[3] = desired_command_word_3
        self.eeipclient.o_t_iodata[4] = desired_command_word_4
        self.eeipclient.o_t_iodata[5] = desired_command_word_5

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
        self.motor_current = 0
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

        self.motor_position = 0
        self.encoder_position = 0
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

    def process_incoming_data(self):
        #print(par, data)
        if len(self.eeipclient.t_o_iodata) == 20:
            b = struct.pack('20B', *self.eeipclient.t_o_iodata)
            data = struct.unpack('<HHHHHHHHHH', b)
    
            word0 = format(data[0], 'b').zfill(16)
            word1 = format(data[1], 'b').zfill(16)
            mode = int(word0[0], 2)
            if mode != self.mode_select_bit:
                if mode:
                    #print('Changed to configuration mode')
                    #print(data)
                    self.request_write_return_to_command_mode()
            self.mode_select_bit = int(word0[0], 2)
            
            if self.mode_select_bit: ## configuration mode
                self.disable_anti_resonance_bit = int(word0[1], 2)
                self.enable_stall_detection_bit = int(word0[2], 2)
                self.use_backplane_proximitword0y_bit = int(word0[4], 2)
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
                self.motor_current = data[8]
                self.current_loop_gain = data[9]
            else: ## command mode
                module_ok = int(word0[1], 2)
                # if module_ok != self.module_ok:
                #     self.module_ok_signal.emit(module_ok)
                self.module_ok = module_ok
                configuration_error = int(word0[2], 2)
                if configuration_error != self.configuration_error:
                    if self.verbose:
                        print('Configuration Error')
                    # self.configuration_error_signal.emit(configuration_error)
                self.configuration_error = configuration_error
                command_error = int(word0[3], 2)
                # if command_error != self.command_error:
                #     self.command_error_signal.emit(command_error)
                self.command_error = command_error
                input_error = int(word0[4], 2)
                if input_error != self.input_error:
                    if input_error:
                        if self.fast_ccw_limit_homing:
                            self.request_write_reset_errors()
                            self.fast_ccw_limit_homing = False
                            self.slow_ccw_limit_homing = True
                            steps = [{'pos': 1000, 'speed': 500, 'acc': 100, 'dec': 100, 'jerk': 0}, 
                                     {'pos': -1000, 'speed': 200, 'acc': 100, 'dec': 100, 'jerk': 0}]
                            self.request_program_run_assembled_move(steps, dwell_move=1, dwell_time=100)
                            # self.request_write_relative_move(target_position=1000, programmed_speed=500)
                            # self.request_write_ccw_jog(programmed_speed=200)
                            # print('Step 1')
                        elif self.slow_ccw_limit_homing:
                            self.request_write_reset_errors()
                            self.slow_ccw_limit_homing = False
                            self.request_write_preset_position(0)
                    # self.input_error_signal.emit(input_error)
                self.input_error = input_error
                position_invalid = int(word0[5], 2)
                # if position_invalid != self.position_invalid:
                #     self.position_invalid_signal.emit(position_invalid)
                self.position_invalid = position_invalid
                waiting_for_assembled_segment = int(word0[6], 2)
                # if waiting_for_assembled_segment != self.waiting_for_assembled_segment:
                #     self.waiting_for_assembled_segment_signal.emit(waiting_for_assembled_segment)
                self.waiting_for_assembled_segment = waiting_for_assembled_segment
                in_assembled_mode = int(word0[7], 2)
                # if in_assembled_mode != self.in_assembled_mode:
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
                # if stall_detected != self.stall_detected:
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
                # if limit_condition != self.limit_condition:
                #     self.limit_condition_signal.emit(limit_condition)
                self.limit_condition = limit_condition
                invalid_jog_change = int(word1[6], 2)
                # if invalid_jog_change != self.invalid_jog_change:
                #     self.invalid_jog_change_signal.emit(invalid_jog_change)
                self.invalid_jog_change = invalid_jog_change
                motion_lag = int(word1[7], 2)
                # if motion_lag != self.motion_lag:
                #     self.motion_lag_signal.emit(motion_lag)
                self.motion_lag = motion_lag
                driver_fault = int(word1[8], 2)
                # if driver_fault != self.driver_fault:
                #     self.driver_fault_signal.emit(driver_fault)
                self.driver_fault = driver_fault
                connection_was_lost = int(word1[9], 2)
                # if connection_was_lost != self.connection_was_lost:
                #     self.connection_was_lost_signal.emit(connection_was_lost)
                self.connection_was_lost = connection_was_lost
                plc_in_prog_mode = int(word1[10], 2)
                # if plc_in_prog_mode != self.plc_in_prog_mode:
                #     self.plc_in_prog_mode_signal.emit(plc_in_prog_mode)
                self.plc_in_prog_mode = plc_in_prog_mode
                temperature_above_90 = int(word1[11], 2)
                # if temperature_above_90 != self.temperature_above_90:
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
    
                motor_position = data[2]*1000 + data[3]
                # if motor_position != self.motor_position:
                #     self.motor_position_signal.emit(motor_position)
                self.motor_position = motor_position
                encoder_position = data[4]*1000 + data[5]
                # if encoder_position != self.encoder_position:
                #     self.encoder_position_signal.emit(encoder_position)
                self.encoder_position = encoder_position
                captured_encoder_position = data[6]*1000 + data[7]
                # if captured_encoder_position != self.captured_encoder_position:
                #     self.captured_encoder_position_signal.emit(captured_encoder_position)
                self.captured_encoder_position = captured_encoder_position
                programed_motor_current = data[8]
                # if programed_motor_current != self.programed_motor_current:
                #     self.programed_motor_current_signal.emit(programed_motor_current)
                self.programed_motor_current = programed_motor_current
                acceleration_jerk = data[9]
                # if acceleration_jerk != self.acceleration_jerk:
                #     self.acceleration_jerk_signal.emit(acceleration_jerk)
                self.acceleration_jerk = acceleration_jerk
    
    def write_configuration(self, setting):
        word0 = f'{setting.desired_mode_select_bit}{setting.desired_disable_anti_resonance_bit}{setting.desired_enable_stall_detection_bit}0{setting.desired_use_backplane_proximity_bit}{setting.desired_use_encoder_bit}{setting.desired_home_to_encoder_z_pulse}' + format(setting.desired_input_3_function_bits, 'b').zfill(3) + format(setting.desired_input_2_function_bits, 'b').zfill(3) + format(setting.desired_input_1_function_bits, 'b').zfill(3)
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'0{setting.desired_output_functionality_bit}{setting.desired_output_state_control_on_network_lost}{setting.desired_output_state_on_network_lost}{setting.desired_read_present_configuration}{setting.desired_save_configuration}{setting.desired_binary_input_format}{setting.desired_binary_output_format}{setting.desired_binary_endian}0000{setting.desired_input_3_active_level}{setting.desired_input_2_active_level}{setting.desired_input_1_active_level}'
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        word2 = setting.desired_starting_speed // 1000
        word3 = setting.desired_starting_speed %  1000

        word4 = setting.desired_motors_step_turn
        word5 = setting.desired_hybrid_control_gain
        word6 = setting.desired_encoder_pulses_turn
        word7 = setting.desired_idle_current_percentage
        word8 = setting.desired_motor_current
        word9 = setting.desired_current_loop_gain

        self.actual_settings = setting
        print(word0, word1, word2, word3, word4, word5, word6, word7, word8, word9)
        values = struct.pack('!10h', word0, word1, word2, word3, word4, word5, word6, word7, word8, word9)

        self.explicit_poll.append(values)
        self.changeEvent.set()

    def request_write_set_starting_speed(self, starting_speed):
        #self.request_write_configuration(disable_anti_resonance_bit=self.disable_anti_resonance_bit, enable_stall_detection_bit=self.enable_stall_detection_bit, use_backplane_proximity_bit=self.use_backplane_proximity_bit, use_encoder_bit=self.use_encoder_bit, home_to_encoder_z_pulse=self.home_to_encoder_z_pulse, input_3_function_bits=self.input_3_function_bits, input_2_function_bits=self.input_2_function_bits, input_1_function_bits=self.input_1_function_bits, output_functionality_bit=self.output_functionality_bit, output_state_control_on_network_lost=self.output_state_control_on_network_lost, output_state_on_network_lost=self.output_state_on_network_lost, read_present_configuration=self.read_present_configuration, save_configuration=self.save_configuration, binary_input_format=self.binary_input_format, binary_output_format=self.binary_output_format, binary_endian=self.binary_endian, input_3_active_level=self.input_3_active_level, input_2_active_level=self.input_2_active_level, input_1_active_level=self.input_1_active_level, starting_speed=starting_speed, motors_step_turn=self.motors_step_turn, hybrid_control_gain=self.hybrid_control_gain, encoder_pulses_turn=self.encoder_pulses_turn, idle_current_percentage=self.idle_current_percentage, motor_current=self.motor_current, current_loop_gain=self.current_loop_gain)
        self.actual_settings.desired_starting_speed = starting_speed
        self.write_configuration(self.actual_settings)

    def write_command(self, command):
        word0 = f'{command.desired_mode_select_bit}{command.desired_preset_encoder}{command.desired_run_assembled_move}{command.desired_read_assembled_data}{command.desired_program_assembled}{command.desired_reset_errors}{command.desired_preset_motor_position}{command.desired_jog_ccw}{command.desired_jog_cw}{command.desired_find_home_ccw}{command.desired_find_home_cw}{command.desired_immediate_stop}{command.desired_resume_move}{command.desired_hold_move}{command.desired_relative_move}{command.desired_absolute_move}'
        #print(word0)
        word0 = int(word0, 2)
        #print(word0)
        if word0 >= 2**15:
            word0 -= 2**16
        #print(word0)
        # if word0:
        #     self.in_command_mode = False
        # else:
        #     self.in_command_mode = True

        word1 = f'{command.desired_enable_driver}{command.desired_virtual_encoder_follower}{command.desired_general_purpose_output_state}{command.desired_virtual_position_follower}{command.desired_backplane_proximity_bit}{command.desired_clear_driver_fault}{command.desired_assembled_move_type}{command.desired_indexed_command}{command.desired_registration_move}{command.desired_enable_electronic_gearing_mode}{command.desired_save_assembled_move}{command.desired_reverse_blend_direction}{command.desired_hybrid_control_enable}{command.desired_encoder_registration_move}' + format(command.desired_current_key, 'b').zfill(2)
        #print(word1)
        word1 = int(word1, 2)
        #print(word1)
        if word1 >= 2**15:
            word1 -= 2**16
        #print(word1)

        word2 = command.desired_command_word_2
        word3 = command.desired_command_word_3
        #print(word2, word3)
        word4 = command.desired_command_word_4
        word5 = command.desired_command_word_5
        word6 = command.desired_command_word_6
        word7 = command.desired_command_word_7
        word8 = command.desired_command_word_8
        word9 = command.desired_command_word_9

        #print(f'@4/150/3=(UINT){word0},{word1},{word2},{word3},{word4},{word5},{word6},{word7},{word8},{word9}')

        # with self.via:
        #     data, = self.via.read( [(f'@4/150/3=(INT){word0},{word1},{word2},{word3},{word4},{word5},{word6},{word7},{word8},{word9}', ("INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))])
        
        values = struct.pack('!10H', word0, word1, word2, word3, word4, word5, word6, word7, word8, word9)

        self.explicit_poll.append(values)
        self.changeEvent.set()
        
    def request_write_absolute_move(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=5):
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

        self.write_command(command)

    def request_write_relative_move(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0):
        command = Command(relative_move=1, name='Relative Move')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_hold_move(self, motor_current=30):
        command = Command(hold_move=1, name='Hold')
        command.desired_command_word_8 = motor_current

        self.write_command(command)

    def request_write_resume_move(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(resume_move=1, name='Resume Move')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_immediate_stop(self, motor_current=30):
        command = Command(immediate_stop=1, name='Immediate Stop')
        command.desired_command_word_8 = motor_current

        self.write_command(command)

    def request_write_ccw_find_home_to_limit(self):
        self.fast_ccw_limit_homing = True
        self.request_write_ccw_jog(programmed_speed=1000)

    def request_write_cw_find_home(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        #self.request_write_configuration(setting=self.homing_settings, save_local=False)

        command = Command(find_home_cw=1, name='CW Find Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_ccw_find_home(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        #self.request_write_configuration(setting=self.homing_settings, save_local=False)
        #sleep(1)
        #self.request_write_return_to_command_mode()
        command = Command(find_home_ccw=1, name='CCW FFind Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_cw_jog(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(jog_cw=1, name='CW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_cw_registration_move(self):
        pass
    
    def request_program_run_assembled_move(self, steps, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0):
        self.programming_assembly = True
        self.request_write_program_assembled()
        for step in steps:
            self.request_write_assembled_segment(target_position=step['pos'], programmed_speed=step['speed'], acceleration=step['acc'], deceleration=step['dec'], acceleration_jerk=step['jerk'])
            self.request_write_program_assembled()

        self.request_write_return_to_command_mode()
        self.request_write_run_assembled_move(motor_current=motor_current, blend_direction=blend_direction, dwell_move=dwell_move, dwell_time=dwell_time)
        #print('Requested move', self.hostname)

    def request_write_synchrostep_move(self, position, direction, speed=200, acceleration=100, deceleration=100, proportional_coefficient=1, network_delay=0):
        #print(target_position, programmed_speed)
        command = Command(virtual_position_follower=1, name='Synchrostep Move')
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
        

        # command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        # command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        # command.desired_command_word_4 = abs(position) // 1000 * sign(position)
        # command.desired_command_word_5 = abs(position)  % 1000 * sign(position)

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

        self.write_command(command)

    def request_write_ccw_jog(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(jog_ccw=1, name='CCW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_ccw_registration_move(self):
        pass

    def request_write_encoder_follower_move(self):
        pass

    def request_write_preset_position(self, position):
        command = Command(preset_motor_position=1, name='Preset Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)

        self.write_command(command)

    def request_write_reset_errors(self):
        command = Command(reset_errors=1, name='Reset Errors')
        
        self.write_command(command)

    def request_write_run_assembled_move(self, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0):
        command = Command(run_assembled_move=1, reverse_blend_direction=blend_direction, assembled_move_type=dwell_move, name='Run Assembled Move')
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = dwell_time

        self.write_command(command)

    def request_write_program_assembled(self):
        command = Command(program_assembled=1, name='Program Assembles')
        
        self.write_command(command)

    def request_write_assembled_segment(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0):
        command = Command(read_assembled_data=1, program_assembled=1, name='Write Assembled Segment')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk

        self.write_command(command)

    def request_write_preset_encoder_position(self, position):
        command = Command(preset_encoder=1, name='Preset Encoder Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        
        self.write_command(command)

    def request_write_return_to_command_mode(self):
        #print("self.driver.request_write_return_to_command_mode()")
        command = Command(name='Return to Command Mode')

        self.write_command(command)

    def request_write_reset(self):
        if self.verbose:
            print('Reset')
            
        self.write_configuration(self.initial_settings)

    def request_write_enable_current(self):
        command = Command(name='Enable Current')
        
        self.write_command(command)

    def request_write_disable_current(self):
        command = Command(enable_driver=0, name='Disable Current')
        
        self.write_command(command)
 
class VirtualAxis(threading.Thread):
    def __init__(self, running, motor, interval, t0, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.motor = motor
        self.ref = [(0,0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.verbose = verbose
        
    def run(self):
        while self.running.is_set():
            t = time.time() - self.t0
            pos, vel = self.get_ref(t)
            self.motor.change_ref(pos, vel)
            if self.verbose:
                print(t, pos, vel)
            self.update_ref(t)
            time.sleep(self.interval)

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

class FlowControllerDriver(threading.Thread):
    def __init__(self, host, running, connected=True):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.host = host
        self.running = running
        self.connected = connected

        self.mass_flow_reading = 0
        self.vol_flow_reading = 0
        self.temp_reading = 0
        
        self.eeipclient = EEIPClient()
        #Ip-Address of the Ethernet-IP Device (In this case Allen-Bradley 1734-AENT Point I/O)
        #A Session has to be registered before any communication can be established
        if self.connected:
            self.eeipclient.register_session(host)

        # 0x20, 0x04,0x24, 0x01, 0x2C, 0x64, 0x2C, 0x65
        #Parameters from Originator -> Target 
        #self.eeipclient.configuration_assembly_instance_id = 0x01

        self.eeipclient.o_t_instance_id = 0x64
        self.eeipclient.o_t_length = 4
        self.eeipclient.o_t_requested_packet_rate = 100000  #Packet rate 100ms (default 500ms)
        self.eeipclient.o_t_realtime_format = RealTimeFormat.HEADER32BIT
        self.eeipclient.o_t_owner_redundant = False
        self.eeipclient.o_t_variable_length = False
        self.eeipclient.o_t_connection_type = ConnectionType.POINT_TO_POINT

        #Parameters from Target -> Originator
        self.eeipclient.t_o_instance_id = 0x65
        self.eeipclient.t_o_length = 26
        self.eeipclient.t_o_requested_packet_rate = 100000  #Packet rate 100ms (default 500ms)
        self.eeipclient.t_o_realtime_format = RealTimeFormat.MODELESS
        self.eeipclient.t_o_owner_redundant = False
        self.eeipclient.t_o_variable_length = False
        self.eeipclient.t_o_connection_type = ConnectionType.POINT_TO_POINT
        self.eeipclient.o_t_iodata[3] = 64

        self.explicit_poll = []
        self.changeEvent = threading.Event()

        self.gas = 0
        self.tov_o = 0 # Temperature Overflow (TOV)
        self.tov_u = 0 # Temperature Underflow (TOV)
        self.vov_o = 0 # Volumetric Overflow (VOV)
        self.vov_u = 0 # Volumetric Underflow (VOV)
        self.mov_o = 0 # Mass Overflow (MOV)
        self.mov_u = 0 # Mass Underflow (MOV)
        self.pov_o = 0 # Pressure Overflow (POV)
        self.pov_u = 0 # Totalizer Overflow (OVR)
        self.hld = 0 # PID Loop in Hold (HLD)
        self.adc = 0 # ADC Error (ADC)
        self.exh = 0 # PID Exhaust (EXH)
        self.opl = 0 # Over pressure limit (OPL)
        self.tmf = 0 # Flow overflow during totalize (TMF)
        self.mwa = 0 # Measurement was aborted
        self.absolute_pressure = 0
        self.flow_temperature = 0
        self.volumetric_flow = 0
        self.mass_flow = 0
        self.mass_flow_set_point = 0

    def run(self):
        if self.connected:
            pass #self.eeipclient.forward_open()
        while self.running.is_set():
            self.changeEvent.wait(timeout=0.5)
            while len(self.explicit_poll):
                dir, values = self.explicit_poll.pop(0)
                if self.connected:
                    self.eeipclient.set_attribute_single(dir[0], dir[1], dir[2], values)

            self.process_incoming_data()
            # print('State of the first Input byte: {0}'.format(self.eeipclient.t_o_iodata[8]))
            # print('State of the second Input byte: {0}'.format(self.eeipclient.t_o_iodata[9]))

            self.changeEvent.clear()

        #Close the Session (First stop implicit Messaging then unregister the session)
        if self.connected:
            #self.eeipclient.forward_close()
            self.eeipclient.unregister_session()
        
        print('Flow Controller Driver Thread killed')
    
    def process_incoming_data(self):
        if len(self.eeipclient.t_o_iodata) == 26:
            b = struct.pack('26B', *self.eeipclient.t_o_iodata)
            [g, s, ap, ft, vf, mf, mfsp] = struct.unpack('<HIfffff', b) # gas, status, absolut pressure, flow temperature, volumetric flow, mass flow, mass flow set point
            self.gas = g

            s = format(s, '#035b')[2:]
            self.tov_o = s[0]
            self.tov_u = s[1]
            self.vov_o = s[2]
            self.vov_u = s[3]
            self.mov_o = s[4]
            self.mov_u = s[5]
            self.pov_o = s[6]
            self.pov_u = s[7]
            self.hld = s[8]
            self.adc = s[9]
            self.exh = s[10]
            self.opl = s[11]
            self.tmf = s[12]
            self.mwa = s[13]

            self.absolute_pressure = ap
            self.flow_temperature = ft
            self.volumetric_flow = vf
            self.mass_flow = mf
            self.mass_flow_set_point = mfsp
            print(mfsp)
    
    def change_ref(self, value):
        #print(self.eeipclient.o_t_iodata)
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        self.eeipclient.set_attribute_single(4, 100, 3, b)
        # self.eeipclient.o_t_iodata = [b[0], b[1], b[2], b[3]]
        

class PressureSensorDriver(threading.Thread):
    def __init__(self, host, running, connected=True):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.host = host
        self.running = running
        self.connected = connected
        self.eeipclient = EEIPClient()
        #Ip-Address of the Ethernet-IP Device (In this case Allen-Bradley 1734-AENT Point I/O)
        #A Session has to be registered before any communication can be established
        if self.connected:
            self.eeipclient.register_session(host)

        # 0x20, 0x04,0x24, 0x01, 0x2C, 0x64, 0x2C, 0x65
        #Parameters from Originator -> Target 
        # self.eeipclient.configuration_assembly_instance_id = 0x01

        self.eeipclient.o_t_instance_id = 0x64
        self.eeipclient.o_t_length = 4
        self.eeipclient.o_t_requested_packet_rate = 10000  #Packet rate 100ms (default 500ms)
        self.eeipclient.o_t_realtime_format = RealTimeFormat.HEADER32BIT
        self.eeipclient.o_t_owner_redundant = False
        self.eeipclient.o_t_variable_length = False
        self.eeipclient.o_t_connection_type = ConnectionType.POINT_TO_POINT

        #Parameters from Target -> Originator
        self.eeipclient.t_o_instance_id = 0x65
        self.eeipclient.t_o_length = 10
        self.eeipclient.t_o_requested_packet_rate = 10000  #Packet rate 100ms (default 500ms)
        self.eeipclient.t_o_realtime_format = RealTimeFormat.MODELESS
        self.eeipclient.t_o_owner_redundant = False
        self.eeipclient.t_o_variable_length = False
        self.eeipclient.t_o_connection_type = ConnectionType.POINT_TO_POINT
        
        self.gas = 0
        self.tov_o = 0 # Temperature Overflow (TOV)
        self.tov_u = 0 # Temperature Underflow (TOV)
        self.vov_o = 0 # Volumetric Overflow (VOV)
        self.vov_u = 0 # Volumetric Underflow (VOV)
        self.mov_o = 0 # Mass Overflow (MOV)
        self.mov_u = 0 # Mass Underflow (MOV)
        self.pov_o = 0 # Pressure Overflow (POV)
        self.pov_u = 0 # Totalizer Overflow (OVR)
        self.hld = 0 # PID Loop in Hold (HLD)
        self.adc = 0 # ADC Error (ADC)
        self.exh = 0 # PID Exhaust (EXH)
        self.opl = 0 # Over pressure limit (OPL)
        self.tmf = 0 # Flow overflow during totalize (TMF)
        self.mwa = 0 # Measurement was aborted
        self.pressure = 0

    def run(self):
        if self.connected:
            self.eeipclient.forward_open()
        while self.running.is_set():
            self.process_incoming_data()
            # print('State of the first Input byte: {0}'.format(self.eeipclient.t_o_iodata[8]))
            # print('State of the second Input byte: {0}'.format(self.eeipclient.t_o_iodata[9]))
            time.sleep(0.5)

        #Close the Session (First stop implicit Messaging then unregister the session)
        if self.connected:
            self.eeipclient.forward_close()
            self.eeipclient.unregister_session()
        
        print('Pressure Sensor Driver Thread killed')
    
    def process_incoming_data(self):
        print(self.eeipclient.t_o_iodata)
        if len(self.eeipclient.t_o_iodata) == 10:
            b = struct.pack('10B', *self.eeipclient.t_o_iodata)
            [g, s, p] = struct.unpack('<HIf', b)
            self.gas = g
            self.pressure = p
            s = format(s, '#035b')[2:]
            self.tov_o = s[0]
            self.tov_u = s[1]
            self.vov_o = s[2]
            self.vov_u = s[3]
            self.mov_o = s[4]
            self.mov_u = s[5]
            self.pov_o = s[6]
            self.pov_u = s[7]
            self.hld = s[8]
            self.adc = s[9]
            self.exh = s[10]
            self.opl = s[11]
            self.tmf = s[12]
            self.mwa = s[13]

class VirtualFlow(threading.Thread):
    def __init__(self, running, valve, interval, t0, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.valve = valve
        self.ref = [(0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.verbose = verbose
        self.vibrato_amp = 0
        self.vibrato_freq = 0
        
    def run(self):
        while self.running.is_set():
            t = time.time() - self.t0
            flow = self.get_ref(t)
            self.valve.change_ref(flow)
            if self.verbose:
                print(t, flow)
            self.update_ref(t)
            time.sleep(self.interval)

    def get_ref(self, t):
        if self.ref[-1][0] > t:
            ramp = get_value_from_func(t, self.ref, approx=False)
            vibr = ramp * self.vibrato_amp * sin(t * 2*pi * self.vibrato_freq)
            flow = max(0,min(50, ramp+vibr))
        else:
            ramp = self.ref[-1][1]
            vibr = ramp * self.vibrato_amp * sin(t * 2*pi * self.vibrato_freq)
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

class Musician:
    def __init__(self, x_driver, z_driver, alpha_driver, flow_driver, pressure_driver, running, interval):
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.flow_driver = flow_driver
        self.pressure_driver = pressure_driver
        self.t0 = time.time()
        self.x_virtual_axis = VirtualAxis(running, x_driver, interval, self.t0, verbose=False)
        self.z_virtual_axis = VirtualAxis(running, z_driver, interval, self.t0, verbose=False)
        self.alpha_virtual_axis = VirtualAxis(running, alpha_driver, interval, self.t0, verbose=False)
        self.virtual_flow = VirtualFlow(running, flow_driver, interval, self.t0, verbose=False)
        self.x_virtual_axis.start()
        self.z_virtual_axis.start()
        self.alpha_virtual_axis.start()
        self.virtual_flow.start()

    def move_to(self, desired_state, T=None):
        my_state = State(0, 0, 0, 0)
        my_state.x = x_units_to_mm(self.x_driver.motor_position)
        my_state.z = z_units_to_mm(self.z_driver.motor_position)
        my_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
        my_state.flow = self.flow_driver.mass_flow_reading
        route = get_route(my_state, desired_state, T=T)

        move_t0 = time.time() - self.t0
        route_x = []
        route_z = []
        route_alpha = []
        route_flow = []
        for i in range(len(route['t'])):
            route_x.append((route['t'][i] + move_t0, route['x'][i], route['x_vel'][i]))
            route_z.append((route['t'][i] + move_t0, route['z'][i], route['z_vel'][i]))
            route_alpha.append((route['t'][i] + move_t0, route['alpha'][i], route['alpha_vel'][i]))
            route_flow.append((route['t'][i] + move_t0, route['flow'][i]))

        self.x_virtual_axis.merge_ref(route_x)
        self.z_virtual_axis.merge_ref(route_z)
        self.alpha_virtual_axis.merge_ref(route_alpha)
        self.virtual_flow.merge_ref(route_flow)
        self.virtual_flow.vibrato_amp = desired_state.vibrato_amp
        self.virtual_flow.vibrato_freq = desired_state.vibrato_freq

    def execute_score(self, path, go_back=True):
        route = get_route_complete(path, go_back=go_back)
        score_t0 = time.time() - self.t0

        route_x = []
        route_z = []
        route_alpha = []
        route_flow = []
        for i in range(len(route['t'])):
            route_x.append((route['t'][i] + score_t0, route['x'][i], route['x_vel'][i]))
            route_z.append((route['t'][i] + score_t0, route['z'][i], route['z_vel'][i]))
            route_alpha.append((route['t'][i] + score_t0, route['alpha'][i], route['alpha_vel'][i]))
            route_flow.append((route['t'][i] + score_t0, route['flow'][i]))

        self.x_virtual_axis.merge_ref(route_x)
        self.z_virtual_axis.merge_ref(route_z)
        self.alpha_virtual_axis.merge_ref(route_alpha)
        self.virtual_flow.merge_ref(route_flow)

if __name__ == "__main__":
    event = threading.Event()
    
    event.set()
    host1 = '192.168.2.104'
    host2 = '192.168.2.102'
    host3 = '192.168.2.103'

    host4 = '192.168.2.101'
    host5 = '192.168.2.100'

    connected = False
    # eje_x = AMCIDriver(host1, event, connected=connected)
    # eje_x.start()

    # eje_z = AMCIDriver(host2, event, connected=connected)
    # eje_z.start()

    # eje_alpha = AMCIDriver(host3, event, connected=True)
    # eje_alpha.start()

    flow_valve = FlowControllerDriver(host4, event, connected=True)
    flow_valve.start()

    # pressure_driver = PressureSensorDriver(host5, event, connected=True)
    # pressure_driver.start()

    # axis_intel = Musician(eje_x, eje_z, eje_alpha, flow_valve, pressure_driver, event, 0.1)

    while True:
        p = input(">> ")
        if p == "q":
            event.clear()
            break
        elif p == "s":
            pass #axis_intel.execute_score("/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/test_motor_2.json")
        elif p == "m":
            desired_state = State(15, 45, 0, 10)
            pass #axis_intel.move_to(desired_state)
        elif p == "v":
            v = input('Set Point: ')
            flow_valve.change_ref(float(v))