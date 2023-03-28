import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
import ethernetip
from utils.motor_route import *
import struct
import time
import numpy as np

class Command:
    def __init__(self, preset_encoder=0, run_assembled_move=0, program_assembled=0, read_assembled_data=0, reset_errors=0, preset_motor_position=0, jog_ccw=0, jog_cw=0, find_home_ccw=0, find_home_cw=0, immediate_stop=0, resume_move=0, hold_move=0, relative_move=0, absolute_move=0, enable_driver=1, virtual_encoder_follower=0, general_purpose_output_state=0, virtual_position_follower=0, backplane_proximity_bit=0, clear_driver_fault=0, assembled_move_type=0, indexed_command=0, registration_move=0, enable_electronic_gearing_mode=0, save_assembled_move=0, reverse_blend_direction=0, hybrid_control_enable=0, encoder_registration_move=0, current_key_1=0, current_key_0=0, desired_command_word_2=0, desired_command_word_3=0, desired_command_word_4=0, desired_command_word_5=0, desired_command_word_6=0, desired_command_word_7=0, desired_command_word_8=0, desired_command_word_9=0, name=''):
        self._desired_mode_select_bit = 0
        self._desired_preset_encoder = preset_encoder
        self._desired_run_assembled_move = run_assembled_move
        self._desired_program_assembled = program_assembled
        self._desired_read_assembled_data = read_assembled_data
        self._desired_reset_errors = reset_errors
        self._desired_preset_motor_position = preset_motor_position
        self._desired_jog_ccw = jog_ccw
        self._desired_jog_cw = jog_cw
        self._desired_find_home_ccw = find_home_ccw
        self._desired_find_home_cw = find_home_cw
        self._desired_immediate_stop = immediate_stop
        self._desired_resume_move = resume_move
        self._desired_hold_move = hold_move
        self._desired_relative_move = relative_move
        self._desired_absolute_move = absolute_move

        self._desired_enable_driver = enable_driver
        self._desired_virtual_encoder_follower = virtual_encoder_follower
        self._desired_general_purpose_output_state = general_purpose_output_state
        self._desired_virtual_position_follower = virtual_position_follower
        self._desired_backplane_proximity_bit = backplane_proximity_bit
        self._desired_clear_driver_fault = clear_driver_fault
        self._desired_assembled_move_type = assembled_move_type
        self._desired_indexed_command = indexed_command
        self._desired_registration_move = registration_move
        self._desired_enable_electronic_gearing_mode = enable_electronic_gearing_mode
        self._desired_save_assembled_move = save_assembled_move
        self._desired_reverse_blend_direction = reverse_blend_direction
        self._desired_hybrid_control_enable = hybrid_control_enable
        self._desired_encoder_registration_move = encoder_registration_move
        self._desired_current_key_1 = current_key_1
        self._desired_current_key_0 = current_key_0

        self._desired_command_word_2 = desired_command_word_2        
        self._desired_command_word_3 = desired_command_word_3        
        self._desired_command_word_4 = desired_command_word_4
        self._desired_command_word_5 = desired_command_word_5
        self._desired_command_word_6 = desired_command_word_6
        self._desired_command_word_7 = desired_command_word_7
        self._desired_command_word_8 = desired_command_word_8
        self._desired_command_word_9 = desired_command_word_9

        self.update()
        self.name = name
    
    def update(self):
        self.list_to_send = []
        self.list_to_send += [self._desired_jog_ccw, self._desired_preset_motor_position, self._desired_reset_errors, self._desired_program_assembled, self._desired_read_assembled_data, self._desired_run_assembled_move, self._desired_preset_encoder, 0]
        self.list_to_send += [self._desired_absolute_move, self._desired_relative_move, self._desired_hold_move, self._desired_resume_move, self._desired_immediate_stop, self._desired_find_home_cw, self._desired_find_home_ccw, self._desired_jog_cw]
        self.list_to_send += [self._desired_indexed_command, self._desired_assembled_move_type, self._desired_clear_driver_fault, self._desired_backplane_proximity_bit, self._desired_virtual_position_follower, self._desired_general_purpose_output_state, self._desired_virtual_encoder_follower, self._desired_enable_driver]
        self.list_to_send += [self._desired_current_key_0, self._desired_current_key_1, self._desired_encoder_registration_move, self._desired_hybrid_control_enable, self._desired_reverse_blend_direction, self._desired_save_assembled_move, self._desired_enable_electronic_gearing_mode, self._desired_registration_move]

        desired_command_word_2 = self._desired_command_word_2
        if self._desired_command_word_2 < 0:
            desired_command_word_2 += 2**16
        desired_command_word_3 = self._desired_command_word_3
        if self._desired_command_word_3 < 0:
            desired_command_word_3 += 2**16
        desired_command_word_4 = self._desired_command_word_4
        if self._desired_command_word_4 < 0:
            desired_command_word_4 += 2**16
        desired_command_word_5 = self._desired_command_word_5
        if self._desired_command_word_5 < 0:
            desired_command_word_5 += 2**16
        desired_command_word_6 = self._desired_command_word_6
        if self._desired_command_word_6 < 0:
            desired_command_word_6 += 2**16
        desired_command_word_7 = self._desired_command_word_7
        if self._desired_command_word_7 < 0:
            desired_command_word_7 += 2**16
        desired_command_word_8 = self._desired_command_word_8
        if self._desired_command_word_8 < 0:
            desired_command_word_8 += 2**16
        desired_command_word_9 = self._desired_command_word_9
        if self._desired_command_word_9 < 0:
            desired_command_word_9 += 2**16

        self._desired_command_word_2_list = [int(i) for i in "{0:b}".format(desired_command_word_2).zfill(16)]
        self._desired_command_word_3_list = [int(i) for i in "{0:b}".format(desired_command_word_3).zfill(16)]
        self._desired_command_word_4_list = [int(i) for i in "{0:b}".format(desired_command_word_4).zfill(16)]
        self._desired_command_word_5_list = [int(i) for i in "{0:b}".format(desired_command_word_5).zfill(16)]
        self._desired_command_word_6_list = [int(i) for i in "{0:b}".format(desired_command_word_6).zfill(16)]
        self._desired_command_word_7_list = [int(i) for i in "{0:b}".format(desired_command_word_7).zfill(16)]
        self._desired_command_word_8_list = [int(i) for i in "{0:b}".format(desired_command_word_8).zfill(16)]
        self._desired_command_word_9_list = [int(i) for i in "{0:b}".format(desired_command_word_9).zfill(16)]

        self.list_to_send += self._desired_command_word_2_list[7::-1]
        self.list_to_send += self._desired_command_word_2_list[:7:-1]
        self.list_to_send += self._desired_command_word_3_list[7::-1]
        self.list_to_send += self._desired_command_word_3_list[:7:-1]
        self.list_to_send += self._desired_command_word_4_list[7::-1]
        self.list_to_send += self._desired_command_word_4_list[:7:-1]
        self.list_to_send += self._desired_command_word_5_list[7::-1]
        self.list_to_send += self._desired_command_word_5_list[:7:-1]
        self.list_to_send += self._desired_command_word_6_list[7::-1]
        self.list_to_send += self._desired_command_word_6_list[:7:-1]
        self.list_to_send += self._desired_command_word_7_list[7::-1]
        self.list_to_send += self._desired_command_word_7_list[:7:-1]
        self.list_to_send += self._desired_command_word_8_list[7::-1]
        self.list_to_send += self._desired_command_word_8_list[:7:-1]
        self.list_to_send += self._desired_command_word_9_list[7::-1]
        self.list_to_send += self._desired_command_word_9_list[:7:-1]

        for i in range(len(self.list_to_send)):
            self.list_to_send[i] = self.list_to_send[i] == 1
        
        word0 = f'{self._desired_mode_select_bit}{self._desired_preset_encoder}{self._desired_run_assembled_move}{self._desired_read_assembled_data}{self._desired_program_assembled}{self._desired_reset_errors}{self._desired_preset_motor_position}{self._desired_jog_ccw}{self._desired_jog_cw}{self._desired_find_home_ccw}{self._desired_find_home_cw}{self._desired_immediate_stop}{self._desired_resume_move}{self._desired_hold_move}{self._desired_relative_move}{self._desired_absolute_move}'
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

        word1 = f'{self._desired_enable_driver}{self._desired_virtual_encoder_follower}{self._desired_general_purpose_output_state}{self._desired_virtual_position_follower}{self._desired_backplane_proximity_bit}{self._desired_clear_driver_fault}{self._desired_assembled_move_type}{self._desired_indexed_command}{self._desired_registration_move}{self._desired_enable_electronic_gearing_mode}{self._desired_save_assembled_move}{self._desired_reverse_blend_direction}{self._desired_hybrid_control_enable}{self._desired_encoder_registration_move}{self._desired_current_key_1}{self._desired_current_key_0}'
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        self.ints_to_send = [word0, word1, self._desired_command_word_2, self._desired_command_word_3, self._desired_command_word_4, self._desired_command_word_5, self._desired_command_word_6, self._desired_command_word_7, self._desired_command_word_8, self._desired_command_word_9]

        self.byte_to_send  = b''
        for i in self.ints_to_send:
            self.byte_to_send += struct.pack("h", i)

    @property
    def desired_mode_select_bit(self):
        return self._desired_mode_select_bit
    
    @desired_mode_select_bit.setter
    def desired_mode_select_bit(self, new_desired_mode_select_bit):
        self._desired_mode_select_bit = new_desired_mode_select_bit
        self.update()

    @property
    def desired_preset_encoder(self):
        return self._desired_preset_encoder
    
    @desired_preset_encoder.setter
    def desired_preset_encoder(self, new_desired_preset_encoder):
        self._desired_preset_encoder = new_desired_preset_encoder
        self.update()

    @property
    def desired_run_assembled_move(self):
        return self._desired_run_assembled_move
    
    @desired_run_assembled_move.setter
    def desired_run_assembled_move(self, new_desired_run_assembled_move):
        self._desired_run_assembled_move = new_desired_run_assembled_move
        self.update()

    @property
    def desired_program_assembled(self):
        return self._desired_program_assembled
    
    @desired_program_assembled.setter
    def desired_program_assembled(self, new_desired_program_assembled):
        self._desired_program_assembled = new_desired_program_assembled
        self.update()

    @property
    def desired_read_assembled_data(self):
        return self._desired_read_assembled_data
    
    @desired_read_assembled_data.setter
    def desired_read_assembled_data(self, new_desired_read_assembled_data):
        self._desired_read_assembled_data = new_desired_read_assembled_data
        self.update()

    @property
    def desired_reset_errors(self):
        return self._desired_reset_errors
    
    @desired_reset_errors.setter
    def desired_reset_errors(self, new_desired_reset_errors):
        self._desired_reset_errors = new_desired_reset_errors
        self.update()

    @property
    def desired_preset_motor_position(self):
        return self._desired_preset_motor_position
    
    @desired_preset_motor_position.setter
    def desired_preset_motor_position(self, new_desired_preset_motor_position):
        self._desired_preset_motor_position = new_desired_preset_motor_position
        self.update()

    @property
    def desired_jog_ccw(self):
        return self._desired_jog_ccw
    
    @desired_jog_ccw.setter
    def desired_jog_ccw(self, new_desired_jog_ccw):
        self._desired_jog_ccw = new_desired_jog_ccw
        self.update()

    @property
    def desired_jog_cw(self):
        return self._desired_jog_cw
    
    @desired_jog_cw.setter
    def desired_jog_cw(self, new_desired_jog_cw):
        self._desired_jog_cw = new_desired_jog_cw
        self.update()

    @property
    def desired_find_home_ccw(self):
        return self._desired_find_home_ccw
    
    @desired_find_home_ccw.setter
    def desired_find_home_ccw(self, new_desired_find_home_ccw):
        self._desired_find_home_ccw = new_desired_find_home_ccw
        self.update()

    @property
    def desired_find_home_cw(self):
        return self._desired_find_home_cw
    
    @desired_find_home_cw.setter
    def desired_find_home_cw(self, new_desired_find_home_cw):
        self._desired_find_home_cw = new_desired_find_home_cw
        self.update()

    @property
    def desired_immediate_stop(self):
        return self._desired_immediate_stop
    
    @desired_immediate_stop.setter
    def desired_immediate_stop(self, new_desired_immediate_stop):
        self._desired_immediate_stop = new_desired_immediate_stop
        self.update()

    @property
    def desired_resume_move(self):
        return self._desired_resume_move
    
    @desired_resume_move.setter
    def desired_resume_move(self, new_desired_resume_move):
        self._desired_resume_move = new_desired_resume_move
        self.update()

    @property
    def desired_hold_move(self):
        return self._desired_hold_move
    
    @desired_hold_move.setter
    def desired_hold_move(self, new_desired_hold_move):
        self._desired_hold_move = new_desired_hold_move
        self.update()

    @property
    def desired_relative_move(self):
        return self._desired_relative_move
    
    @desired_relative_move.setter
    def desired_relative_move(self, new_desired_relative_move):
        self._desired_relative_move = new_desired_relative_move
        self.update()

    @property
    def desired_absolute_move(self):
        return self._desired_absolute_move
    
    @desired_absolute_move.setter
    def desired_absolute_move(self, new_desired_absolute_move):
        self._desired_absolute_move = new_desired_absolute_move
        self.update()


    @property
    def desired_enable_driver(self):
        return self._desired_enable_driver
    
    @desired_enable_driver.setter
    def desired_enable_driver(self, new_desired_enable_driver):
        self._desired_enable_driver = new_desired_enable_driver
        self.update()

    @property
    def desired_virtual_encoder_follower(self):
        return self._desired_virtual_encoder_follower
    
    @desired_virtual_encoder_follower.setter
    def desired_virtual_encoder_follower(self, new_desired_virtual_encoder_follower):
        self._desired_virtual_encoder_follower = new_desired_virtual_encoder_follower
        self.update()

    @property
    def desired_general_purpose_output_state(self):
        return self._desired_general_purpose_output_state
    
    @desired_general_purpose_output_state.setter
    def desired_general_purpose_output_state(self, new_desired_general_purpose_output_state):
        self._desired_general_purpose_output_state = new_desired_general_purpose_output_state
        self.update()

    @property
    def desired_virtual_position_follower(self):
        return self._desired_virtual_position_follower
    
    @desired_virtual_position_follower.setter
    def desired_virtual_position_follower(self, new_desired_virtual_position_follower):
        self._desired_virtual_position_follower = new_desired_virtual_position_follower
        self.update()

    @property
    def desired_backplane_proximity_bit(self):
        return self._desired_backplane_proximity_bit
    
    @desired_backplane_proximity_bit.setter
    def desired_backplane_proximity_bit(self, new_desired_backplane_proximity_bit):
        self._desired_backplane_proximity_bit = new_desired_backplane_proximity_bit
        self.update()

    @property
    def desired_clear_driver_fault(self):
        return self._desired_clear_driver_fault
    
    @desired_clear_driver_fault.setter
    def desired_clear_driver_fault(self, new_desired_clear_driver_fault):
        self._desired_clear_driver_fault = new_desired_clear_driver_fault
        self.update()

    @property
    def desired_assembled_move_type(self):
        return self._desired_assembled_move_type
    
    @desired_assembled_move_type.setter
    def desired_assembled_move_type(self, new_desired_assembled_move_type):
        self._desired_assembled_move_type = new_desired_assembled_move_type
        self.update()

    @property
    def desired_indexed_command(self):
        return self._desired_indexed_command
    
    @desired_indexed_command.setter
    def desired_indexed_command(self, new_desired_indexed_command):
        self._desired_indexed_command = new_desired_indexed_command
        self.update()

    @property
    def desired_registration_move(self):
        return self._desired_registration_move
    
    @desired_registration_move.setter
    def desired_registration_move(self, new_desired_registration_move):
        self._desired_registration_move = new_desired_registration_move
        self.update()

    @property
    def desired_enable_electronic_gearing_mode(self):
        return self._desired_enable_electronic_gearing_mode
    
    @desired_enable_electronic_gearing_mode.setter
    def desired_enable_electronic_gearing_mode(self, new_desired_enable_electronic_gearing_mode):
        self._desired_enable_electronic_gearing_mode = new_desired_enable_electronic_gearing_mode
        self.update()

    @property
    def desired_save_assembled_move(self):
        return self._desired_save_assembled_move
    
    @desired_save_assembled_move.setter
    def desired_save_assembled_move(self, new_desired_save_assembled_move):
        self._desired_save_assembled_move = new_desired_save_assembled_move
        self.update()

    @property
    def desired_reverse_blend_direction(self):
        return self._desired_reverse_blend_direction
    
    @desired_reverse_blend_direction.setter
    def desired_reverse_blend_direction(self, new_desired_reverse_blend_direction):
        self._desired_reverse_blend_direction = new_desired_reverse_blend_direction
        self.update()

    @property
    def desired_hybrid_control_enable(self):
        return self._desired_hybrid_control_enable
    
    @desired_hybrid_control_enable.setter
    def desired_hybrid_control_enable(self, new_desired_hybrid_control_enable):
        self._desired_hybrid_control_enable = new_desired_hybrid_control_enable
        self.update()

    @property
    def desired_encoder_registration_move(self):
        return self._desired_encoder_registration_move
    
    @desired_encoder_registration_move.setter
    def desired_encoder_registration_move(self, new_desired_encoder_registration_move):
        self._desired_encoder_registration_move = new_desired_encoder_registration_move
        self.update()

    @property
    def desired_current_key_1(self):
        return self._desired_current_key_1
    
    @desired_current_key_1.setter
    def desired_current_key_1(self, new_desired_current_key_1):
        self._desired_current_key_1 = new_desired_current_key_1
        self.update()

    @property
    def desired_current_key_0(self):
        return self._desired_current_key_0
    
    @desired_current_key_0.setter
    def desired_current_key_0(self, new_desired_current_key_0):
        self._desired_current_key_0 = new_desired_current_key_0
        self.update()


    @property
    def desired_mode_select_bit(self):
        return self._desired_mode_select_bit
        
    @desired_mode_select_bit.setter
    def desired_mode_select_bit(self, new_desired_mode_select_bit):
        self._desired_mode_select_bit = new_desired_mode_select_bit
        self.update
    
    @property
    def desired_command_word_2(self):
        return self._desired_command_word_2
    
    @desired_command_word_2.setter
    def desired_command_word_2(self, new_desired_command_word_2):
        self._desired_command_word_2 = new_desired_command_word_2
        self.update()

    @property
    def desired_command_word_3(self):
        return self._desired_command_word_3
    
    @desired_command_word_3.setter
    def desired_command_word_3(self, new_desired_command_word_3):
        self._desired_command_word_3 = new_desired_command_word_3
        self.update()

    @property
    def desired_command_word_4(self):
        return self._desired_command_word_4
    
    @desired_command_word_4.setter
    def desired_command_word_4(self, new_desired_command_word_4):
        self._desired_command_word_4 = new_desired_command_word_4
        self.update()

    @property
    def desired_command_word_5(self):
        return self._desired_command_word_5
    
    @desired_command_word_5.setter
    def desired_command_word_5(self, new_desired_command_word_5):
        self._desired_command_word_5 = new_desired_command_word_5
        self.update()

    @property
    def desired_command_word_6(self):
        return self._desired_command_word_6
    
    @desired_command_word_6.setter
    def desired_command_word_6(self, new_desired_command_word_6):
        self._desired_command_word_6 = new_desired_command_word_6
        self.update()

    @property
    def desired_command_word_7(self):
        return self._desired_command_word_7
    
    @desired_command_word_7.setter
    def desired_command_word_7(self, new_desired_command_word_7):
        self._desired_command_word_7 = new_desired_command_word_7
        self.update()

    @property
    def desired_command_word_8(self):
        return self._desired_command_word_8
    
    @desired_command_word_8.setter
    def desired_command_word_8(self, new_desired_command_word_8):
        self._desired_command_word_8 = new_desired_command_word_8
        self.update()

    @property
    def desired_command_word_9(self):
        return self._desired_command_word_9
    
    @desired_command_word_9.setter
    def desired_command_word_9(self, new_desired_command_word_9):
        self._desired_command_word_9 = new_desired_command_word_9
        self.update()

class Setting:
    def __init__(self, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=50, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5):
        
        self._desired_mode_select_bit = 1
        self._desired_disable_anti_resonance_bit = disable_anti_resonance_bit
        self._desired_enable_stall_detection_bit = enable_stall_detection_bit
        self._desired_use_backplane_proximity_bit = use_backplane_proximity_bit
        self._desired_use_encoder_bit = use_encoder_bit
        self._desired_home_to_encoder_z_pulse = home_to_encoder_z_pulse
        self._desired_input_3_function_bits = input_3_function_bits
        self._desired_input_2_function_bits = input_2_function_bits
        self._desired_input_1_function_bits = input_1_function_bits

        self._desired_output_functionality_bit = output_functionality_bit
        self._desired_output_state_control_on_network_lost = output_state_control_on_network_lost
        self._desired_output_state_on_network_lost = output_state_on_network_lost
        self._desired_read_present_configuration = read_present_configuration
        self._desired_save_configuration = save_configuration
        self._desired_binary_input_format = binary_input_format
        self._desired_binary_output_format = binary_output_format
        self._desired_binary_endian = binary_endian
        self._desired_input_3_active_level = input_3_active_level
        self._desired_input_2_active_level = input_2_active_level
        self._desired_input_1_active_level = input_1_active_level        

        self._desired_starting_speed = starting_speed
        self._desired_motors_step_turn = motors_step_turn
        self._desired_hybrid_control_gain = hybrid_control_gain
        self._desired_encoder_pulses_turn = encoder_pulses_turn
        self._desired_idle_current_percentage = idle_current_percentage
        self._desired_motor_current = motor_current
        self._desired_current_loop_gain = current_loop_gain
        
        self.update()

        self.name = 'Settings configuration'

    def update(self):
        self.list_to_send = []

        input_3_function_bits_in_bits = "{0:b}".format(self._desired_input_3_function_bits).zfill(3)
        input_2_function_bits_in_bits = "{0:b}".format(self._desired_input_2_function_bits).zfill(3)
        input_1_function_bits_in_bits = "{0:b}".format(self._desired_input_1_function_bits).zfill(3)

        self.list_to_send += [int(input_3_function_bits_in_bits[0]), self._desired_home_to_encoder_z_pulse, self._desired_use_encoder_bit, self._desired_use_backplane_proximity_bit, 0, self._desired_enable_stall_detection_bit, self._desired_disable_anti_resonance_bit, 1]
        self.list_to_send += [int(input_1_function_bits_in_bits[2]), int(input_1_function_bits_in_bits[1]), int(input_1_function_bits_in_bits[0]), int(input_2_function_bits_in_bits[2]), int(input_2_function_bits_in_bits[1]), int(input_2_function_bits_in_bits[0]), int(input_3_function_bits_in_bits[2]), int(input_3_function_bits_in_bits[1])]

        self.list_to_send += [self._desired_binary_output_format, self._desired_binary_input_format, self._desired_save_configuration, self._desired_read_present_configuration, self._desired_output_state_on_network_lost, self._desired_output_state_control_on_network_lost, self._desired_output_functionality_bit, 0]
        self.list_to_send += [self._desired_input_1_active_level, self._desired_input_2_active_level, self._desired_input_3_active_level, 0, 0, 0, 0, self._desired_binary_endian]

        starting_speed = self._desired_starting_speed
        if self._desired_starting_speed < 0:
            starting_speed += 2**32
        word2 = starting_speed // 1000
        word3 = starting_speed %  1000
        self._desired_starting_speed_list = [int(i) for i in "{0:b}".format(word2).zfill(16)] + [int(i) for i in "{0:b}".format(word3).zfill(16)]
        
        motors_step_turn = self._desired_motors_step_turn
        if self._desired_motors_step_turn < 0:
            motors_step_turn += 2**16
        self._desired_motors_step_turn_list = [int(i) for i in "{0:b}".format(motors_step_turn).zfill(16)]
        
        hybrid_control_gain = self._desired_hybrid_control_gain
        if self._desired_hybrid_control_gain < 0:
            hybrid_control_gain += 2**16
        self._desired_hybrid_control_gain_list = [int(i) for i in "{0:b}".format(hybrid_control_gain).zfill(16)]
        
        encoder_pulses_turn = self._desired_encoder_pulses_turn
        if self._desired_encoder_pulses_turn < 0:
            encoder_pulses_turn += 2**16
        self._desired_encoder_pulses_turn_list = [int(i) for i in "{0:b}".format(encoder_pulses_turn).zfill(16)]
        
        idle_current_percentage = self._desired_idle_current_percentage
        if self._desired_idle_current_percentage < 0:
            idle_current_percentage += 2**16
        self._desired_idle_current_percentage_list = [int(i) for i in "{0:b}".format(idle_current_percentage).zfill(16)]
        
        motor_current = self._desired_motor_current
        if self._desired_motor_current < 0:
            motor_current += 2**16
        self._desired_motor_current_list = [int(i) for i in "{0:b}".format(motor_current).zfill(16)]
        
        current_loop_gain = self._desired_current_loop_gain
        if self._desired_current_loop_gain < 0:
            current_loop_gain += 2**16
        self._desired_current_loop_gain_list = [int(i) for i in "{0:b}".format(current_loop_gain).zfill(16)]

        self.list_to_send += self._desired_starting_speed_list[15-8::-1]
        self.list_to_send += self._desired_starting_speed_list[15:7:-1]
        self.list_to_send += self._desired_starting_speed_list[15+8:15:-1]
        self.list_to_send += self._desired_starting_speed_list[:15+8:-1]

        self.list_to_send += self._desired_motors_step_turn_list[7::-1]
        self.list_to_send += self._desired_motors_step_turn_list[:7:-1]

        self.list_to_send += self._desired_hybrid_control_gain_list[7::-1]
        self.list_to_send += self._desired_hybrid_control_gain_list[:7:-1]

        self.list_to_send += self._desired_encoder_pulses_turn_list[7::-1]
        self.list_to_send += self._desired_encoder_pulses_turn_list[:7:-1]

        self.list_to_send += self._desired_idle_current_percentage_list[7::-1]
        self.list_to_send += self._desired_idle_current_percentage_list[:7:-1]

        self.list_to_send += self._desired_motor_current_list[7::-1]
        self.list_to_send += self._desired_motor_current_list[:7:-1]

        self.list_to_send += self._desired_current_loop_gain_list[7::-1]
        self.list_to_send += self._desired_current_loop_gain_list[:7:-1]

        for i in range(len(self.list_to_send)):
            self.list_to_send[i] = self.list_to_send[i] == 1

        word0 = f'{self._desired_mode_select_bit}{self._desired_disable_anti_resonance_bit}{self._desired_enable_stall_detection_bit}0{self._desired_use_backplane_proximity_bit}{self._desired_use_encoder_bit}{self._desired_home_to_encoder_z_pulse}' + format(self._desired_input_3_function_bits, 'b').zfill(3) + format(self._desired_input_2_function_bits, 'b').zfill(3) + format(self._desired_input_1_function_bits, 'b').zfill(3)
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'0{self._desired_output_functionality_bit}{self._desired_output_state_control_on_network_lost}{self._desired_output_state_on_network_lost}{self._desired_read_present_configuration}{self._desired_save_configuration}{self._desired_binary_input_format}{self._desired_binary_output_format}{self._desired_binary_endian}0000{self._desired_input_3_active_level}{self._desired_input_2_active_level}{self._desired_input_1_active_level}'
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        self.ints_to_send = [word0, word1, self._desired_starting_speed//1000, self._desired_starting_speed%1000, self._desired_motors_step_turn, self._desired_hybrid_control_gain, self.desired_encoder_pulses_turn, self._desired_idle_current_percentage, self._desired_motor_current, self._desired_current_loop_gain]

        self.byte_to_send = b''
        for i in self.ints_to_send:
            self.byte_to_send += struct.pack("h", i)

    @property
    def desired_mode_select_bit(self):
        return self._desired_mode_select_bit
    
    @desired_mode_select_bit.setter
    def desired_mode_select_bit(self, new_desired_mode_select_bit):
        self._desired_mode_select_bit = new_desired_mode_select_bit
        self.update()

    @property
    def desired_disable_anti_resonance_bit(self):
        return self._desired_disable_anti_resonance_bit
    
    @desired_disable_anti_resonance_bit.setter
    def desired_disable_anti_resonance_bit(self, new_desired_disable_anti_resonance_bit):
        self._desired_disable_anti_resonance_bit = new_desired_disable_anti_resonance_bit
        self.update()

    @property
    def desired_enable_stall_detection_bit(self):
        return self._desired_enable_stall_detection_bit
    
    @desired_enable_stall_detection_bit.setter
    def desired_enable_stall_detection_bit(self, new_desired_enable_stall_detection_bit):
        self._desired_enable_stall_detection_bit = new_desired_enable_stall_detection_bit
        self.update()

    @property
    def desired_use_backplane_proximity_bit(self):
        return self._desired_use_backplane_proximity_bit
    
    @desired_use_backplane_proximity_bit.setter
    def desired_use_backplane_proximity_bit(self, new_desired_use_backplane_proximity_bit):
        self._desired_use_backplane_proximity_bit = new_desired_use_backplane_proximity_bit
        self.update()

    @property
    def desired_use_encoder_bit(self):
        return self._desired_use_encoder_bit
    
    @desired_use_encoder_bit.setter
    def desired_use_encoder_bit(self, new_desired_use_encoder_bit):
        self._desired_use_encoder_bit = new_desired_use_encoder_bit
        self.update()

    @property
    def desired_home_to_encoder_z_pulse(self):
        return self._desired_home_to_encoder_z_pulse
    
    @desired_home_to_encoder_z_pulse.setter
    def desired_home_to_encoder_z_pulse(self, new_desired_home_to_encoder_z_pulse):
        self._desired_home_to_encoder_z_pulse = new_desired_home_to_encoder_z_pulse
        self.update()

    @property
    def desired_input_3_function_bits(self):
        return self._desired_input_3_function_bits
    
    @desired_input_3_function_bits.setter
    def desired_input_3_function_bits(self, new_desired_input_3_function_bits):
        self._desired_input_3_function_bits = new_desired_input_3_function_bits
        self.update()

    @property
    def desired_input_2_function_bits(self):
        return self._desired_input_2_function_bits
    
    @desired_input_2_function_bits.setter
    def desired_input_2_function_bits(self, new_desired_input_2_function_bits):
        self._desired_input_2_function_bits = new_desired_input_2_function_bits
        self.update()

    @property
    def desired_input_1_function_bits(self):
        return self._desired_input_1_function_bits
    
    @desired_input_1_function_bits.setter
    def desired_input_1_function_bits(self, new_desired_input_1_function_bits):
        self._desired_input_1_function_bits = new_desired_input_1_function_bits
        self.update()

    @property
    def desired_output_functionality_bit(self):
        return self._desired_output_functionality_bit
    
    @desired_output_functionality_bit.setter
    def desired_output_functionality_bit(self, new_desired_output_functionality_bit):
        self._desired_output_functionality_bit = new_desired_output_functionality_bit
        self.update()

    @property
    def desired_output_state_control_on_network_lost(self):
        return self._desired_output_state_control_on_network_lost
    
    @desired_output_state_control_on_network_lost.setter
    def desired_output_state_control_on_network_lost(self, new_desired_output_state_control_on_network_lost):
        self._desired_output_state_control_on_network_lost = new_desired_output_state_control_on_network_lost
        self.update()

    @property
    def desired_output_state_on_network_lost(self):
        return self._desired_output_state_on_network_lost
    
    @desired_output_state_on_network_lost.setter
    def desired_output_state_on_network_lost(self, new_desired_output_state_on_network_lost):
        self._desired_output_state_on_network_lost = new_desired_output_state_on_network_lost
        self.update()

    @property
    def desired_read_present_configuration(self):
        return self._desired_read_present_configuration
    
    @desired_read_present_configuration.setter
    def desired_read_present_configuration(self, new_desired_read_present_configuration):
        self._desired_read_present_configuration = new_desired_read_present_configuration
        self.update()

    @property
    def desired_save_configuration(self):
        return self._desired_save_configuration
    
    @desired_save_configuration.setter
    def desired_save_configuration(self, new_desired_save_configuration):
        self._desired_save_configuration = new_desired_save_configuration
        self.update()

    @property
    def desired_binary_input_format(self):
        return self._desired_binary_input_format
    
    @desired_binary_input_format.setter
    def desired_binary_input_format(self, new_desired_binary_input_format):
        self._desired_binary_input_format = new_desired_binary_input_format
        self.update()

    @property
    def desired_binary_output_format(self):
        return self._desired_binary_output_format
    
    @desired_binary_output_format.setter
    def desired_binary_output_format(self, new_desired_binary_output_format):
        self._desired_binary_output_format = new_desired_binary_output_format
        self.update()

    @property
    def desired_binary_endian(self):
        return self._desired_binary_endian
    
    @desired_binary_endian.setter
    def desired_binary_endian(self, new_desired_binary_endian):
        self._desired_binary_endian = new_desired_binary_endian
        self.update()

    @property
    def desired_input_3_active_level(self):
        return self._desired_input_3_active_level
    
    @desired_input_3_active_level.setter
    def desired_input_3_active_level(self, new_desired_input_3_active_level):
        self._desired_input_3_active_level = new_desired_input_3_active_level
        self.update()

    @property
    def desired_input_2_active_level(self):
        return self._desired_input_2_active_level
    
    @desired_input_2_active_level.setter
    def desired_input_2_active_level(self, new_desired_input_2_active_level):
        self._desired_input_2_active_level = new_desired_input_2_active_level
        self.update()

    @property
    def desired_input_1_active_level(self):
        return self._desired_input_1_active_level
    
    @desired_input_1_active_level.setter
    def desired_input_1_active_level(self, new_desired_input_1_active_level):
        self._desired_input_1_active_level = new_desired_input_1_active_level
        self.update()

    @property
    def desired_starting_speed(self):
        return self._desired_starting_speed
    
    @desired_starting_speed.setter
    def desired_starting_speed(self, new_desired_starting_speed):
        if new_desired_starting_speed < 0:
            new_desired_starting_speed += 2**32

        word2 = new_desired_starting_speed // 1000
        word3 = new_desired_starting_speed %  1000
        self._desired_starting_speed = [int(i) for i in "{0:b}".format(word2).zfill(16)] + [int(i) for i in "{0:b}".format(word3).zfill(16)]
        self.update()
        
    @property
    def desired_motors_step_turn(self):
        return self._desired_motors_step_turn
    
    @desired_motors_step_turn.setter
    def desired_motors_step_turn(self, new_desired_motors_step_turn):
        if new_desired_motors_step_turn < 0:
            new_desired_motors_step_turn += 2**16
        self._desired_motors_step_turn = [int(i) for i in "{0:b}".format(new_desired_motors_step_turn).zfill(16)]
        self.update()
        
    @property
    def desired_hybrid_control_gain(self):
        return self._desired_hybrid_control_gain
    
    @desired_hybrid_control_gain.setter
    def desired_hybrid_control_gain(self, new_desired_hybrid_control_gain):
        if new_desired_hybrid_control_gain < 0:
            new_desired_hybrid_control_gain += 2**16
        self._desired_hybrid_control_gain = [int(i) for i in "{0:b}".format(new_desired_hybrid_control_gain).zfill(16)]
        self.update()
        
    @property
    def desired_encoder_pulses_turn(self):
        return self._desired_encoder_pulses_turn
    
    @desired_encoder_pulses_turn.setter
    def desired_encoder_pulses_turn(self, new_desired_encoder_pulses_turn):
        if new_desired_encoder_pulses_turn < 0:
            new_desired_encoder_pulses_turn += 2**16
        self._desired_encoder_pulses_turn = [int(i) for i in "{0:b}".format(new_desired_encoder_pulses_turn).zfill(16)]
        self.update()
        
    @property
    def desired_idle_current_percentage(self):
        return self._desired_idle_current_percentage
    
    @desired_idle_current_percentage.setter
    def desired_idle_current_percentage(self, new_desired_idle_current_percentage):
        if new_desired_idle_current_percentage < 0:
            new_desired_idle_current_percentage += 2**16
        self._desired_idle_current_percentage = [int(i) for i in "{0:b}".format(new_desired_idle_current_percentage).zfill(16)]
        self.update()
        
    @property
    def desired_motor_current(self):
        return self._desired_motor_current
    
    @desired_motor_current.setter
    def desired_motor_current(self, new_desired_motor_current):
        if new_desired_motor_current < 0:
            new_desired_motor_current += 2**16
        self._desired_motor_current = [int(i) for i in "{0:b}".format(new_desired_motor_current).zfill(16)]
        self.update()
        
    @property
    def desired_current_loop_gain(self):
        return self._desired_current_loop_gain
    
    @desired_current_loop_gain.setter
    def desired_current_loop_gain(self, new_desired_current_loop_gain):
        if new_desired_current_loop_gain < 0:
            new_desired_current_loop_gain += 2**16
        self._desired_current_loop_gain = [int(i) for i in "{0:b}".format(new_desired_current_loop_gain).zfill(16)]
        self.update()

class VirtualAxis(threading.Thread):
    def __init__(self, running, interval, t0, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = [(0,0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.verbose = verbose
        self.pos = 0
        self.vel = 0
        
    def run(self):
        while self.running.is_set():
            t = time.time() - self.t0
            self.pos, self.vel = self.get_ref(t)
            if self.verbose:
                print(t, self.pos, self.vel)
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

class AMCIDriver(threading.Thread):
    def __init__(self, hostname, running, virtual_axis, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5, homing_slow_speed=200, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = virtual_axis
        self.connected = connected

        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)

        self.verbose = verbose
        
        self.EIP = ethernetip.EtherNetIP(self.hostname)
        self.C1 = self.EIP.explicit_conn(self.hostname)

        pkt = self.C1.listID()
        if pkt is not None:
            print("Product name: ", pkt.product_name.decode())

        inputsize = 20
        outputsize = 20

        # configure i/o
        # print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
        self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 100, self.C1)
        self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 150, self.C1)
        
    def run(self):
        self.EIP.startIO()
        self.C1.registerSession()
        
        print(self.initial_settings.ints_to_send)
        return_to_command_mode = self.get_return_to_command_mode_command()
        print(return_to_command_mode.ints_to_send)
        preset_pos = self.get_preset_position_command(0)
        print(preset_pos.ints_to_send)

        # data = [-32768,0,0,1,10000,0,1000,30,30,5] ## Configuracion inicial
        # output  = b''
        # for i in data:
        #     output += struct.pack("h", i)
        # #output  = b'\x00\x80\x00\x00\x00\x00\x01\x00\xe8\x03\x00\x00\xe8\x03\x1e\x00\x1e\x00\x05\x00'
        # sen = self.C1.setAttrSingle(0x04, 150, 0x03, output)
        # print(sen, len(output), output)

        # data = [0,-32768,0,0,0,0,0,0,0,0] ## Return to command mode
        # output  = b''
        # for i in data:
        #     output += struct.pack("h", i)
        # #output  = b'\x00\x80\x00\x00\x00\x00\x01\x00\xe8\x03\x00\x00\xe8\x03\x1e\x00\x1e\x00\x05\x00'
        # sen = self.C1.setAttrSingle(0x04, 150, 0x03, output)
        # print(sen, len(output), output)

        # data = [512,-32768,0,0,0,0,0,0,0,0] ## Preset position
        # output  = b''
        # for i in data:
        #     output += struct.pack("h", i)
        # #output  = b'\x00\x80\x00\x00\x00\x00\x01\x00\xe8\x03\x00\x00\xe8\x03\x1e\x00\x1e\x00\x05\x00'
        # sen = self.C1.setAttrSingle(0x04, 150, 0x03, output)
        # print(sen, len(output), output)

        # data = [1,-32768,2,500,10,0,1000,1000,30,5] ## Jog move
        # output  = b''
        # for i in data:
        #     output += struct.pack("h", i)
        # #output  = b'\x00\x80\x00\x00\x00\x00\x01\x00\xe8\x03\x00\x00\xe8\x03\x1e\x00\x1e\x00\x05\x00'
        # sen = self.C1.setAttrSingle(0x04, 150, 0x03, output)
        # print(sen, len(output), output)

        # return_to_command_mode = self.get_return_to_command_mode_command()
        # sen = self.send_data(return_to_command_mode.list_to_send)
        # print(sen, len(return_to_command_mode.list_to_send))

        # preset_pos = self.get_preset_position_command(0)
        # sen = self.send_data(preset_pos.list_to_send)
        # print(sen, len(preset_pos.list_to_send))

        # jog_ccw = self.get_ccw_jog_command()
        # sen = self.send_data(jog_ccw.list_to_send)
        # print(sen, len(jog_ccw.list_to_send))

        synchrostep_command = self.get_synchrostep_move_command(0, 0, speed=0, acceleration=20, deceleration=20, proportional_coefficient=1, network_delay=0)

        self.C1.outAssem = synchrostep_command.list_to_send

        return

        self.C1.sendFwdOpenReq(100, 150, 0x01, torpi=5, otrpi=5)
        self.C1.produce()

        while self.running.is_set():
            time.sleep(0.05)
            self.read_input()
            self.set_output(self.virtual_axis.pos, self.virtual_axis.vel)
            
        self.C1.stopProduce()
        self.C1.sendFwdCloseReq(100, 150, 0x01)
        self.EIP.stopIO()

    def get_ccw_jog_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1):
        command = Command(jog_ccw=1, name='CCW Jog')
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

        return command

    def read_input(self, read_output=False):
        words = []
        for w in range(20):
            words.append(int("".join(["1" if self.C1.inAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(20)
        struct.pack_into('20B', b, 0, *words)
        [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9] = struct.unpack('<10H', b)
        print(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9)
        
        if read_output:
            words2 = []
            for w in range(20):
                words2.append(int("".join(["1" if self.C1.outAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b2 = bytearray(20)
            struct.pack_into('20B', b2, 0, *words2)
            [o0, o1, o2, o3, o4, o5, o6, o7, o8, o9] = struct.unpack('<10H', b2)
            print(o0, o1, o2, o3, o4, o5, o6, o7, o8, o9)

    def set_output(self, pos_value, vel_value):
        if pos_value < 0:
            pos_value += 2*(2**31)
        pos_in_bits = "{0:b}".format(pos_value).zfill(32)
        
        if vel_value < 0:
            vel_value += 2*(2**31)
        speed_in_bits = "{0:b}".format(vel_value).zfill(32)

        w2 = pos_in_bits[16::-1]
        for j in range(8):
            self.C1.outAssem[2*8+j] = int(w2[j]) == 1
        w3 = pos_in_bits[:16:-1]
        for j in range(8):
            self.C1.outAssem[3*8+j] = int(w3[j]) == 1
        w4 = speed_in_bits[16::-1]
        for j in range(8):
            self.C1.outAssem[4*8+j] = int(w4[j]) == 1
        w5 = speed_in_bits[:16:-1]
        for j in range(8):
            self.C1.outAssem[5*8+j] = int(w5[j]) == 1

    def send_data(self, data):
        output = b""
        cnt = 0
        val = 0
        for bit in data:
            if bit is True:
                val += 1 << cnt
            cnt += 1
            if cnt == 8:
                cnt = 0
                output += struct.pack("B", val)
                val = 0
        # print(output[13:14], len(output))
        #output = b'\x80\x00\x00\x00\x00\x00\x00\x01\x03\xe8\x00\x00\x03\xe8\x00\x1e\x00\x1e\x00\x05'
        output  = b'\x00\x80\x00\x00\x00\x00\x01\x00\xe8\x03\x00\x00\xe8\x03\x1e\x00\x1e\x00\x05\x00'
        return self.C1.setAttrSingle(0x04, 150, 0x03, output)

    def get_synchrostep_move_command(self, position, direction, speed=200, acceleration=100, deceleration=100, proportional_coefficient=1, network_delay=0):
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
        
        command.desired_command_word_2 = int(pos_in_bits[16:], 2)
        command.desired_command_word_3 = int(pos_in_bits[:16], 2)
        command.desired_command_word_4 = int(speed_in_bits[16:], 2)
        command.desired_command_word_5 = int(speed_in_bits[:16], 2)

        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = proportional_coefficient
        command.desired_command_word_9 = network_delay

        return command

class FlowControllerDriver(threading.Thread):
    def __init__(self, hostname, running, virtual_axis, connected=True, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = virtual_axis
        self.connected = connected

        self.verbose = verbose
        
        self.EIP = ethernetip.EtherNetIP(self.hostname)
        self.C1 = self.EIP.explicit_conn(self.hostname)

        pkt = self.C1.listID()
        if pkt is not None:
            print("Product name: ", pkt.product_name.decode())

        inputsize = 26
        outputsize = 4

        # configure i/o
        # print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
        self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 101, self.C1)
        self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 100, self.C1)

    def run(self):
        self.EIP.startIO()
        self.C1.registerSession()
        
        data = 15
        sen = self.send_ref(data)
        print(sen)
    
    def send_ref(self, value):
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        return self.C1.setAttrSingle(0x04, 100, 0x03, b)

        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])

        # output = b""
        # cnt = 0
        # val = 0
        # for bit in data:
        #     if bit is True:
        #         val += 1 << cnt
        #     cnt += 1
        #     if cnt == 8:
        #         cnt = 0
        #         output += struct.pack("B", val)
        #         val = 0
        # print(output[13:14], len(output))
        #output = b'\x80\x00\x00\x00\x00\x00\x00\x01\x03\xe8\x00\x00\x03\xe8\x00\x1e\x00\x1e\x00\x05'
        

if __name__ == "__main__":
    host = "192.168.2.103"
    host2 = "192.168.2.101"
    event = threading.Event()
    event.set()

    t0 = time.time()
    virtual_axis = VirtualAxis(event, 10, t0)
    
    axis = AMCIDriver(host, event, virtual_axis, connected=False)
    axis.start()

    # valve = FlowControllerDriver(host2, event, virtual_axis, connected=True)
    # valve.start()

    # c = Command(preset_encoder=1)
    # print(c.list_to_send)

    # s = Setting(disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5)
    # print(s.list_to_send)

    # def get_synchrostep_move_command(position, direction, speed=200, acceleration=100, deceleration=100, proportional_coefficient=1, network_delay=0):
    #     #print(target_position, programmed_speed)
    #     command = Command(virtual_position_follower=1, name='Synchrostep Move')
    #     if direction:
    #         command.desired_jog_cw = 1
    #         command.desired_jog_ccw = 0
    #     else:
    #         command.desired_jog_cw = 0
    #         command.desired_jog_ccw = 1

    #     if position < 0:
    #         position += 2*(2**31)
    #         pos_in_bits = "{0:b}".format(position).zfill(32)
    #     else:
    #         pos_in_bits = "{0:b}".format(position).zfill(32)
        
    #     if speed < 0:
    #         speed += 2*(2**31)
    #         speed_in_bits = "{0:b}".format(speed).zfill(32)
    #     else:
    #         speed_in_bits = "{0:b}".format(speed).zfill(32)
        
    #     command.desired_command_word_2 = int(pos_in_bits[16:], 2)
    #     command.desired_command_word_3 = int(pos_in_bits[:16], 2)
    #     command.desired_command_word_4 = int(speed_in_bits[16:], 2)
    #     command.desired_command_word_5 = int(speed_in_bits[:16], 2)
        
    #     command.desired_command_word_6 = acceleration
    #     command.desired_command_word_7 = deceleration
    #     command.desired_command_word_8 = proportional_coefficient
    #     command.desired_command_word_9 = network_delay

    #     return command

    # s = get_synchrostep_move_command(2, 0, speed=-2, acceleration=20, deceleration=20, proportional_coefficient=1, network_delay=0)
    
    # output = b""
    # cnt = 0
    # val = 0
    # for bit in s.list_to_send:
    #     if bit is True:
    #         val += 1 << cnt
    #     cnt += 1
    #     if cnt == 8:
    #         cnt = 0
    #         output += struct.pack("B", val)
    #         val = 0

    # print(output)