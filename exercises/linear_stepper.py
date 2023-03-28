import struct
import threading
import time

from numpy import *
import sys

import pandas as pd
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import lib.ethernet_ip.ethernetip as ethernetip

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
            self.pos, self.vel = 0, 0
            if self.verbose:
                print(t, self.pos, self.vel)
            time.sleep(self.interval)


class AMCIDriver(threading.Thread):
    def __init__(self, EIP, hostname, running, virtual_axis, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5, homing_slow_speed=200, verbose=False, virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True, virtual_axis_proportional_coef=1):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.EIP = EIP
        self.hostname = hostname
        self.running = running
        self.virtual_axis = virtual_axis
        self.connected = connected
        self.acc = virtual_axis_follow_acceleration
        self.dec = virtual_axis_follow_deceleration
        self.virtual_axis_proportional_coef = virtual_axis_proportional_coef
        self.home = home

        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)

        self.verbose = verbose
        self.init_params()
        #self.programming_assembly = False
        self.homing_event = threading.Event()
        self.homing_move_target = 0
        self.fast_ccw_limit_homing = False
        self.slow_ccw_limit_homing = False
        self.homing_movements = []

        if self.connected:
            self.C1 = self.EIP.explicit_conn(self.hostname)
            self.C1.outAssem = [0 for i in range(20*8)]
            self.C1.inAssem = [0 for i in range(20*8)]
            
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
        if self.connected:
            self.C1.registerSession()

            sen = self.send_data(self.initial_settings.get_bytes_to_send())
            time.sleep(0.1)

            return_to_command_mode = self.get_return_to_command_mode_command()
            sen = self.send_data(return_to_command_mode.get_bytes_to_send())
            time.sleep(0.1)

            preset_pos = self.get_preset_position_command(0)
            sen = self.send_data(preset_pos.get_bytes_to_send())
            time.sleep(0.1)

            jog = self.get_ccw_jog_command(motor_current=6)
            sen = self.send_data(jog.get_bytes_to_send())
            time.sleep(0.1)


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
        ccw_jog = self.get_ccw_jog_command(programmed_speed=1000)
        self.C1.outAssem = ccw_jog.get_list_to_send()

    def get_reset_errors_command(self):
        command = Command(reset_errors=1, name='Reset Errors')
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
                words.append(int("".join(["1" if self.C1.inAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b = bytearray(20)
            struct.pack_into('20B', b, 0, *words)
        else:
            status = self.C1.getAttrSingle(0x04, 100, 0x03)
            b = status[1]
        
        [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9] = struct.unpack('<10H', b)
        #print(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9)
        
        if read_output:
            words2 = []
            for w in range(20):
                words2.append(int("".join(["1" if self.C1.outAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b2 = bytearray(20)
            struct.pack_into('20B', b2, 0, *words2)
            [o0, o1, o2, o3, o4, o5, o6, o7, o8, o9] = struct.unpack('<10H', b2)
            if self.verbose:
                print(o0, o1, o2, o3, o4, o5, o6, o7, o8, o9)
        
        return [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9]

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
        self.C1.outAssem = c.get_list_to_send()
        time.sleep(0.01)
        for step in steps:
            c = self.get_assembled_segment_command(target_position=step['pos'], programmed_speed=step['speed'], acceleration=step['acc'], deceleration=step['dec'], acceleration_jerk=step['jerk'])
            self.C1.outAssem = c.get_list_to_send()
            time.sleep(0.01)
            c = self.get_program_assembled_command()
            self.C1.outAssem = c.get_list_to_send()
            time.sleep(0.01)
        c = self.get_return_to_command_mode_command()
        self.C1.outAssem = c.get_list_to_send()
        time.sleep(0.01)
        c = self.get_run_assembled_move_command(motor_current=motor_current, blend_direction=blend_direction, dwell_move=dwell_move, dwell_time=dwell_time)
        self.C1.outAssem = c.get_list_to_send()
        time.sleep(0.01)

    def process_incoming_data(self, data):
        #print(data)
        word0 = format(data[0], 'b').zfill(16)
        word1 = format(data[1], 'b').zfill(16)
        mode = int(word0[0], 2)

        if mode != self.mode_select_bit:
            if mode:
                #print('Changed to configuration mode')
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
            # if input_error != self.input_error:
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
            if limit_condition:
                #print('limit condition')
                if self.fast_ccw_limit_homing:
                    #print('was moving fast')
                    # self.request_write_reset_errors()
                    self.slow_ccw_limit_homing = True
                    self.fast_ccw_limit_homing = False
                    steps = [{'pos': 1000, 'speed': 500, 'acc': 100, 'dec': 100, 'jerk': 0}, 
                                {'pos': -1000, 'speed': 200, 'acc': 100, 'dec': 100, 'jerk': 0}]
                    self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100)
                    # self.request_write_relative_move(target_position=1000, programmed_speed=500)
                    # self.request_write_ccw_jog(programmed_speed=200)
                    # print('Step 1')
                elif self.slow_ccw_limit_homing:
                    #print('was moving slow')
                    c = self.get_reset_errors_command()
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)
                    self.slow_ccw_limit_homing = False
                    c = self.get_preset_position_command(-200)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

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

            pos1 = data[2]
            pos2 = data[3]
            if pos1 > 2**15:
                pos1 = pos1 - 2**16
            if pos2 > 2**15:
                pos2 = pos2 - 2**16
            motor_position = pos1*1000 + pos2
            # if motor_position != self.motor_position:
                # self.motor_position_signal.emit(motor_position)
            self.motor_position = motor_position
            # if self.verbose:
            #     print(self.motor_position)
            encoder_position = data[4]*1000 + data[5]
            # if encoder_position != self.encoder_position:
                # self.encoder_position_signal.emit(encoder_position)
            self.encoder_position = encoder_position
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
                self.C1.outAssem[16*2+i*8+j] = int(words[i][j]) == 1

    def send_data(self, data):
        return self.C1.setAttrSingle(0x04, 150, 0x03, data)

    def get_synchrostep_move_command(self, position, direction, speed=200, acceleration=50, deceleration=50, proportional_coefficient=1, network_delay=0):
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


if __name__ == "__main__":
    run = threading.Event()
    run.set()
    t0 = time.time()

    host = "192.168.2.10"
    connections = "192.168.2.103"
    EIP = ethernetip.EtherNetIP(host)

    va = VirtualAxis(run, 0.1, t0)
    va.start()
    motor = AMCIDriver(EIP, connections, run, va, connected=True, motor_current=6)
    motor.start()