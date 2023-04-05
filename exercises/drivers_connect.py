import socket
import sys

import pandas as pd
import sounddevice as sd
from scipy import signal
from librosa import yin, note_to_hz
from scipy.io.wavfile import write

sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
import lib.ethernet_ip.ethernetip as ethernetip
from utils.motor_route import *
from utils.driver_fingers import FingersDriver
import struct
import time
import numpy as np

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
        # while self.running.is_set():
        #     t = time.time() - self.t0
        #     self.pos = int(200 * np.sin(2*np.pi * self.f * t))
        #     self.vel = int(200 * 2*np.pi*self.f * np.cos(2*np.pi * self.f * t))
        #     time.sleep(0.01)
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

class AMCIDriver(threading.Thread):
    def __init__(self, EIP, hostname, running, virtual_axis, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=1, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=40, current_loop_gain=5, homing_slow_speed=200, verbose=False, virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True, virtual_axis_proportional_coef=1, Kp=0, Ki=5, Kd=0.01):
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
        self.forced_break = False
        self.motor_current = motor_current

        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)
        #print(starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)
        
        self.verbose = verbose
        self.init_params()
        #self.programming_assembly = False
        self.homing_event = threading.Event()
        self.homing_move_target = 0
        self.fast_ccw_limit_homing = False
        self.slow_ccw_limit_homing = False
        self.fast_cw_limit_homing = False
        self.slow_cw_limit_homing = False
        self.homing_movements = []

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
            data = self.read_input(explicit=True)
            self.process_incoming_data(data)
            

            return_to_command_mode = self.get_return_to_command_mode_command()
            sen = self.send_data(return_to_command_mode.get_bytes_to_send())
            time.sleep(0.1)

            # preset_pos = self.get_preset_position_command(0)
            # sen = self.send_data(preset_pos.get_bytes_to_send())
            # time.sleep(0.1)

            # preset_encoder = self.get_preset_encoder_position_command(0)
            # sen = self.send_data(preset_encoder.get_bytes_to_send())
            # time.sleep(0.1)
            # self.virtual_axis_proportional_coef
            synchrostep_command = self.get_synchrostep_move_command(0, 0, speed=0, acceleration=self.acc, deceleration=self.dec, proportional_coefficient=self.virtual_axis_proportional_coef, network_delay=0, encoder=False)
            #print(synchrostep_command.get_bytes_to_send())
            # sen = self.send_data(synchrostep_command.get_bytes_to_send())
            # time.sleep(0.1)

            #self.C1.outAssem = synchrostep_command.get_list_to_send()

            self.C1.sendFwdOpenReq(100, 150, 110, torpi=50, otrpi=50, priority=ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH)
            self.C1.produce()
            
            stop = self.get_immediate_stop_command()
            self.C1.outAssem = stop.get_list_to_send()
            time.sleep(0.1)

            # input()
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
                    c = self.get_relative_move_command(565, programmed_speed=1000, acceleration=self.acc, deceleration=self.dec, motor_current=self.motor_current)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(2)

                    c = self.get_reset_errors_command()
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_position_command(0)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_encoder_position_command(0)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_return_to_command_mode_command()
                    self.C1.outAssem = c.get_list_to_send()
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
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(2)

                    c = self.get_reset_errors_command()
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_position_command(0)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_preset_encoder_position_command(0)
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    c = self.get_return_to_command_mode_command()
                    self.C1.outAssem = c.get_list_to_send()
                    time.sleep(0.1)

                    if self.verbose:
                        print('Homed')
                    # print("Buscando CW")
                    # self.alpha_home_switch_pos = 1437
                    # if self.verbose:
                    #     print("Buscando home clockwise")
                    # cw_jog = self.get_cw_jog_command(programmed_speed=500)
                    # stop = self.get_immediate_stop_command()
                    # self.C1.outAssem = stop.get_list_to_send()
                    # time.sleep(0.5)
                    # # self.C1.outAssem = cw_jog.get_list_to_send()
                    # # while True:
                    # #     time.sleep(1)
                    # self.cw_find_home_to_limit()
                    # self.fast_cw_limit_homing = True
                    # while self.fast_cw_limit_homing or self.slow_cw_limit_homing:
                    #     if self.verbose:
                    #         print('Still not homed3...')
                    #     time.sleep(0.5)
                    #     #break
                    #     data = self.read_input()
                    #     self.process_incoming_data(data)
                    # c = self.get_relative_move_command(-self.alpha_home_switch_pos, programmed_speed=1000, acceleration=self.acc, deceleration=self.dec)
                    # self.C1.outAssem = c.get_list_to_send()
                    # time.sleep(2)
                    # c = self.get_return_to_command_mode_command()
                    # self.C1.outAssem = c.get_list_to_send()
                    # time.sleep(0.1)
                    # if self.verbose:
                    #     print('Homed')
                # else:
                #     if self.verbose:
                #         print('Homing through user input')
                #     self.homing_event.set()
                #     old_pos = self.homing_move_target
                #     while self.homing_event.is_set():
                #         if old_pos != alpha_angle_to_units(self.homing_move_target):
                #             if self.verbose:
                #                 print(f'Moving to {self.homing_move_target}')
                #             c = self.get_relative_move_command(alpha_angle_to_units(self.homing_move_target)-old_pos, programmed_speed=200)
                #             self.C1.outAssem = c.get_list_to_send()
                #             old_pos = alpha_angle_to_units(self.h15oming_move_target)
                #         time.sleep(0.25)
                #         c = self.get_return_to_command_mode_command()
                #         self.C1.outAssem = c.get_list_to_send()
                #     if self.verbose:
                #         print('Homing done')
                #     time.sleep(0.25)
                #     c = self.get_preset_position_command(0)
                #     self.C1.outAssem = c.get_list_to_send()
                #     time.sleep(0.1)
            else:
                c = self.get_preset_position_command(0)
                self.C1.outAssem = c.get_list_to_send()
                time.sleep(0.1)

            self.C1.outAssem = synchrostep_command.get_list_to_send()

            while self.running.is_set():
                if self.forced_break:
                    break
                time.sleep(self.Ts)
                data = self.read_input(read_output=False)
                # if self.verbose:
                #     print(data)
                self.process_incoming_data(data)
                corrected_pos, corrected_vel = self.pid_control(self.virtual_axis.pos, self.virtual_axis.vel)
                if type(corrected_pos) == int and type(corrected_vel) == int:
                    try:
                        self.set_output(-corrected_pos, -corrected_vel)
                        #print(corrected_pos, corrected_vel, self.encoder_position, self.motor_position)
                    except:
                        print(f'Error en referencia: {self.virtual_axis.pos}, {self.virtual_axis.vel}')
                
            self.C1.stopProduce()
            self.C1.sendFwdCloseReq(100, 150, 110)

    def pid_control(self, ref_pos, ref_vel):
        SP = ref_pos
        CV = self.encoder_position
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
        ccw_jog = self.get_ccw_jog_command(programmed_speed=400, motor_current=self.motor_current)
        self.C1.outAssem = ccw_jog.get_list_to_send()
    
    def cw_find_home_to_limit(self):
        self.fast_cw_limit_homing = True
        cw_jog = self.get_cw_jog_command(programmed_speed=4000, motor_current=self.motor_current)
        self.C1.outAssem = cw_jog.get_list_to_send()

    def get_reset_errors_command(self):
        command = Command(reset_errors=1, enable_driver=0, clear_driver_fault=1, name='Reset Errors')
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
            # if self.verbose:
            #     print(o0, o1, o2, o3, o4, o5, o6, o7, o8, o9)
        
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
        time.sleep(0.1)
        for step in steps:
            c = self.get_assembled_segment_command(target_position=step['pos'], programmed_speed=step['speed'], acceleration=step['acc'], deceleration=step['dec'], acceleration_jerk=step['jerk'], motor_current=motor_current)
            self.C1.outAssem = c.get_list_to_send()
            time.sleep(0.1)
            c = self.get_program_assembled_command()
            self.C1.outAssem = c.get_list_to_send()
            time.sleep(0.1)
        c = self.get_return_to_command_mode_command()
        self.C1.outAssem = c.get_list_to_send()
        time.sleep(0.1)
        c = self.get_run_assembled_move_command(motor_current=motor_current, blend_direction=blend_direction, dwell_move=dwell_move, dwell_time=dwell_time)
        self.C1.outAssem = c.get_list_to_send()
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
                                {'pos': -400, 'speed': 100, 'acc': 400, 'dec': 400, 'jerk': 0}]
                    self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100, motor_current=self.motor_current)
                    #print("aqui estoy", self.motor_current)
                    # self.request_write_relative_move(target_position=1000, programmed_speed=500)
                    # self.request_write_ccw_jog(programmed_speed=200)
                    # print('Step 1')
                elif self.slow_ccw_limit_homing:
                    if self.verbose:
                        print('was moving slow ccw')
                    self.slow_ccw_limit_homing = False
                    # c = self.get_preset_position_command(200)
                    # #c2 = self.get_preset_encoder_position_command(-800)
                    # self.C1.outAssem = c.get_list_to_send()
                    # time.sleep(0.1)
                    # print(1)
                    # data = self.read_input()
                    # self.process_incoming_data(data)
                    # print(2)
                    # #self.C1.outAssem = c2.get_list_to_send()
                    # #time.sleep(0.1)
                elif self.fast_cw_limit_homing:
                    if self.verbose:
                        print('was moving fast cw')
                    # self.request_write_reset_errors()
                    self.slow_cw_limit_homing = True
                    self.fast_cw_limit_homing = False
                    steps = [{'pos': -4000, 'speed': 2000, 'acc': 400, 'dec': 400, 'jerk': 0}, 
                                {'pos': 4000, 'speed': 800, 'acc': 400, 'dec': 400, 'jerk': 0}]
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
            self.motor_position = -motor_position
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
            self.encoder_position = -encoder_position
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

class FlowControllerDriver(threading.Thread):
    def __init__(self, EIP, hostname, running, virtual_axis, connected=True, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = virtual_axis
        self.connected = connected

        self.verbose = verbose
        self.mass_flow_reading = 0
        self.vol_flow_reading = 0
        self.temperature_reading = 0
        self.absolute_preasure_reading = 0
        self.mass_flow_set_point_reading = 0
        
        self.EIP = EIP
        if self.connected:
            self.C1 = self.EIP.explicit_conn(self.hostname)
            self.C1.outAssem = [0 for i in range(26*8)]
            self.C1.inAssem = [0 for i in range(4*8)]

            pkt = self.C1.listID()
            if pkt is not None:
                print("Product name: ", pkt.product_name.decode())

            inputsize = 26
            outputsize = 4

            self.C1.outAssem = [False for i in range(8*4)]

            # configure i/o
            # print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
            self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 101, self.C1)
            self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 100, self.C1)

    def run(self):
        if self.connected:
            self.C1.registerSession()
            
            self.C1.sendFwdOpenReq(101, 100, 0x6e, torpi=50, otrpi=50, priority=ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH)
            self.C1.produce()

            while self.running.is_set():
                time.sleep(0.01)
                self.read_input(read_output=False)
                self.set_output(self.virtual_axis.flow)
                
            self.C1.stopProduce()
            self.C1.sendFwdCloseReq(101, 100, 0x6e)
    
    def send_ref(self, value):
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        return self.C1.setAttrSingle(0x04, 100, 0x03, b)
    
    def set_output(self, value):
        value = min(50, max(0, value))
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
        
        #self.C1.outAssem = [0 for i in range(4*8)]
        for i in range(4):
            for j in range(8):
                self.C1.outAssem[i*8+j] = int(words[i][j]) == 1

    def read_input(self, read_output=False):
        words = []
        for w in range(26):
            words.append(int("".join(["1" if self.C1.inAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(26)
        struct.pack_into('26B', b, 0, *words)
        [g, s, ap, ft, vf, mf, mfsp] = struct.unpack('<HIfffff', b)

        self.mass_flow_reading = mf
        self.vol_flow_reading = vf
        self.temperature_reading = ft
        self.absolute_preasure_reading = ap
        self.mass_flow_set_point_reading = mfsp

        # if s != 0:
        #     print(s)
        if read_output:
            words2 = []
            for w in range(4):
                words2.append(int("".join(["1" if self.C1.outAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            
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

class VirtualFlow(threading.Thread):
    def __init__(self, running, interval, t0, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = [(0,0)]
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.verbose = verbose
        self.flow = 0
        self.vibrato_amp = 0
        self.vibrato_freq = 0
        
    def run(self):
        # f = 1
        # while self.running.is_set():
        #     t = time.time() - self.t0
        #     self.flow = int(15 + 15 * np.sin(2*np.pi * f * t))
        #     time.sleep(0.01)
        while self.running.is_set():
            t = time.time() - self.t0
            self.flow = self.get_ref(t)
            if not (type(self.flow) == float or type(self.flow) == int or type(self.flow) == np.float64):
                print(type(self.flow), self.flow)
            if self.verbose:
                print(t, self.flow)
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

    def stop(self):
        self.ref = [(0,0)]

class VirtualFingers(threading.Thread):
    def __init__(self, running, interval, t0, fingers_driver, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = []
        self.t0 = t0
        self.verbose = verbose
        self.note = 'D3'
        self.interval = interval
        self.fingers_driver = fingers_driver
        self.changeEvent = threading.Event()
        
    def run(self):
        note_time = 0
        next_note_time = 0
        while self.running.is_set():
            if len(self.ref) > 0:
                next_note_time, self.note = self.ref.pop(0)
                sleep_time = next_note_time - note_time
                self.fingers_driver.request_finger_action(self.note)
                if self.verbose:
                    print(sleep_time, self.note)
            else:
                sleep_time = self.interval
            self.changeEvent.wait(timeout=sleep_time)
            self.changeEvent.clear()
            note_time = next_note_time
    
    def stop(self):
        self.ref = []
        self.changeEvent.set()

class PreasureSensor(threading.Thread):
    def __init__(self, EIP, hostname, running, connected=False, verbose=False):
        threading.Thread.__init__(self)
        self.running = running
        self.connected = connected
        self.verbose = verbose
        self.preasure = 0
        self.hostname = hostname
        self.verbose = verbose
        
        self.EIP = EIP
        if self.connected:
            self.C1 = self.EIP.explicit_conn(self.hostname)
            self.C1.inAssem = [0 for i in range(4*8)]

            pkt = self.C1.listID()
            if pkt is not None:
                print("Product name: ", pkt.product_name.decode())

            inputsize = 10
            outputsize = 4

            # self.C1.outAssem = [False for i in range(8*4)]

            # configure i/o
            # print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
            self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 101, self.C1)
            self.EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 100, self.C1)

    def run(self):
        if self.connected:
            self.C1.registerSession()
            
            self.C1.sendFwdOpenReq(101, 100, 0x6e, torpi=50, otrpi=50)#, priority=ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH)
            self.C1.produce()

            while self.running.is_set():
                time.sleep(0.01)
                self.read_input()
                
            self.C1.stopProduce()
            self.C1.sendFwdCloseReq(101, 100, 0x6e)

    def read_input(self):
        words = []
        for w in range(10):
            words.append(int("".join(["1" if self.C1.inAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(10)
        struct.pack_into('10B', b, 0, *words)
        [g, s, ap] = struct.unpack('<HIf', b)

        self.preasure = ap

class Microphone(threading.Thread):
    def __init__(self, running, connected=False, verbose=False):
        threading.Thread.__init__(self)
        self.running = running
        self.connected = connected
        self.verbose = verbose
        self.pitch = 0
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
        # self.print_i = (self.print_i + 1) % 20
        # print(self.print_i)
        if status:
            print('Status:', status)
        #senal_filtrada1 = signal.lfilter(self.flt, self.A, indata)
        #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)
        self.last_mic_data = np.hstack((self.last_mic_data, np.transpose(indata)[0]))
        self.last_mic_data = self.last_mic_data[-self.max_num_points:]
        # if self.saving:
        #     self.data = np.hstack((self.data, np.transpose(indata)[0]))
            #print(self.data.size)

    def start_saving(self):
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
            with sd.InputStream(samplerate=self.sr, channels=1, callback=self.micCallback, device=0, latency='high'):#,  blocksize=300000): #, latency='high'
                while self.running.is_set():
                    sd.sleep(50)
                    #pitches, harmonic_rates, argmins, times = compute_yin(self.last_mic_data, self.sr, f0_max=2000)#, w_len=int(len(self.last_mic_data)-1), harmo_thresh=0.1,f0_max=self.sr/2, w_step=int(len(self.last_mic_data)-1)) 
                    #senal_filtrada1 = signal.lfilter(self.flt, self.A, self.last_mic_data)
                    #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)

                    pitches = yin(self.last_mic_data, sr=self.sr, fmin=100, fmax=note_to_hz('C7'), trough_threshold=0.1) #, trough_threshold=0.0001)]
                    #print(pitches[-1])
                    #print(1/(self.last_mic_data.shape[0]*(1/self.sr)), pitches[-1])
                    #compute_yin() NUT = 1
                    # N = 44100*1
                    # T = 1/44100
                    # U = 1
                    #print(pitches, len(self.last_mic_data))
                    self.pitch = pitches[-1]
                    #print(self.pitch)
                    #self.last_mic_data = np.array([])
                    #self.callback.doc.add_next_tick_callback(partial(self.callback.update2, self.last))

            
            print("Mic thread killed")

class Musician(threading.Thread):
    def __init__(self, host, connections, running, interval=0.01, home=True, x_connect=True, z_connect=True, alpha_connect=True, flow_connect=True, fingers_connect=True, preasure_sensor_connect=True):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.t0 = time.time()
        self.host = host
        self.running = running
        self.interval = interval
        self.x_connect = x_connect
        self.z_connect = z_connect
        self.alpha_connect = alpha_connect
        self.flow_connect = flow_connect
        self.fingers_connect = fingers_connect
        self.preasure_sensor_connect = preasure_sensor_connect
        self.connections = connections
        self.home = home
        self.instrument = 'flute'
        self.x_virtual_axis = VirtualAxis(running, interval, self.t0, verbose=False)
        self.z_virtual_axis = VirtualAxis(running, interval, self.t0, verbose=False)
        self.alpha_virtual_axis = VirtualAxis(running, interval, self.t0, verbose=False)
        self.virtual_flow = VirtualFlow(running, interval, self.t0, verbose=False)

        if x_connect or z_connect or alpha_connect or flow_connect or preasure_sensor_connect:
            self.EIP = ethernetip.EtherNetIP(self.host)
        else:
            self.EIP = None

        self.x_driver = AMCIDriver(self.EIP, connections[0], running, self.x_virtual_axis, connected=self.x_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=5, Kd=0.01)
        
        self.z_driver = AMCIDriver(self.EIP, connections[1], running, self.z_virtual_axis, connected=self.z_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=5, Kd=0.01)
        
        self.alpha_driver = AMCIDriver(self.EIP, connections[2], running, self.alpha_virtual_axis, connected=self.alpha_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=home, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=1, Kd=0.01)

        self.flow_driver = FlowControllerDriver(self.EIP, connections[3], running, self.virtual_flow, connected=self.flow_connect, verbose=False) 
        try:
            self.fingers_driver = FingersDriver('/dev/ttyACM0', running, connected=self.fingers_connect, verbose=False)
        except:
            try:
                self.fingers_driver = FingersDriver('/dev/ttyACM1', running, connected=self.fingers_connect, verbose=False)
            except:
                raise Exception('Finger Driver not connected')
        self.virtual_fingers = VirtualFingers(running, 0.05, self.t0, self.fingers_driver, verbose=True)

        #self.pressure_driver = pressure_driver host, running, connected=True
        self.preasure_sensor = PreasureSensor(self.EIP, connections[4], running, connected=self.preasure_sensor_connect, verbose=False)
        self.microphone = Microphone(running, connected=True, verbose=False)

        self.memory = Memory(self.running, self.x_driver, self.z_driver, self.alpha_driver, self.flow_driver, self.preasure_sensor, self.x_virtual_axis, self.z_virtual_axis, self.alpha_virtual_axis, self.virtual_flow, self.microphone, windowWidth=200, interval=0.05)

        self.loaded_route_x = []
        self.loaded_route_z = []
        self.loaded_route_alpha = []
        self.loaded_route_flow = []
        self.loaded_route_notes = []
    
    def run(self):
        if not self.EIP is None:
            self.EIP.startIO()

        self.memory.start()
        self.x_virtual_axis.start()
        self.z_virtual_axis.start()
        self.alpha_virtual_axis.start()
        self.virtual_flow.start()
        self.fingers_driver.start()
        self.virtual_fingers.start()
        
        self.x_driver.start()
        self.z_driver.start()
        self.alpha_driver.start()
        self.flow_driver.start()
        self.fingers_driver.start()
        self.preasure_sensor.start()

        self.microphone.start()

        while self.running.is_set():
            time.sleep(0.5)
        
        if not self.EIP is None:
            self.EIP.stopIO()

    def move_to(self, desired_state, T=None, only_x=False, only_z=False, only_alpha=False, only_flow=False):
        my_state = State(0, 0, 0, 0)
        my_state.x = encoder_units_to_mm(self.x_driver.encoder_position)
        my_state.z = encoder_units_to_mm(self.z_driver.motor_position)
        my_state.alpha = encoder_units_to_angle(self.alpha_driver.motor_position)
        my_state.flow = self.flow_driver.mass_flow_reading
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
            self.x_virtual_axis.merge_ref(route_x)
        if not only_x and not only_alpha and not only_flow:
            self.z_virtual_axis.merge_ref(route_z)
        if not only_x and not only_z and not only_flow:
            self.alpha_virtual_axis.merge_ref(route_alpha)
        self.virtual_flow.merge_ref(route_flow)
        self.virtual_flow.vibrato_amp = desired_state.vibrato_amp
        self.virtual_flow.vibrato_freq = desired_state.vibrato_freq

        return route['t'][-1]

    def execute_score(self, path, go_back=True):
        my_state = State(0, 0, 0, 0)
        my_state.x = x_units_to_mm(self.x_driver.motor_position)
        my_state.z = z_units_to_mm(self.z_driver.motor_position)
        my_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
        my_state.flow = self.flow_driver.mass_flow_reading

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
        self.x_virtual_axis.merge_ref(self.loaded_route_x)
        self.z_virtual_axis.merge_ref(self.loaded_route_z)
        self.alpha_virtual_axis.merge_ref(self.loaded_route_alpha)
        self.virtual_flow.merge_ref(self.loaded_route_flow)
        self.virtual_fingers.ref = self.loaded_route_notes
        self.virtual_fingers.changeEvent.set()
        self.memory.start_saving()

    def stop(self):
        self.x_virtual_axis.stop()
        self.z_virtual_axis.stop()
        self.alpha_virtual_axis.stop()
        self.virtual_flow.stop()
        self.virtual_fingers.stop()

    def get_ref_state(self):
        s = State(0,0,0,0)
        s.x = x_units_to_mm(self.x_virtual_axis.pos)
        s.z = z_units_to_mm(self.z_virtual_axis.pos)
        s.alpha = alpha_units_to_angle(self.alpha_virtual_axis.pos)
        s.flow = self.virtual_flow.flow
        return s

    def move_to_alpha(self, value):
        self.alpha_driver.homing_move_target = value
        # my_state = State(0, 0, 0, 0)
        # my_state.x = x_units_to_mm(self.x_driver.motor_position)
        # my_state.z = z_units_to_mm(self.z_driver.motor_position)
        # my_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
        # desired_state = State(0,0,0,0)
        # desired_state.x = x_units_to_mm(self.x_driver.motor_position)
        # my_state.z = z_units_to_mm(self.z_driver.motor_position)
        # my_state.alpha = value
        # my_state.flow = self.flow_driver.mass_flow_reading
        # route = get_route(my_state, desired_state, T=None, acc=200, dec=200)
        
        # move_t0 = time.time() - self.t0
        # route_alpha = []
        # for i in range(len(route['t'])):
        #     route_alpha.append((route['t'][i] + move_t0, route['alpha'][i], route['alpha_vel'][i]))

        # self.alpha_virtual_axis.merge_ref(route_alpha)

    def reset_x_controller(self):
        self.x_driver.break_loop()
        time.sleep(0.1)
        self.x_driver = AMCIDriver(self.EIP, self.connections[0], self.running, self.x_virtual_axis, connected=self.x_connect, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=100, virtual_axis_follow_deceleration=100, home=self.home)
        self.x_driver.start()

    def reset_z_controller(self):
        self.z_driver.break_loop()
        time.sleep(0.1)
        self.z_driver = AMCIDriver(self.EIP, self.connections[1], self.running, self.z_virtual_axis, connected=self.z_connect, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=100, virtual_axis_follow_deceleration=100, home=self.home)
        self.z_driver.start()

    def reset_alpha_controller(self):
        self.alpha_driver.break_loop()
        time.sleep(0.1)
        self.alpha_driver = AMCIDriver(self.EIP, self.connections[2], self.running, self.alpha_virtual_axis, connected=self.alpha_connect, starting_speed=1, verbose=False, motors_step_turn=10000, virtual_axis_follow_acceleration=1000, virtual_axis_follow_deceleration=1000, home=self.home, virtual_axis_proportional_coef=10)
        self.alpha_driver.start()

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
        my_state.flow = self.flow_driver.mass_flow_reading
        print(my_state)

class Memory(threading.Thread):
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, running, x_driver, z_driver, alpha_driver, flow_controller, preasure_sensor, x_reference, z_reference, alpha_reference, flow_reference, microphone, windowWidth=200, interval=0.05):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.flow_controller = flow_controller
        self.preasure_sensor = preasure_sensor
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.alpha_reference = alpha_reference
        self.flow_reference = flow_reference
        self.microphone = microphone
        self.windowWidth = windowWidth
        self.ref_state = State(0,0,0,0)
        self.real_state = State(0,0,0,0)

        self.ref_state.x = x_units_to_mm(self.x_reference.pos)
        self.ref_state.z = z_units_to_mm(self.z_reference.pos)
        self.ref_state.alpha = alpha_units_to_angle(self.alpha_reference.pos)
        self.ref_state.flow = self.flow_reference.flow
        self.real_state.x = x_units_to_mm(self.x_driver.motor_position)
        self.real_state.z = z_units_to_mm(self.z_driver.motor_position)
        self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
        self.real_state.flow = self.flow_controller.mass_flow_reading

        self.flow_ref = linspace(0,0,self.windowWidth)
        self.x_ref = linspace(0,0,self.windowWidth)
        self.z_ref = linspace(0,0,self.windowWidth)
        self.alpha_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.z = linspace(0,0,self.windowWidth)
        self.alpha = linspace(0,0,self.windowWidth)
        self.radius = linspace(0,0,self.windowWidth)
        self.theta = linspace(0,0,self.windowWidth)
        self.offset = linspace(0,0,self.windowWidth)
        self.radius_ref = linspace(0,0,self.windowWidth)
        self.theta_ref = linspace(0,0,self.windowWidth)
        self.offset_ref = linspace(0,0,self.windowWidth)
        self.mouth_pressure = linspace(0,0,self.windowWidth)
        self.volume_flow = linspace(0,0,self.windowWidth)
        self.mass_flow = linspace(0,0,self.windowWidth)
        self.temperature = linspace(0,0,self.windowWidth)
        self.frequency = linspace(0,0,self.windowWidth)
        self.times = linspace(0,0,self.windowWidth)
        self.t0 = time.time()
        self.first_entry = False
        self.t1 = 0

        self.data = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])

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
            
            self.ref_state.x = x_units_to_mm(self.x_reference.pos)
            self.ref_state.z = z_units_to_mm(self.z_reference.pos)
            self.ref_state.alpha = alpha_units_to_angle(self.alpha_reference.pos)
            self.ref_state.flow = self.flow_reference.flow
            self.real_state.x = encoder_units_to_mm(self.x_driver.encoder_position)
            self.real_state.z = z_units_to_mm(self.z_driver.encoder_position)
            self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position)
            self.real_state.flow = self.flow_controller.mass_flow_reading

            self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.x_ref[-1] = self.ref_state.x
            self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.z_ref[-1] = self.ref_state.z
            self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.alpha_ref[-1] = self.ref_state.alpha
            self.flow_ref[:-1] = self.flow_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.flow_ref[-1] = self.flow_controller.mass_flow_set_point_reading # ref_state.flow
            self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
            self.x[-1] = self.real_state.x
            self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
            self.z[-1] = self.real_state.z
            self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
            self.alpha[-1] = self.real_state.alpha
            self.volume_flow[:-1] = self.volume_flow[1:]                      # shift data in the temporal mean 1 sample left
            self.volume_flow[-1] = self.flow_controller.vol_flow_reading

            self.radius[:-1] = self.radius[1:]                      # shift data in the temporal mean 1 sample left
            self.radius[-1] = self.real_state.r
            self.theta[:-1] = self.theta[1:]                      # shift data in the temporal mean 1 sample left
            self.theta[-1] = self.real_state.theta
            self.offset[:-1] = self.offset[1:]                      # shift data in the temporal mean 1 sample left
            self.offset[-1] = self.real_state.o
            self.radius_ref[:-1] = self.radius_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.radius_ref[-1] = self.ref_state.r
            self.theta_ref[:-1] = self.theta_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.theta_ref[-1] = self.ref_state.theta
            self.offset_ref[:-1] = self.offset_ref[1:]                      # shift data in the temporal mean 1 sample left
            self.offset_ref[-1] = self.ref_state.o
            self.mouth_pressure[:-1] = self.mouth_pressure[1:]                      # shift data in the temporal mean 1 sample left
            self.mouth_pressure[-1] = self.preasure_sensor.preasure
            #self.volume_flow[:-1] = self.volume_flow[1:]                      # shift data in the temporal mean 1 sample left
            #self.volume_flow[-1] = self.flowController.values['vol_flow']
            self.mass_flow[:-1] = self.mass_flow[1:]                      # shift data in the temporal mean 1 sample left
            self.mass_flow[-1] = self.flow_controller.mass_flow_reading
            self.temperature[:-1] = self.temperature[1:]                      # shift data in the temporal mean 1 sample left
            self.temperature[-1] = self.flow_controller.temperature_reading
            self.frequency[:-1] = self.frequency[1:]                      # shift data in the temporal mean 1 sample left
            self.frequency[-1] = self.microphone.pitch
            self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
            self.times[-1] = time.time() - self.t0
            if self.saving:
                if self.first_entry:
                    self.t1 = self.times[-1]
                    self.first_entry = False
                # t = self.times[-1] - self.t1
                # new_data = pd.DataFrame({'time': [t for i in range(18)], 'signal':['frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'], 'value': [self.x_ref[-1], self.x[-1], self.z_ref[-1], self.z[-1], self.alpha_ref[-1], self.alpha[-1], self.vel_state_x, self.vel_state_z, self.vel_state_alpha]})
                # self.data = self.data.append(new_data, ignore_index = True)

                new_data = pd.DataFrame([[self.times[-1] - self.t1, self.frequency[-1], self.temperature[-1], self.mass_flow[-1], self.volume_flow[-1], self.mouth_pressure[-1], self.offset[-1], self.theta[-1], self.radius[-1], self.offset_ref[-1], self.theta_ref[-1], self.radius_ref[-1], self.alpha[-1], self.z[-1], self.x[-1], self.alpha_ref[-1], self.z_ref[-1], self.x_ref[-1], self.alpha_ref[-1]]], columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
                self.data = pd.concat([self.data, new_data], ignore_index=True)
            time.sleep(self.interval)
    
    def start_saving(self):
        self.first_entry = True
        self.data = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
        # self.data = pd.DataFrame(columns=['time', 'signal', 'value'])
        self.saving = True
        self.microphone.start_saving()
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        self.data.to_csv(file_name)

    def save_recorded_data(self, filename1, filename2):
        self.finish_saving(filename1)
        self.microphone.finish_saving(filename2)

    def stop_recording(self):
        self.saving = False
        self.microphone.saving = False

if __name__ == "__main__":
    print(sd.query_devices())
    event = threading.Event()
    event.set()
    mic = Microphone(event, connected=True, verbose=True)
    mic.start()

    while True:
        t = input()
        if t == "q":
            event.clear()
            break
        elif t == "r":
            mic.start_saving()
        elif t == "s":
            mic.finish_saving("/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/escala2.wav")

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
    
# Normal output to AMCI:
# 256 36864 125 0 0 0 100 100 1 20
# Normal input to AMCI:
# [16386, 40966, 0, 125, 0, 0, 0, 0, 30, 20]

# Obtenido en el error:
# [2, 32770, 8, 3826872493, 65536, 177, 10, 65536]

#1 ['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 3826863591, 238, 177, 10, 238] 
# b'\x01\x00\x00\x00\x00\x00\x00\x00'

#['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 3729719315, 65536, 177, 26, 65536] 
# b'\x01\x00\x00\x00\x00\x01\x00\x902\x02\x00\x00\x00\x00\x00\x00d\x00d\x00\x01\x00\x14\x00'

#['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 1004208147, 65536, 177, 26, 65536]
# b'\x01\x00\x00\x00\x00\x01\x00\x90\x00\x00\x00\x00\x00\x00\x00\x00\xe8\x03\xe8\x03\n\x00\x14\x00'

#['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 3177906195, 65536, 177, 26, 65536] 
# b'\x01\x00\x00\x00\x00\x01\x00\x90\xfa\x00\x00\x00\x00\x00\x00\x00d\x00d\x00\x01\x00\x14\x00'

#['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 3826899964, 65536, 177, 10, 65536] 
# b'\x01\x00\x00\x00\x8f\xde\xafA'

#['count', 'type_id_seq_addr', 'len_seq_addr', 'conn_id', 'seq_num', 'type_id_conn_data', 'len_conn_data', 'seq_count'] 
# [2, 32770, 8, 3826870539, 65536, 177, 10, 65536] 
# b'\x01\x00\x00\x00\x00\x00\x00\x00'