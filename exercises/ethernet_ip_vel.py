from time import time, sleep
import threading
import numpy as np
from cpppo.server.enip import poll
from cpppo.server.enip.get_attribute import proxy_simple as device
import matplotlib.pyplot as plt

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

def write_configuration(setting, via):
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

        #print(f'@4/150/3=(UINT){word0},{word1},{word2},{word3},{word4},{word5},{word6},{word7},{word8},{word9}')
        with via:
            data, = via.read( [(f'@4/150/3=(INT){word0},{word1},{word2},{word3},{word4},{word5},{word6},{word7},{word8},{word9}', ("INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))])

def write_command(command, via):
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

    with via:
        data, = via.read( [(f'@4/150/3=(INT){word0},{word1},{word2},{word3},{word4},{word5},{word6},{word7},{word8},{word9}', ("INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))])
    

times_1 = []
t_ant_1 = time()

def process_incoming_data_1(par, val):
    global times_1, t_ant_1
    t_act_1 = time()
    times_1.append(t_act_1-t_ant_1)
    t_ant_1 = t_act_1

times_2 = []
t_ant_2 = time()

def process_incoming_data_2(par, val):
    global times_2, t_ant_2
    t_act_2 = time()
    times_2.append(t_act_2-t_ant_2)
    t_ant_2 = t_act_2

times_3 = []
t_ant_3 = time()

def process_incoming_data_3(par, val):
    global times_3, t_ant_3
    t_act_3 = time()
    times_3.append(t_act_3-t_ant_3)
    t_ant_3 = t_act_3

times_4 = []
t_ant_4 = time()

def process_incoming_data_4(par, val):
    global times_4, t_ant_4
    t_act_4 = time()
    times_4.append(t_act_4-t_ant_4)
    t_ant_4 = t_act_4

times_5 = []
t_ant_5 = time()

def process_incoming_data_5(par, val):
    global times_5, t_ant_5
    t_act_5 = time()
    times_5.append(t_act_5-t_ant_5)
    t_ant_5 = t_act_5

def get_poller_via(hostname, params, func, cycle_time=0.05):
    poller                  = threading.Thread(
        target=poll.poll, args=(device,), kwargs={
            'address':      (hostname, 44818),
            'cycle':        cycle_time,
            'timeout':      0.5,
            'process':      func,
            'params':       params,
        })
    poller.daemon           = True
    via = device(hostname)
    return poller, via

host_1                  = '192.168.2.101'
params_1                = [('@4/101/3',("INT", "DINT", "REAL", "REAL", "REAL", "REAL", "REAL"))]
p1, v1 = get_poller_via(host_1, params_1, process_incoming_data_1, cycle_time=0.05)
p1.start()

host_2                  = '192.168.2.100'
params_2                = [('@4/101/3',("INT", "DINT", "REAL"))]
p2, v2 = get_poller_via(host_2, params_2, process_incoming_data_2, cycle_time=0.05)
p2.start()

host_3                  = '192.168.2.102'
params_3                = [('@4/100/3',("WORD", "WORD", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))]
p3, v3 = get_poller_via(host_3, params_3, process_incoming_data_3, cycle_time=0.05)
p3.start()
initial_settings = Setting(motors_step_turn=1000)
write_configuration(initial_settings, v3)
sleep(0.1)
position = 250
command = Command(preset_motor_position=1, name='Preset Position')
command.desired_command_word_2 = abs(position) // 1000 * np.sign(position)
command.desired_command_word_3 = abs(position)  % 1000 * np.sign(position)
write_command(command, v3)

host_4                  = '192.168.2.103'
params_4                = [('@4/100/3',("WORD", "WORD", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))]
p4, v4 = get_poller_via(host_4, params_4, process_incoming_data_4, cycle_time=0.05)
p4.start()
initial_settings = Setting(motors_step_turn=10000)
write_configuration(initial_settings, v4)
sleep(0.1)
position = 250
command = Command(preset_motor_position=1, name='Preset Position')
command.desired_command_word_2 = abs(position) // 1000 * np.sign(position)
command.desired_command_word_3 = abs(position)  % 1000 * np.sign(position)
write_command(command, v4)

host_5                  = '192.168.2.104'
params_5                = [('@4/100/3',("WORD", "WORD", "INT", "INT", "INT", "INT", "INT", "INT", "INT", "INT"))]
p5, v5 = get_poller_via(host_5, params_5, process_incoming_data_5, cycle_time=0.05)
p5.start()
initial_settings = Setting(motors_step_turn=1000)
write_configuration(initial_settings, v5)
sleep(0.1)
position = 250
command = Command(preset_motor_position=1, name='Preset Position')
command.desired_command_word_2 = abs(position) // 1000 * np.sign(position)
command.desired_command_word_3 = abs(position)  % 1000 * np.sign(position)
write_command(command, v5)

def task_1():
    global v1
    t0 = time()
    n=0
    while time() - t0 < 5:
        t = time()
        ref = 25+25*np.sin(2*np.pi*t*6)
        with v1:
            data, = v1.read( [('@4/100/3={}'.format(ref),"REAL")] )
            n+=1
        #sleep(0.03)

    with v1:
        data, = v1.read( [('@4/100/3=0.0',"REAL")] )

    print('Instructions wrote on Flow controller:' + str(n)) 

def task_2():
    global v2
    pass

def task_3():
    global v3
    t0 = time()
    n=0
    freq = 2
    amp = 250
    off = 250
    t = 0
    while t < 5:
        position = round(off+amp*np.sin(2*np.pi*t*freq))
        direction = 0
        speed = round(amp*2*np.pi*freq*np.cos(2*np.pi*t*freq))
        acceleration = 100
        deceleration = 100
        proportional_coefficient = 1
        network_delay = 0
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

        write_command(command, v3)
        
        n += 1
        t = time() - t0
        #sleep(0.03)

    print('Instructions wrote on Motor 1:' + str(n)) 

def task_4():
    global v4
    t0 = time()
    n=0
    freq = 2
    amp = 100
    off = 250
    t = 0
    while t < 5:
        position = round(off+amp*np.sin(2*np.pi*t*freq))
        direction = 0
        speed = round(amp*2*np.pi*freq*np.cos(2*np.pi*t*freq))
        acceleration = 100
        deceleration = 100
        proportional_coefficient = 1
        network_delay = 0
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

        write_command(command, v4)
        
        n += 1
        t = time() - t0
        #sleep(0.03)

def task_5():
    global v5
    t0 = time()
    n=0
    freq = 2
    amp = 250
    off = 250
    t = 0
    while t < 5:
        position = round(off+amp*np.sin(2*np.pi*t*freq))
        direction = 0
        speed = round(amp*2*np.pi*freq*np.cos(2*np.pi*t*freq))
        acceleration = 100
        deceleration = 100
        proportional_coefficient = 1
        network_delay = 0
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

        write_command(command, v5)
        
        n += 1
        t = time() - t0
        #sleep(0.03)

    print('Instructions wrote on Motor 3:' + str(n)) 

t1 = threading.Thread(target=task_1)
t2 = threading.Thread(target=task_2)
t3 = threading.Thread(target=task_3)
t4 = threading.Thread(target=task_4)
t5 = threading.Thread(target=task_5)

t1.start()
t2.start()
t3.start()
t4.start()
t5.start()

t1.join()
t2.join()
t3.join()
t4.join()
t5.join()

plt.hist(times_4[1:], bins=100)
plt.show()

# plt.hist(times_2[1:], bins=100)
# plt.show()