import sys
import struct
import time
from multiprocessing import Process, Event, Pipe, Value, Manager
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista')
import lib.ethernet_ip.ethernetip as ethernetip
from exercises.drivers_connect import Command, Setting, INPUT_FUNCTION_BITS, VirtualFlow, VirtualAxis

class CommunicationCenter(Process):
    def __init__(self, event, pipe, data):
        Process.__init__(self)
        self.event = event
        self.pipe = pipe
        self.EIP = ethernetip.EtherNetIP(host)
        self.connections = {}
        self.data = data
    
    def run(self):
        self.EIP.startIO()
        while self.event.is_set():
            for host, conn in self.connections.items():
                try:
                    self.data[host + '_in'] = conn.inAssem
                    conn.outAssem = self.data[host + '_out']
                except:
                    print("Hubo un error en la lectura del input en el centro de comunicaciones")
            if self.pipe.poll(0.1):
                message = self.pipe.recv()
                print("Message received", message)
                if message[0] == "explicit_conn":
                    C1 = self.EIP.explicit_conn(message[1])
                    C1.outAssem = [0 for i in range(message[2])]
                    C1.inAssem = [0 for i in range(message[3])]

                    pkt = C1.listID()
                    if pkt is not None:
                        print("Product name: ", pkt.product_name.decode())

                    inputsize = message[4]
                    outputsize = message[5]

                    self.data[message[1] + '_in'] = [0 for i in range(inputsize*8)]
                    self.data[message[1] + '_out'] = [0 for i in range(outputsize*8)]

                    self.EIP.registerAssembly(message[6], inputsize, message[7], C1)
                    self.EIP.registerAssembly(message[8], outputsize, message[9], C1)

                    self.connections[message[1]] = C1
                    print("Assembly registered...")

                elif message[0] == "registerSession":
                    self.connections[message[1]].registerSession()

                elif message[0] == "sendFwdOpenReq":
                    self.connections[message[1]].sendFwdOpenReq(message[2], message[3], message[4], torpi=message[5], otrpi=message[6], priority=message[7])
                    self.connections[message[1]].produce()

                elif message[0] == "stopProduce":
                    self.connections[message[1]].stopProduce()
                    self.connections[message[1]].sendFwdCloseReq(message[2], message[3], message[4])
                
                elif message[0] == "setAttrSingle":
                    self.connections[message[1]].setAttrSingle(message[2], message[3], message[4], message[5])
                
                elif message[0] == "getAttrSingle":
                    att = self.connections[message[1]].getAttrSingle(message[2], message[3], message[4])
                    self.pipe.send([message[1], att])

        self.EIP.stopIO()
        
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

class FlowControllerDriver(Process):
    def __init__(self, hostname, running, t0, pipe_end, comm_pipe, comm_data, connected=True, verbose=False): # 
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.t0 = t0
        self.pipe_end = pipe_end
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
        self.virtual_flow = VirtualFlow(self.running, 0.01, self.t0, self.pipe_end, verbose=False)
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

class AMCIDriver(Process):
    def __init__(self, hostname, running, virtual_axis, musician_pipe, comm_pipe, comm_data, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=1, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=40, current_loop_gain=5, homing_slow_speed=200, verbose=False, virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True, virtual_axis_proportional_coef=1, Kp=0, Ki=5, Kd=0.01):
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = virtual_axis
        self.musician_pipe = musician_pipe
        self.comm_pipe = comm_pipe
        self.comm_data = comm_data
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
        if self.connected:
            self.C1.registerSession()

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
                corrected_pos, corrected_vel = self.pid_control(self.virtual_axis.pos.value, self.virtual_axis.vel.value)
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
        ccw_jog = self.get_ccw_jog_command(programmed_speed=400, motor_current=self.motor_current)
        self.comm_data[self.hostname + '_out'] = ccw_jog.get_list_to_send()
    
    def cw_find_home_to_limit(self):
        self.fast_cw_limit_homing = True
        cw_jog = self.get_cw_jog_command(programmed_speed=4000, acceleration=5, motor_current=self.motor_current)
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

if __name__ == "__main__":
    host = "192.168.2.10"
    devices = ["192.168.2.100", "192.168.2.101"]

    event1 = Event()
    event1.set()
    event2 = Event()
    event2.set()

    main_pipe1, main_pipe2 = Pipe()
    comm_pipe1, comm_pipe2 = Pipe()

    mgr = Manager()
    comm_data = mgr.dict()

    cc = CommunicationCenter(event1, comm_pipe1, comm_data)
    cc.start()

    ps = PressureSensor(devices[0], event2, comm_pipe2, comm_data, connected=True, verbose=True)
    ps.start()

    t0 = time.time()
    mfc = FlowControllerDriver(devices[1], event2, t0, main_pipe2, comm_pipe2, comm_data)
    mfc.start()
    
    while True:
        if input() == "q":
            event2.clear()
            time.sleep(0.1)
            event1.clear()
            break
        else:
            print(2)
    
    print("Closing...")