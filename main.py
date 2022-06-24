import sys
import threading
from PyQt5.QtWidgets import QApplication

from utils.driver_amci import AMCIDriver, INPUT_FUNCTION_BITS
from utils.microphone import Microphone
from utils.player import Player, State
from utils.sensores_alicat import FlowController, PreasureSensor
from view_control.main_window import Window

connected = True

preasure_sensor_event = threading.Event()
preasure_sensor_event.set()
preasure_sensor = PreasureSensor('192.168.2.100', preasure_sensor_event, connected=connected)
preasure_sensor.start()

flow_controler_event = threading.Event()
flow_controler_event.set()
flow_controller = FlowController('192.168.2.101', flow_controler_event, connected=connected)
flow_controller.start()

x_event = threading.Event()
x_event.set()
x_driver = AMCIDriver('192.168.2.104', x_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
x_driver.start()

z_event = threading.Event()
z_event.set()
z_driver = AMCIDriver('192.168.2.103', z_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
z_driver.start()

alpha_event = threading.Event()
alpha_event.set()
alpha_driver = AMCIDriver('192.168.2.102', alpha_event, connected=connected, starting_speed=1, motors_step_turn=10000)#, input_1_function_bits=INPUT_FUNCTION_BITS['Home'])
alpha_driver.start()

microphone_event = threading.Event()
microphone_event.set()
microphone = Microphone(microphone_event)
#microphone.start()

state = State(0,0,0,0)
##state.homed()  ## Cambiar despues, agregar rutina de homing!!!

musician_event = threading.Event()
musician_event.set()
musician = Player(musician_event, state, flow_controller, preasure_sensor, x_driver, z_driver, alpha_driver, microphone)
musician.start()

app = QApplication(sys.argv)

win = Window(app, preasure_sensor_event, flow_controler_event, x_event, z_event, alpha_event, microphone_event, musician_event, musician, state)

win.show()

sys.exit(app.exec())