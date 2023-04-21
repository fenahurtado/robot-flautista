import sys
from PyQt5.QtWidgets import QApplication
#import threading
from multiprocessing import Process, Event, Value, Pipe, Manager
from time import time
from exercises.main_window import Window
from exercises.drivers_connect import Musician
from numpy import linspace

if __name__ == '__main__':
    app = QApplication(sys.argv)

    host = "192.168.2.10"
    connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]
    event = Event()
    event.set()

    mgr = Manager()
    data = mgr.dict()
    
    t0 = time()
    pipe2pierre, pierre_pipe = Pipe()
    pierre = Musician(host, connections, event, pierre_pipe, data, fingers_connect=False, x_connect=True, z_connect=True, alpha_connect=True, flow_connect=False, pressure_sensor_connect=False, mic_connect=False)
    pierre.start()

    print(pipe2pierre.recv())

    win = Window(app, event, pipe2pierre, data)
    win.show()

    print("Executing GUI...")

    sys.exit(app.exec())


# import sys
# import threading
# from PyQt5.QtWidgets import QApplication

# from utils.driver_amci import AMCIDriver, INPUT_FUNCTION_BITS
# from utils.microphone import Microphone
# from utils.driver_fingers import FingersDriver
# from utils.player import Player, State
# from utils.sensores_alicat import FlowController, PreasureSensor
# from view_control.main_window import Window

# connected = False

# preasure_sensor_event = threading.Event()
# preasure_sensor_event.set()
# preasure_sensor = PreasureSensor('192.168.2.100', preasure_sensor_event, connected=connected)
# preasure_sensor.start()

# flow_controler_event = threading.Event()
# flow_controler_event.set()
# flow_controller = FlowController('192.168.2.101', flow_controler_event, connected=connected)
# flow_controller.start()

# x_event = threading.Event()
# x_event.set()
# x_driver = AMCIDriver('192.168.2.102', x_event, connected=True, starting_speed=1, verbose=True, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
# x_driver.start()

# z_event = threading.Event()
# z_event.set()
# z_driver = AMCIDriver('192.168.2.104', z_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
# z_driver.start()

# #v405

# alpha_event = threading.Event()
# alpha_event.set()
# alpha_driver = AMCIDriver('192.168.2.103', alpha_event, connected=connected, verbose=True, starting_speed=1, motors_step_turn=10000)#, input_1_function_bits=INPUT_FUNCTION_BITS['Home'])
# alpha_driver.start()

# microphone_event = threading.Event()
# microphone_event.set()
# microphone = Microphone(microphone_event)
# #microphone.start()

# # Funcionalidad de servos para presionar las llaves
# fingers_event = threading.Event()
# fingers_event.set()
# ## TEFO: '/dev/cu.usbserial-142420'
# fingers_driver = FingersDriver('/dev/ttyUSB0', fingers_event, connected=False)
# # try:
# #     fingers_driver = FingersDriver('/dev/ttyUSB0', fingers_event, connected=connected)
# # except:
# #     fingers_driver = FingersDriver('/dev/ttyUSB1', fingers_event, connected=connected)
# fingers_driver.start()

# state = State(0, 0, 0, 0)
# ##state.homed()  ## Cambiar despues, agregar rutina de homing!!!

# musician_event = threading.Event()
# musician_event.set()
# musician = Player(musician_event, state, flow_controller, preasure_sensor, x_driver, z_driver, alpha_driver,
#                   microphone, fingers_driver)
# musician.start()

# app = QApplication(sys.argv)

# win = Window(app, preasure_sensor_event, flow_controler_event, x_event, z_event, alpha_event, microphone_event,
#              fingers_event, musician_event, musician, state)
# win.show()

# sys.exit(app.exec())