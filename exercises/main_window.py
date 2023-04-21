#from importlib.util import set_loader
#from ssl import AlertDescription
import sys
import sys
import threading
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout, QGridLayout, QWidget, QLabel, QSpinBox, QMenu, QShortcut
)
from PyQt5.QtCore import QEventLoop, Qt
from PyQt5.QtGui import QPainterPath, QRegion, QKeySequence

#from PyQt5.uic import loadUi
#from matplotlib.pyplot import close
#from regex import D
#from utils.driver_amci import AMCIDriver
#from regex import D
from views.main_window import Ui_MainWindow
from view_control.forms import FingersActionForm, MoveActionForm, CalibrateFluteForm, CalibrateAngleForm, \
    ConfigureFluteControlForm, StartActionForm, InstrumentForm, ScaleTimeForm, ParamCorrectionForm, StatesFromNotesForm, ZoomScoreForm

from exercises.manual_move import ManualMoveCollapsibleBox
from view_control.action_display import ActionWidget
from view_control.plot_window import Window as PlotWindow
from utils.cinematica import *
from utils.motor_route import max_dist_rec, get_route_positions
#from view_control.amci_control import AMCIWidget

#from utils.player import *
from exercises.drivers_connect import Musician, Memory

from tools.score_tools import *

#from dialog_control import Ui_Dialog
from pyqtgraph.Qt import QtGui, QtCore
#import pyqtgraph as pg
#import random

import json
#import typing
import os
#import serial
#import threading
#import math
#import queue
from time import time, sleep
#import io
from functools import partial
#from pybase64 import b64decode
from datetime import date, datetime
from numpy import *
#from utils.motor_control import Window as PlayingWindow


class Window(QMainWindow, Ui_MainWindow):
    '''
    Esta clase conecta los elementos de la GUI principal con todas las funciones que se quieren realizar con el robot.
    '''
    def __init__(self, app, running, musician_pipe, data, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        #self.actionTypeComboBox.model().item(1).setEnabled(False)
        #self.actionTypeComboBox.model().item(2).setEnabled(False)
        self.fingersActionButton.hide()
        self.addActionButton.hide()
        self.actionsCount = 0
        self.fingerActionsCount = 0
        #self.actionChoise = 0
        #self.timeSelect = 1
        #self.positioning_time = 2
        self.initialPositionLayout.setAlignment(Qt.AlignLeft)
        #self.initialPositionLayout.setFixedWidth(int(100*self.positioning_time))
        self.scoreLayout.setAlignment(Qt.AlignLeft)
        self.fingersScoreLayout.setAlignment(Qt.AlignLeft)
        self.filename = None
        self.base_path = os.path.dirname(os.path.realpath(__file__))
        self.last_path = self.base_path
        print(self.base_path)
        #self.performing = False
        #self.playing = False
        #self.vars = read_variables()
        #self.state = state
        #self.state.homed()
        # self.desired_state = State(0,0,0,0)
        # self.desired_state.homed()

        self.running = running
        self.musician_pipe = musician_pipe

        # self.musician_pipe.send(["get_memory_data"])
        self.data = data
        # self.preasure_sensor_event = preasure_sensor_event
        # #self.preasure_sensor = self.musician.preasure_sensor
        # self.flow_controler_event = flow_controler_event
        # #self.flow_controller = self.musician.flow_controller
        # self.x_event = x_event
        # #self.x_driver = self.musician.x_drive
        # self.z_event = z_event
        # #self.z_driver = self.musician.z_drive
        # self.alpha_event = alpha_event
        # #self.alpha_driver = self.musician.alpha_drive
        # self.microphone_event = microphone_event
        # #self.microphone = self.musician.microphone
        # self.fingers_event = fingers_event

        self.initial_position = None
        self.phrase_actions = []
        self.finger_actions = []
        #self.performing = threading.Event()
        #self.playing = threading.Event()
        #self.player = Player(self.phrase_actions, self.state, self.desired_state, self.performing, self.playing, self.flow_controller, self.preasure_sensor, self.x_driver, self.z_driver, self.alpha_driver, self.microphone, parent=self)
        #self.player.motors_control.write_at_home()
        self.scrolled = 0

        self.moveBox = ManualMoveCollapsibleBox("Manual Move", musician_pipe, parent=self)
        self.moveBox.stopButton.clicked.connect(self.stop_motors)
        self.gridLayout.addWidget(self.moveBox, 3, 0, 1, 8)

        self.setWindowTitle("Pierre - Flutist Robot")
        self.connectSignalsSlots()
        self.file_saved = True

        self.find_recent_files()
        
        #self.autohome_routine()
        #self.autohome_Alpha_routine()

        # Selección de instrumento
        self.instrument_dialog = None
        self.instrument = 'flute'
        self.zoom_factor = 1
        self.select_instrument()
        self.moveBox.add_notes(self.instrument)
        
        #self.musician.recorder.start()

    def closeEvent(self, a0: QtGui.QCloseEvent):
        '''
        Esta función se ejecuta al cerrar el programa, para terminar todos los threads que están corriendo
        '''
        #self.musician.motors_controller.reset_drivers()
        self.running.clear()
        sleep(0.2)
        #self.player.flowSignalEvent.clear()
        #self.player.moveSignalEvent.clear()
        # self.preasure_sensor_event.clear()
        # self.flow_controler_event.clear()
        # self.x_event.clear()
        # self.z_event.clear()
        # self.alpha_event.clear()
        # self.microphone_event.clear()
        # self.fingers_event.clear()
        return super().closeEvent(a0)
        
    def connectSignalsSlots(self):
        '''
        Conecta los elementos de la GUI, asi como señales emitidas desde otros threads, con funciones a ejecutar
        '''
        self.addActionButton.clicked.connect(self.add_action)
        self.fingersActionButton.clicked.connect(self.add_fingers_action)
        self.setInitialPositionButton.clicked.connect(self.add_initial_position_action)
        #self.actionTypeComboBox.currentIndexChanged.connect(self.change_action_type)
        self.actionChangeFlutePosition.triggered.connect(self.change_flute_position)
        #self.actionAutoHomeRoutine.triggered.connect(self.autohome_routine)
        self.actionAutoHome_X.triggered.connect(self.reset_x_controller)
        self.actionAutoHome_Z.triggered.connect(self.reset_z_controller)
        self.actionAutoHome_Alpha.triggered.connect(self.reset_alpha_controller)

        self.actionMeasureRadius.triggered.connect(self.measure_radius)
        self.actionMeasureTheta.triggered.connect(self.measure_theta)
        self.actionMeasureOffset.triggered.connect(self.measure_offset)
        self.actionMeasurePosition.triggered.connect(self.measure_position)
        self.actionMeasureMouthPressure.triggered.connect(self.measure_mouth_presure)
        self.actionMeasureMassFlowRate.triggered.connect(self.measure_mass_flow_rate)
        self.actionMeasureVolumeFlowRate.triggered.connect(self.measure_volume_flow_rate)
        self.actionMeasureAirTemperature.triggered.connect(self.measure_temperature)
        self.actionMeasureSoundFrequency.triggered.connect(self.measure_sound_frequency)
        self.actionMeasureXPosition.triggered.connect(self.measure_x_position)
        self.actionMeasureZPosition.triggered.connect(self.measure_z_position)
        self.actionMeasureAlphaPosition.triggered.connect(self.measure_alpha_position)

        self.actionGenerate_states_from_notesOn_same_file.triggered.connect(self.score_generate_states)
        self.actionGenerate_states_from_notesOn_different_file.triggered.connect(self.score_generate_states_diff)

        self.actionScale_timeOn_same_file.triggered.connect(self.score_scale_time)
        self.actionScale_timeOn_different_file.triggered.connect(self.score_scale_time_diff)

        self.actionAdd_correctionOn_same_file.triggered.connect(self.score_add_correction)
        self.actionAdd_correctionOn_different_file.triggered.connect(self.score_add_correction_diff)

        self.actionUpdate_default_positions.triggered.connect(self.update_default_positions)

        self.actionZoomScore.triggered.connect(self.zoom_score)
        #self.actionReconnectFlowController.triggered.connect(self.reconnect_flow_controller)
        #self.actionReconnectPreasureSensor.triggered.connect(self.reconnect_preasure_sensor)
        #self.actionReconnectXController.triggered.connect(self.reconnect_x_controller)
        #self.actionReconnectZController.triggered.connect(self.reconnect_z_controller)
        #self.actionReconnectAngleController.triggered.connect(self.reconnect_angle_controller)

        self.actionConfigureFlowControlLoop.triggered.connect(self.configure_flow_control_loop)

        self.actionSave.triggered.connect(self.save)
        self.actionSaveAs.triggered.connect(self.save_as)
        self.actionOpen_2.triggered.connect(self.open)
        self.fastSave = QShortcut(QKeySequence('Ctrl+S'), self)
        self.fastSave.activated.connect(self.save)
        self.actionNew.triggered.connect(self.new_file)

        self.startButton.hide()
        self.startButton.clicked.connect(self.start_loaded_script)
        # self.pauseButton.clicked.connect(self.pause)
        # self.pauseButton.hide()
        # self.resumeButton.hide()
        # self.resumeButton.clicked.connect(self.resume)
        self.stopButton.clicked.connect(self.stop)
        self.stopButton.hide()
        self.executeButton.clicked.connect(self.execute_score)

        # self.actionXAxisTool.triggered.connect(self.open_x_driver_tool)
        # self.actionZAxisTool.triggered.connect(self.open_z_driver_tool)
        # self.actionAlphaAxisTool.triggered.connect(self.open_alpha_driver_tool)

        # self.musician.finished_score.connect(self.finished_score)
        # self.musician.finished_initial_positioning.connect(self.change_playing_initial_position)
        # self.musician.begin_phrase_action.connect(self.change_playing_phrase_act)
        # self.musician.begin_finger_action.connect(self.change_playing_fingers_act)

    def reset_x_controller(self):
        self.musician_pipe.send(["reset_x_controller"])

    def reset_z_controller(self):
        self.musician_pipe.send(["reset_z_controller"])

    def reset_alpha_controller(self):
        self.musician_pipe.send(["reset_alpha_controller"])
        #self.autohome_Alpha_routine()

    def start_loaded_script(self):
        self.musician_pipe.send(["start_loaded_script"])

    # def open_x_driver_tool(self):
    #     '''
    #     Abre una ventana con herramientas para controlar el driver del eje X
    #     '''
    #     toolwin = AMCIWidget(self.musician.x_driver, parent=self)
    #     toolwin.setWindowTitle('X Driver Tool')
    #     toolwin.show()

    # def open_z_driver_tool(self):
    #     '''
    #     Abre una ventana con herramientas para controlar el driver del eje Z
    #     '''
    #     toolwin = AMCIWidget(self.musician.z_driver, parent=self)
    #     toolwin.setWindowTitle('Z Driver Tool')
    #     toolwin.show()

    # def open_alpha_driver_tool(self):
    #     '''
    #     Abre una ventana con herramientas para controlar el driver del eje Alpha
    #     '''
    #     toolwin = AMCIWidget(self.musician.alpha_driver, parent=self)
    #     toolwin.setWindowTitle('Alpha Driver Tool')
    #     toolwin.show()

    # def pause(self):
    #     '''
    #     Mientras se está ejecutando una partitura es posible detenerla (hacer una pausa) con la posibilidad de después seguir ejecutándola desde donde se dejó.
    #     '''
    #     self.pauseButton.hide()
    #     self.resumeButton.show()
    #     self.musician.pause()
    #     self.moveBox.enableButtons()
        
    #     self.moveBox.set_values(self.state)

    def execute_score(self):
        '''
        Esta función comienza la ejecución de la partitura que haya sido ingresada desde la interfaz.
        '''
        r = self.save()
        if r:
            self.musician_pipe.send(["execute_score", self.filename])
            # measure = ['X','X_ref']
            # win = PlayingWindow(self.app, measure, self.musician, parent=self, independ=False)
            # win.show()
            # self.executeButton.setEnabled(False)
            # win.open_score(self.filename)

            # self.pauseButton.show()
            self.stopButton.show()
            self.executeButton.hide()
            self.startButton.show()
            #self.startButton.setEnabled(False)
            self.setInitialPositionButton.setEnabled(False)
            self.addActionButton.setEnabled(False)
            self.fingersActionButton.setEnabled(False)
            #self.actionTypeComboBox.setEnabled(False)
            for index in range(self.scoreLayout.count()):
                self.scoreLayout.itemAt(index).widget().disable_context_menu()
            for index in range(self.fingersScoreLayout.count()):
                self.fingersScoreLayout.itemAt(index).widget().disable_context_menu()
            for index in range(self.initialPositionLayout.count()):
                self.initialPositionLayout.itemAt(index).widget().disable_context_menu()
            self.moveBox.disableButtons()
            self.scrollArea.horizontalScrollBar().setValue(0)

        # self.musician.initial_position = self.initial_position
        # self.musician.phrase_instructions = self.phrase_actions
        # self.musician.finger_instructions = self.finger_actions
        
        

        # # if self.initialPositionLayout.count():
        # #     self.initialPositionLayout.itemAt(0).widget().paint_green()
        
        # self.musician.start_saving_data()

    def stop(self):
        '''
        Esta función se usa para detener una partitura que se esté ejecutando
        '''
        #print('Stop clicked')
        #self.memory.stop_recording()
        self.musician_pipe.send(["stop"])
        # self.musician.playing.clear()
        self.startButton.hide()
        self.stopButton.hide()
        # self.resumeButton.hide()
        self.executeButton.show()
        self.setInitialPositionButton.setEnabled(True)
        self.addActionButton.setEnabled(True)
        self.fingersActionButton.setEnabled(True)
        #self.actionTypeComboBox.setEnabled(True)
        for index in range(self.initialPositionLayout.count()):
            self.initialPositionLayout.itemAt(index).widget().enable_context_menu()
            self.initialPositionLayout.itemAt(index).widget().paint_blue()
        for index in range(self.scoreLayout.count()):
            self.scoreLayout.itemAt(index).widget().enable_context_menu()
            self.scoreLayout.itemAt(index).widget().paint_blue()
        for index in range(self.fingersScoreLayout.count()):
            self.fingersScoreLayout.itemAt(index).widget().enable_context_menu()
            self.fingersScoreLayout.itemAt(index).widget().paint_blue()
        self.moveBox.enableButtons()
        # while QApplication.hasPendingEvents():
        #     QApplication.processEvents()
        # while self.musician.moving():
        #     pass
        self.musician_pipe.send(['get_ref_state'])
        state = self.musician_pipe.recv()[0]
        print("Stop", state)
        #state = self.musician.get_ref_state()
        self.moveBox.set_values(state)

        msg = QMessageBox()
        #msg.setIcon(QMessageBox.Critical)
        msg.setText("Save the data recorded during execution?")
        msg.setInformativeText("Data will be lost if you don't save them.")
        msg.setWindowTitle("Save data?")
        dont_save_button = msg.addButton("Don't save", QtWidgets.QMessageBox.NoRole)
        save_button = msg.addButton("Save", QtWidgets.QMessageBox.YesRole)

        retval = msg.exec_()
        if retval == 0: # don't save
            pass
        elif retval == 1: # save
            self.save_recorded_data()

    def save_recorded_data(self):
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.base_path,"CSV files (*.csv)")
        if fname[-4:] != '.csv':
            fname += '.csv'
        self.last_path = os.path.split(fname)[0]
        fname2, _ = QFileDialog.getSaveFileName(self, 'Open file', self.last_path,"WAV files (*.wav)")
        if fname2[-4:] != '.wav':
            fname2 += '.wav'
        self.last_path = os.path.split(fname2)[0]
        self.musician_pipe.send(["memory.save_recorded_data", fname, fname2])
        #self.memory.save_recorded_data(fname, fname2)

    def change_playing_initial_position(self):
        '''
        Esta función se llama cuando el robot termino de moverse hasta la posición inicial, vuelve a pintar este evento azul y comienza pintando los que vienen verdes
        '''
        if self.initialPositionLayout.count():
            self.initialPositionLayout.itemAt(0).widget().paint_blue()
        if self.fingersScoreLayout.count():
            self.fingersScoreLayout.itemAt(0).widget().paint_green()
        if self.scoreLayout.count():
            self.scoreLayout.itemAt(0).widget().paint_green()

    def finished_score(self):
        if self.scoreLayout.count():
            self.scoreLayout.itemAt(self.scoreLayout.count()-1).widget().paint_blue()
        if self.fingersScoreLayout.count():
            self.fingersScoreLayout.itemAt(self.fingersScoreLayout.count()-1).widget().paint_blue()
        self.stop()

    def change_playing_phrase_act(self, value):
        '''
        Esta función se llama cuando el robot terminó de ejecutar alguna de las acciones de la frase (flujo de aire + posicionamiento) para pintar la acción azul y la siguiente verde.
        '''
        if value:
            self.scoreLayout.itemAt(value-1).widget().paint_blue()
        self.scoreLayout.itemAt(value).widget().paint_green()
        self.scrolled += 100 * self.scoreLayout.itemAt(value).widget().data['time']
        self.scrollArea.horizontalScrollBar().setValue(int(self.scrolled))
    
    def change_playing_fingers_act(self, value):
        '''
        Esta función se llama cuando el robot terminó de ejecutar alguna de las acciones de los dedos para pintar la acción azul y la siguiente verde.
        '''
        if value:
            self.fingersScoreLayout.itemAt(value-1).widget().paint_blue()
        self.fingersScoreLayout.itemAt(value).widget().paint_green()

    def changes_saved(self):
        '''
        Se llama esta función cuando la partitura no tiene cambios respecto a la versión guardada.
        '''
        self.setWindowTitle("Pierre - Flutist Robot")
        self.file_saved = True

    def changes_to_save(self):
        '''
        Se llama esta función cuando la partitura tiene cambios respecto a la versión guardada.
        '''
        self.setWindowTitle("Pierre - Flutist Robot*")
        self.file_saved = False

    def find_recent_files(self):
        '''
        Se usa esta función para encontrar los archivos guardados recientemente y añadirlos al menu de 'Recent Files'
        '''
        if 'recent_saves.txt' in os.listdir(self.base_path):
            recents = []
            with open(self.base_path + '/recent_saves.txt', 'r') as file:
                for line in file.readlines():
                    line = line.replace("\n", "")
                    if line in recents:
                        pass
                    else:
                        if len(recents) < 5:
                            recents.append(line)
                            head, tail = os.path.split(line)
                            editAct = self.menuRecentFiles.addAction(tail)
                            editAct.triggered.connect(partial(self.open, line))
        else:
            pass

    def new_file(self):
        '''
        Se usa esta función para comenzar una partitura nueva, si la actual no está guardada se ofrece la posibilidad de guardar los cambios antes de cerrarlos.
        '''
        if self.scoreLayout.count() != 0 or self.fingersScoreLayout.count() != 0 or self.initialPositionLayout.count() != 0:
            msg = QMessageBox()
            msg.setText("Save changes to this score before creating a new file?")
            msg.setInformativeText("Your changes will be lost if you don't save them.")
            msg.setWindowTitle("Save Score?")
            dont_save_button = msg.addButton("Don't save", QtWidgets.QMessageBox.NoRole)
            cancel_button = msg.addButton("Cancel", QtWidgets.QMessageBox.YesRole)
            save_button = msg.addButton("Save", QtWidgets.QMessageBox.YesRole)

            retval = msg.exec_()
            if retval == 0: # don't save
                self.clean_score()
                self.filename = None
            elif retval == 1: # 
                pass
            elif retval == 2:
                self.save()
                self.clean_score()
                self.filename = None

    def save(self):
        '''
        Se usa esta función para guardar una partitura o los cambios realizados
        '''
        if self.scoreLayout.count() == 0 and self.fingersScoreLayout.count() == 0 and self.initialPositionLayout.count() == 0:
            return False
        if self.filename:
            # phrase_actions = []
            # for index in range(self.scoreLayout.count()):
            #     action = self.scoreLayout.itemAt(index).widget()
            #     action_dict = {'type': action.id, 'data': action.data}
            #     phrase_actions.append(action_dict)


            data = {'init_pos': self.initial_position, 'phrase': self.phrase_actions, 'fingers': self.finger_actions, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
            with open(self.filename, 'w') as file:
                json.dump(data, file, indent=4, sort_keys=True)
                self.changes_saved()
            if 'recent_saves.txt' in os.listdir(self.base_path):
                with open(self.base_path + '/recent_saves.txt', 'r+') as file:
                    content = file.read()
                    file.seek(0, 0)
                    file.write(self.filename + '\n' + content)
                # with open(self.base_path + '/utils/recent_saves.txt', 'a') as file:
                #     file.write('\n' + self.filename)
            else:
                with open(self.base_path + '/recent_saves.txt', 'w') as file:
                    file.write(self.filename)
        else:
            self.save_as()
        return True
        
    def save_as(self):
        '''
        Se usa esta función para guardar la partitura actual como un archivo nuevo.
        '''
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.base_path,"JSON files (*.json)")
        if fname != '':
            if fname[-5:] != '.json':
                fname += '.json'
            self.filename = fname
            self.base_path = os.path.split(fname)[0]
            self.save()

    def clean_score(self):
        '''
        Se usa esta función para borrar todas las acciones ingresadas en una partitura
        '''
        self.actionsCount = 0
        self.phrase_actions = []
        for index in reversed(range(self.scoreLayout.count())):
            self.scoreLayout.itemAt(index).widget().deleteLater()
            self.scoreLayout.removeItem(self.scoreLayout.itemAt(index))
        self.fingerActionsCount = 0
        self.finger_actions = []
        for index in reversed(range(self.fingersScoreLayout.count())):
            self.fingersScoreLayout.itemAt(index).widget().deleteLater()
            self.fingersScoreLayout.removeItem(self.fingersScoreLayout.itemAt(index))
        self.initial_position = None
        for index in reversed(range(self.initialPositionLayout.count())):
            self.initialPositionLayout.itemAt(index).widget().deleteLater()
            self.initialPositionLayout.removeItem(self.initialPositionLayout.itemAt(index))
        self.setInitialPositionButton.show()
        self.addActionButton.hide()
        self.fingersActionButton.hide()

    def open(self, fname=None):
        '''
        Se usa esta función para abrir una partitura que había sido guardada con anterioridad. Además, si hay trabajo sin guardar se ofrece la posibilidad de guardarlo antes de abrir el nuevo.
        '''
        if (self.scoreLayout.count() != 0 or self.fingersScoreLayout.count() != 0 or self.initialPositionLayout.count() != 0) and not self.file_saved:
            msg = QMessageBox()
            #msg.setIcon(QMessageBox.Critical)
            msg.setText("Save changes to this score before opening other file?")
            msg.setInformativeText("Your changes will be lost if you don't save them.")
            msg.setWindowTitle("Save Score?")
            dont_save_button = msg.addButton("Don't save", QtWidgets.QMessageBox.NoRole)
            cancel_button = msg.addButton("Cancel", QtWidgets.QMessageBox.YesRole)
            save_button = msg.addButton("Save", QtWidgets.QMessageBox.YesRole)

            retval = msg.exec_()
            if retval == 0: # don't save
                self.clean_score()
                self.open(fname=fname)
            elif retval == 1: # 
                pass
            elif retval == 2:
                self.save()
                self.clean_score()
                self.open(fname=fname)
        else:
            if not fname:
                fname, _ = QFileDialog.getOpenFileName(self, 'Open file', self.base_path,"JSON files (*.json)")
            try:
                self.clean_score()
                with open(fname) as json_file:
                    data = json.load(json_file)
                    try:
                        index = 0
                        init_position = data['init_pos']
                        phrase_actions = data['phrase']
                        finger_actions = data['fingers']
                        self.add_initial_position_action(init_position, dialog=False)
                        for act in phrase_actions:
                            self.add_action(a=0, pos=index, data=act, dialog=False)
                            index += 1
                        index = 0
                        for act in finger_actions:
                            self.add_fingers_action(a=0, pos=index, data=act, dialog=False)
                            index += 1
                        # while QApplication.hasPendingEvents():
                        #     QApplication.processEvents()
                        #self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
                        self.filename = fname
                        self.changes_saved()
                    except:
                        msg = QMessageBox()
                        msg.setIcon(QMessageBox.Critical)
                        msg.setText("Couldn't open file.")
                        msg.setInformativeText("File format does not coincide. Try with other file.")
                        msg.setWindowTitle("File Error")
                        #msg.setDetailedText("The details are as follows:")
                        retval = msg.exec_()
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Couldn't find file.")
                msg.setInformativeText("Maybe the file was moved or removed.")
                msg.setWindowTitle("File not found")
                retval = msg.exec_()

    def reconnect_flow_controller(self):
        print("TO-DO: Implement reconnecction")

    def reconnect_preasure_sensor(self):
        print("TO-DO: Implement reconnecction")

    def reconnect_x_controller(self):
        print("TO-DO: Implement reconnecction")

    def reconnect_z_controller(self):
        print("TO-DO: Implement reconnecction")

    def reconnect_angle_controller(self):
        print("TO-DO: Implement reconnecction")

    def configure_flow_control_loop(self):
        '''
        Se llama esta función para desplegar una ventana de dialogo que permite cambiar los parametros del lazo de control del controlador de flujo. Estos parámetros quedan guardados para la próxima vez que se inicie el programa.
        '''
        data=[DATA['FlowVar'],DATA['FlowLoop'],DATA['kp'],DATA['ki'],DATA['kd']]
        dlg = ConfigureFluteControlForm(parent=self, data=data)
        dlg.setWindowTitle("Configure Flute Control Loop")
        if dlg.exec():
            DATA['FlowVar'] = data[0]
            var_traduction = {0: 'M', 1: 'V', 2: 'P'}
            self.musician_pipe.send(["flow_driver.change_controlled_var", var_traduction[data[0]]])
            #self.musician.flow_driver.change_controlled_var(var_traduction[data[0]])
            DATA['FlowLoop'] = data[1]
            #self.musician.flow_driver.change_control_loop(data[1])
            self.musician_pipe.send(["flow_driver.change_control_loop", data[1]])
            DATA['kp'] = data[2]
            #self.musician.flow_driver.change_kp(data[2])
            self.musician_pipe.send(["flow_driver.change_kp", data[2]])
            DATA['ki'] = data[3]
            #self.musician.flow_driver.change_ki(data[3])
            self.musician_pipe.send(["flow_driver.change_ki", data[3]])
            DATA['kd'] = data[4]
            #self.musician.flow_driver.change_kd(data[4])
            self.musician_pipe.send(["flow_driver.change_kd", data[4]])
            save_variables()

    def plot_measure(self, measure, title):
        '''
        Se usa esta función para desplegar una ventana con el gráfico de alguna variable de interés
        '''
        plotwin = PlotWindow(self.app, measure, self.data, parent=self)
        plotwin.setWindowTitle(title)
        plotwin.show()

    def measure_radius(self):
        '''
        Para graficar la evolución en el tiempo del radio
        '''
        self.plot_measure(0, "Radius Plot")

    def measure_theta(self):
        '''
        Para graficar la evolución en el tiempo del ángulo de incidencia
        '''
        self.plot_measure(1, "Theta Plot")

    def measure_offset(self):
        '''
        Para graficar la evolución en el tiempo del offset
        '''
        self.plot_measure(2, "Offset Plot")

    def measure_position(self):
        '''
        Para graficar la evolución en el tiempo de la posición (plano XZ)
        '''
        self.plot_measure(3, "Position Plot")

    def measure_mouth_presure(self):
        '''
        Para graficar la evolución en el tiempo de la presión en la boca
        '''
        self.plot_measure(4, "Mouth Preasure Plot")

    def measure_mass_flow_rate(self):
        '''
        Para graficar la evolución en el tiempo del flujo másico
        '''
        self.plot_measure(5, "Mass Flow Rate Plot")

    def measure_volume_flow_rate(self):
        '''
        Para graficar la evolución en el tiempo del flujo volumétrico
        '''
        self.plot_measure(6, "Volume Flow Rate Plot")

    def measure_temperature(self):
        '''
        Para graficar la evolución en el tiempo de la temperatura del flujo
        '''
        self.plot_measure(7, "Flow Temperature Plot")

    def measure_sound_frequency(self):
        '''
        Para graficar la evolución en el tiempo de la frecuencia del sonido
        '''
        self.plot_measure(8, "Sound Frequency Plot")

    def measure_x_position(self):
        '''
        Para graficar la evolución en el tiempo de x
        '''
        self.plot_measure(9, "X Position Plot")

    def measure_z_position(self):
        '''
        Para graficar la evolución en el tiempo de z
        '''
        self.plot_measure(10, "Z Position Plot")
    
    def measure_alpha_position(self):
        '''
        Para graficar la evolución en el tiempo de alpha
        '''
        self.plot_measure(11, "Alpha Position Plot")

    # def autohome_routine(self):
    #     '''
    #     Esta función ejecuta la función de homing de los motores. Los ejes X y Z son automáticos, en cambio para alpha aparece una ventana de dialogo donde se le da instrucciones al motor para llevarlo a su posición de origen (donde la boca se encuentra horizontal)
    #     '''
    #     # while(not self.player.motors_control.started):
    #     #     pass
    #     self.musician.auto_home()
    #     data=[0]
    #     #self.alpha_driver.request_write_reset_errors()
    #     self.autohomeDlg = CalibrateAngleForm(parent=self, data=data)
    #     self.autohomeDlg.angleSpinBox.valueChanged.connect(self.change_motor_angle)
    #     self.autohomeDlg.setWindowTitle("Choose parameters")
    #     self.musician.motors_controller.home_alpha()
    #     if self.autohomeDlg.exec():
    #         pass
    #     #self.musician.motors_controller.homed()
    #     self.musician.motors_controller.homed()
    #     self.moveBox.set_values(self.state)
    #     #self.desired_state.change_state(self.state)
    #     self.musician.finish_autohome()
    
    # def autohome_X_routine(self):
    #     self.musician.auto_home(z=False, alpha=False)

    # def autohome_Z_routine(self):
    #     self.musician.auto_home(x=False, alpha=False)

    # def autohome_Alpha_routine(self):
    #     data=[0]
    #     #self.alpha_driver.request_write_reset_errors()
    #     self.autohomeDlg = CalibrateAngleForm(parent=self, data=data)
    #     self.autohomeDlg.angleSpinBox.valueChanged.connect(self.change_motor_angle)
    #     self.autohomeDlg.setWindowTitle("Choose parameters")
    #     #self.musician.motors_controller.home_alpha()
    #     if self.autohomeDlg.exec():
    #         pass
        
    #     self.musician.alpha_driver.homing_event.clear()
    #     state = self.musician.get_ref_state()
    #     self.moveBox.set_values(state)

    def score_generate_states(self, b=False, selection=[], min_time=0):
        #print(selection)
        if len(selection) == 0:
            selection = [False for i in range(len(self.finger_actions))]
            #print(selection)
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your fascinating new score")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        notes = [{'note': self.finger_actions[i]['note'], 'time': self.finger_actions[i]['time'], 'selected': selection[i]} for i in range(len(self.finger_actions))]
        data=[70, notes, min_time]
        #self.alpha_driver.request_write_reset_errors()
        dlg = StatesFromNotesForm(parent=self, data=data)
        dlg.setWindowTitle("Choose configuration")
        if dlg.exec():
            acc = data[0]
            selection = [i['selected'] for i in data[1]]
            min_time = data[2]
            #print(selection)
            try:
                generate_states_from_notes(self.filename, self.filename, acc=acc, selection=selection, min_time_change=min_time)
                self.clean_score()
                self.open(fname=self.filename)
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Couldn't execute action")
                msg.setInformativeText("There was an error, try different acceleration or note selection.")
                msg.setWindowTitle("Action Error")
                retval = msg.exec_()
                self.score_generate_states(selection=selection, min_time=min_time)
            

    def score_generate_states_diff(self, fname=False, selection=[], min_time=0):
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted two different file dialogs")
            msg.setInformativeText("In the first one, you should select where to store your current work. In the second one, where you would like to store the new one.")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        else:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your new work")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        if not fname:
            fname, _ = QFileDialog.getSaveFileName(self, 'Where would you like to store the new file?', self.base_path,"JSON files (*.json)")
        if fname:
            if fname[-5:] != '.json':
                fname += '.json'

            if len(selection) == 0:
                selection = [False for i in range(len(self.finger_actions))]

            notes = [{'note': self.finger_actions[i]['note'], 'time': self.finger_actions[i]['time'], 'selected': selection[i]} for i in range(len(self.finger_actions))]

            data=[70, notes, min_time]
            dlg = StatesFromNotesForm(parent=self, data=data)
            dlg.setWindowTitle("Choose configuration")
            if dlg.exec():
                acc = data[0]
                selection = [i['selected'] for i in data[1]]
                min_time = data[2]
                #print(selection)
                try:
                    generate_states_from_notes(self.filename, self.filename, acc=acc, selection=selection, min_time_change=min_time)
                    self.clean_score()
                    self.open(fname=fname)
                except:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Critical)
                    msg.setText("Couldn't execute action")
                    msg.setInformativeText("There was an error, try different acceleration or note selection.")
                    msg.setWindowTitle("Action Error")
                    retval = msg.exec_()
                    self.score_generate_states_diff(fname=fname, selection=selection, min_time=min_time)
            
            # generate_states_from_notes(self.filename, fname)
            # self.clean_score()
            # self.open(fname=fname)

    def score_scale_time(self, b=0, scale=1):
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your fascinating new score")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        data=[scale, False]
        dlg = ScaleTimeForm(parent=self, data=data)
        dlg.setWindowTitle("Choose scale")
        if dlg.exec():
            scale = data[0]
            try:
                change_speed(self.filename, scale, self.filename, notes_only=data[1])
                self.clean_score()
                self.open(fname=self.filename)
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Couldn't execute action")
                msg.setInformativeText("There was an error, consider scaling only the notes.")
                msg.setWindowTitle("Action Error")
                retval = msg.exec_()
                self.score_scale_time(scale=scale)
            
    def score_scale_time_diff(self, scale=1, fname=False):
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted two different file dialogs")
            msg.setInformativeText("In the first one, you should select where to store your current work. In the second one, where you would like to store the new one.")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        else:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your new work")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        if not fname:
            fname, _ = QFileDialog.getSaveFileName(self, 'Where would you like to store the new file?', self.base_path,"JSON files (*.json)")
        if fname:
            data=[scale, False]
            dlg = ScaleTimeForm(parent=self, data=data)
            dlg.setWindowTitle("Choose scale")
            if dlg.exec():
                scale = data[0]
                try:
                    change_speed(self.filename, scale, fname, notes_only=data[1])
                    self.clean_score()
                    self.open(fname=fname)
                except:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Critical)
                    msg.setText("Couldn't execute action")
                    msg.setInformativeText("There was an error, consider scaling only the notes.")
                    msg.setWindowTitle("Action Error")
                    retval = msg.exec_()
                    self.score_scale_time_diff(scale=scale, fname=fname)

    def score_add_correction(self):
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your fascinating new score")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        data=[0, 0, 0, 0]
        dlg = ParamCorrectionForm(parent=self, data=data)
        dlg.setWindowTitle("Choose the correction to each parameter")
        if dlg.exec():
            r_correction = data[0]
            theta_correction = data[1]
            offset_correction = data[2]
            flow_correction = data[3]
            add_correction(self.filename, r_correction, theta_correction, offset_correction, flow_correction, self.filename)
            self.clean_score()
            self.open(fname=self.filename)

    def score_add_correction_diff(self):
        if not self.filename:
            msg = QMessageBox()
            msg.setText("You will be prompted two different file dialogs")
            msg.setInformativeText("In the first one, you should select where to store your current work. In the second one, where you would like to store the new one.")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        else:
            msg = QMessageBox()
            msg.setText("You will be prompted a file dialog")
            msg.setInformativeText("You should select where to store your new work")
            msg.setWindowTitle("Instructions")
            msg.exec_()
        self.save()
        if not self.filename:
            return
        fname, _ = QFileDialog.getSaveFileName(self, 'Where would you like to store the new file?', self.base_path,"JSON files (*.json)")
        if fname:
            data=[0, 0, 0, 0]
            dlg = ParamCorrectionForm(parent=self, data=data)
            dlg.setWindowTitle("Choose the correction to each parameter")
            if dlg.exec():
                r_correction = data[0]
                theta_correction = data[1]
                offset_correction = data[2]
                flow_correction = data[3]
                add_correction(self.filename, r_correction, theta_correction, offset_correction, flow_correction, fname)
                self.clean_score()
                self.open(fname=fname)

    def zoom_score(self, *args, disp=True):
        if disp:
            data=[self.zoom_factor]
            dlg = ZoomScoreForm(parent=self, data=data)
            dlg.setWindowTitle("Choose the scale")
            if dlg.exec():
                self.zoom_factor = data[0]
                for index in range(self.scoreLayout.count()):
                    self.scoreLayout.itemAt(index).widget().zoom(self.zoom_factor) 
                for index in range(self.fingersScoreLayout.count()):
                    self.fingersScoreLayout.itemAt(index).widget().zoom(self.zoom_factor) 
        else:
            for index in range(self.scoreLayout.count()):
                self.scoreLayout.itemAt(index).widget().zoom(self.zoom_factor) 
            for index in range(self.fingersScoreLayout.count()):
                self.fingersScoreLayout.itemAt(index).widget().zoom(self.zoom_factor) 
        #print(args, disp)
        #print(f'Zoom a factor of {self.zoom_factor}')

    def update_default_positions(self):
        update_note_position()
        self.moveBox.collapsible_update_note_position()

    # def change_motor_angle(self, value):
    #     '''
    #     Esta función se usa para el homing del eje alpha, conecta los valores que se introducen en el spinbox con el driver del motor.
    #     '''
    #     self.musician.move_to_alpha(value)

    def stop_motors(self):
        self.musician_pipe.send(["stop"])
        self.musician_pipe.send(['get_ref_state'])
        state = self.musician_pipe.recv()[0]
        print("Stop motors", state)
        self.moveBox.set_values(state)
     
    def change_flute_position(self):
        '''
        Esta función despliega una ventana para introducir la posición del bisel de la flauta. Al cambiar los valores estos quedan guardados para la próxima vez que se inicie el programa.
        '''
        data=[DATA['X_F'],DATA['Z_F']]
        dlg = CalibrateFluteForm(parent=self, data=data)
        dlg.setWindowTitle("Choose parameters")
        if dlg.exec():
            DATA['X_F'] = data[0]
            DATA['Z_F'] = data[1]
            save_variables()

    def updateFingerIndexes(self):
        '''
        Actualiza los índices de las acciones de los dedos cuando se introduce una en medio.
        '''
        for index in range(self.fingersScoreLayout.count()):
            self.fingersScoreLayout.itemAt(index).widget().index = index
            # QApplication.processEvents()

    def updateIndexes(self):
        '''
        Actualiza los índices de las acciones de la frase musical (posición + flujo) cuando se introduce una en medio.
        '''
        #print('Count:', self.scoreLayout.count())
        for index in range(self.scoreLayout.count()):
            self.scoreLayout.itemAt(index).widget().index = index
            # QApplication.processEvents()

    def add_initial_position_action(self, data=None, dialog=True):
        '''
        Con esta función se agrega una acción de posicionamiento inicial.
        '''
        if not data:
            self.musician_pipe.send(['get_ref_state'])
            state = self.musician_pipe.recv()[0]
            print("Initial position action", state)
            data={'type': 0, 'r': state.r, 'theta': state.theta,'offset': state.o}
        if dialog:
            dlg = StartActionForm(parent=self, data=data)
            dlg.setWindowTitle("Choose parameters for initial position")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            startPosAction = ActionWidget(data, 'Move to initial position', width=2, parent=self,
                                          context=self.initialPositionLayout, index=0)
            self.saving = self.initialPositionLayout.insertWidget(0, startPosAction)
            #self.phrase_actions.insert(0, {'type': 0, 'data': data})
            #self.finger_actions.insert(0, {'note': data['key']})
            self.initial_position = data
            # while QApplication.hasPendingEvents():
            #     QApplication.processEvents()
            #self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
            self.fingersActionButton.show()
            self.addActionButton.show()
            self.setInitialPositionButton.hide()
            self.changes_to_save()
        
        return rsp

    def add_fingers_action(self, a=0, pos=-1, data=None, dialog=True):
        '''
        Con esta función se agrega una acción de los dedos.
        '''
        if pos == -1:
            pos = self.fingerActionsCount
        if not data:
            note = self.get_previous_note(pos)
            data = {'type': 2, 'time': 1.0, 'note': note}
        if dialog:
            dlg = FingersActionForm(parent=self, data=data, index=pos, instrument=self.instrument)
            dlg.setWindowTitle("Choose parameters")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            newAction = ActionWidget(data, str(data['note']), width=data['time'], parent=self,
                                     context=self.fingersScoreLayout, index=pos)
            self.fingerActionsCount += 1
            self.fingersScoreLayout.insertWidget(pos, newAction)
            self.finger_actions.insert(pos, data)
            # while QApplication.hasPendingEvents():
            #     QApplication.processEvents()
            #self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
            self.changes_to_save()
        self.zoom_score(disp=False)
        return rsp

    def add_action(self, a=0, pos=-1, data=None, dialog=True):
        '''
        Con esta función se agrega una acción de la frase musical (posición + flujo)
        '''
        if pos == -1:
            pos = self.actionsCount
        if not data:
            r, theta, o, f, v_a, v_f = self.get_previous_pos(pos)
            data = {'type': 1, 'move': 0, 'time': 1.0, 'r': r, 'theta': theta, 'offset': o, 'acceleration': 153,
                    'deceleration': 153, 'jerk': 0, 'flow': f, 'deformation': 1, 'vibrato_amp': v_a, 'vibrato_freq': v_f}
        if dialog:
            dlg = MoveActionForm(parent=self, data=data, index=pos)
            dlg.setWindowTitle("Choose parameters")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            while not self.validate_action(data, pos):
                msg = QMessageBox()
                msg.setText("There was an error while submiting the action.")
                msg.setInformativeText("You need to change the parameters to fit the restrictions.")
                msg.setWindowTitle("Invalid Movement")
                msg.exec_()
                rsp = dlg.exec()
                if not rsp:
                    break
            
            if data['move']:
                label = f'Move to...\n R: {data["r"]} \n Theta: {data["theta"]} \n Offset: {data["offset"]} \n Flow: {data["flow"]}'
            else:
                label = 'Stay'
            newAction = ActionWidget(data, label, width=data['time'], parent=self, context=self.scoreLayout,
                                     index=pos)
            self.actionsCount += 1
            self.scoreLayout.insertWidget(pos, newAction)
            self.phrase_actions.insert(pos, data)
            # while QApplication.hasPendingEvents():
            #     QApplication.processEvents()
            #self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
            self.changes_to_save()
        self.zoom_score(disp=False)
        return rsp
        
    def get_previous_note(self, pos):
        '''
        Retorna la nota anterior al de la acción de dedos en la posición pos
        '''
        if pos != 0:
            return self.finger_actions[-1]['note']
        return 'C4'

    def get_previous_pos(self, pos):
        '''
        Retorna el estado anterior al de una acción que se encuentra en la posición pos
        '''
        i = pos - 1
        found = False
        while i >= 0:
            w = self.phrase_actions[i]
            if w['move'] == 1:
                r = w['r']
                theta = w['theta']
                o = w['offset']
                f = w['flow']
                v_a = w['vibrato_amp']
                v_f = w['vibrato_freq']
                found = True
                break
            i -= 1
        if not found:
            r = self.initial_position['r']
            theta = self.initial_position['theta']
            o = self.initial_position['offset']
            f = 0
            v_a = 0
            v_f = 0
        return r, theta, o, f, v_a, v_f
     
    def validate_action(self, actionData, pos):
        '''
        Esta función se usa para validar que una acción de la frase musical sea posible de realizar.
        '''
        #print("TO-DO: validate action")
        #print(actionData)
        if not actionData['move']:
            return True

        r, theta, o, f, v_a, v_f = self.get_previous_pos(pos)
        prevous_state = State(r, theta, o, f, vibrato_amp=v_a, vibrato_freq=v_f)
        desired_state = State(actionData['r'], actionData['theta'], actionData['offset'], actionData['flow'], vibrato_freq=actionData['vibrato_freq'], vibrato_amp=actionData['vibrato_amp'])
        x_points, z_points, alpha_points, d = get_route_positions(*prevous_state.cart_coords(), *desired_state.cart_coords(), divisions=12, plot=False)
        acc = actionData['acceleration']
        dec = actionData['deceleration']

        if acc == 0 or dec == 0:
            return False
            
        T   = actionData['time']
        if max_dist_rec(acc, dec, T) < d[-1]:
            print(f'Impossible to achieve such position with given acceleration and deceleration. {d[-1]} > {max_dist_rec(acc, dec, T)}')
            #print(3, acc, dec)
            return False

        return True

    def select_instrument(self):
        """
        Esta función despliega el diálogo de selección de instrumento
        """
        self.instrument_dialog = InstrumentForm(parent=self)
        self.instrument_dialog.comboBox.currentIndexChanged.connect(self.change_player_instrument)
        if self.instrument_dialog.exec():
            pass

    def change_player_instrument(self, value):
        """
        Esta función provoca un cambio de instrumento en el driver de dedos del músico
        """
        self.instrument = self.instrument_dialog.comboBox.itemText(value).lower()
        self.musician_pipe.send['set_instrument', self.instrument_dialog.comboBox.itemText(value).lower()]


if __name__ == "__main__":
    app = QApplication(sys.argv)

    host = "192.168.2.10"
    connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]
    event = threading.Event()
    event.set()

    t0 = time()
    pierre = Musician(host, connections, event, fingers_connect=False, x_connect=False, z_connect=False, alpha_connect=False, flow_connect=False, preasure_sensor_connect=False)
    pierre.start()

    win = Window(app, event, pierre)
    win.show()

    sys.exit(app.exec())
