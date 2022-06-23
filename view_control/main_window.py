from importlib.util import set_loader
from ssl import AlertDescription
import sys

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout, QGridLayout, QWidget, QLabel, QSpinBox, QMenu, QShortcut
)
from PyQt5.QtCore import QEventLoop, Qt
from PyQt5.QtGui import QPainterPath, QRegion, QKeySequence

from PyQt5.uic import loadUi
from matplotlib.pyplot import close
from regex import D
from utils.driver_amci import AMCIDriver
#from regex import D
from views.main_window import Ui_MainWindow
from view_control.forms import FingersActionForm, MoveActionForm, CalibrateFluteForm, CalibrateAngleForm, ConfigureFluteControlForm, StartActionForm

from view_control.collapsible_widgets import ManualMoveCollapsibleBox
from view_control.action_display import ActionWidget
from view_control.plot_window import Window as PlotWindow
from utils.cinematica import *
from view_control.amci_control import AMCIWidget

from utils.player import *

#from dialog_control import Ui_Dialog
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import random

import json
import typing
import os
import serial
import threading
import math
import queue
from time import time, sleep
import io
from functools import partial
from pybase64 import b64decode
from datetime import date, datetime
from numpy import *

class Window(QMainWindow, Ui_MainWindow):
    '''
    Esta clase conecta los elementos de la GUI principal con todas las funciones que se quieren realizar con el robot.
    '''
    def __init__(self, app, preasure_sensor_event, flow_controler_event, x_event, z_event, alpha_event, microphone_event, musician_event, musician, state, parent=None):
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
        #self.performing = False
        #self.playing = False
        #self.vars = read_variables()
        self.state = state
        #self.state.homed()
        self.desired_state = State(0,0,0,0)
        self.desired_state.homed()

        self.musician_event = musician_event
        self.musician = musician
        self.preasure_sensor_event = preasure_sensor_event
        #self.preasure_sensor = self.musician.preasure_sensor
        self.flow_controler_event = flow_controler_event
        #self.flow_controller = self.musician.flow_controller
        self.x_event = x_event
        #self.x_driver = self.musician.x_drive
        self.z_event = z_event
        #self.z_driver = self.musician.z_drive
        self.alpha_event = alpha_event
        #self.alpha_driver = self.musician.alpha_drive
        self.microphone_event = microphone_event
        #self.microphone = self.musician.microphone

        self.initial_position = None
        self.phrase_actions = []
        self.finger_actions = []
        #self.performing = threading.Event()
        #self.playing = threading.Event()
        #self.player = Player(self.phrase_actions, self.state, self.desired_state, self.performing, self.playing, self.flow_controller, self.preasure_sensor, self.x_driver, self.z_driver, self.alpha_driver, self.microphone, parent=self)
        #self.player.motors_control.write_at_home()
        self.scrolled = 0

        self.moveBox = ManualMoveCollapsibleBox("Manual Move", parent=self, state=self.state, desired_state=self.desired_state, playing=self.musician.playing)
        self.moveBox.stopButton.clicked.connect(self.stop_motors)
        self.gridLayout.addWidget(self.moveBox, 3, 0, 1, 8)

        self.setWindowTitle("Flutist Robot UC")
        self.connectSignalsSlots()
        self.file_saved = True

        self.find_recent_files()
        
        self.autohome_routine()

        self.musician.recorder.start()

    def closeEvent(self, a0: QtGui.QCloseEvent):
        '''
        Esta función se ejecuta al cerrar el programa, para terminar todos los threads que están corriendo
        '''
        self.musician.motors_controller.reset_drivers()
        self.musician_event.clear()
        #self.player.flowSignalEvent.clear()
        #self.player.moveSignalEvent.clear()
        self.preasure_sensor_event.clear()
        self.flow_controler_event.clear()
        self.x_event.clear()
        self.z_event.clear()
        self.alpha_event.clear()
        self.microphone_event.clear()
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
        self.actionAutoHomeRoutine.triggered.connect(self.autohome_routine)
        
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

        self.actionReconnectFlowController.triggered.connect(self.reconnect_flow_controller)
        self.actionReconnectPreasureSensor.triggered.connect(self.reconnect_preasure_sensor)
        self.actionReconnectXController.triggered.connect(self.reconnect_x_controller)
        self.actionReconnectZController.triggered.connect(self.reconnect_z_controller)
        self.actionReconnectAngleController.triggered.connect(self.reconnect_angle_controller)

        self.actionConfigureFlowControlLoop.triggered.connect(self.configure_flow_control_loop)

        self.actionSave.triggered.connect(self.save)
        self.actionSaveAs.triggered.connect(self.save_as)
        self.actionOpen_2.triggered.connect(self.open)
        self.fastSave = QShortcut(QKeySequence('Ctrl+S'), self)
        self.fastSave.activated.connect(self.save)
        self.actionNew.triggered.connect(self.new_file)

        self.pauseButton.clicked.connect(self.pause)
        self.pauseButton.hide()
        self.stopButton.clicked.connect(self.stop)
        self.stopButton.hide()
        self.executeButton.clicked.connect(self.execute_score)

        self.actionXAxisTool.triggered.connect(self.open_x_driver_tool)
        self.actionZAxisTool.triggered.connect(self.open_z_driver_tool)
        self.actionAlphaAxisTool.triggered.connect(self.open_alpha_driver_tool)

        self.musician.finished_score.connect(self.finished_score)
        self.musician.finished_initial_positioning.connect(self.change_playing_initial_position)
        self.musician.begin_phrase_action.connect(self.change_playing_phrase_act)
        self.musician.begin_finger_action.connect(self.change_playing_fingers_act)

    def open_x_driver_tool(self):
        '''
        Abre una ventana con herramientas para controlar el driver del eje X
        '''
        toolwin = AMCIWidget(self.musician.x_drive, parent=self)
        toolwin.setWindowTitle('X Driver Tool')
        toolwin.show()

    def open_z_driver_tool(self):
        '''
        Abre una ventana con herramientas para controlar el driver del eje Z
        '''
        toolwin = AMCIWidget(self.musician.z_drive, parent=self)
        toolwin.setWindowTitle('Z Driver Tool')
        toolwin.show()

    def open_alpha_driver_tool(self):
        '''
        Abre una ventana con herramientas para controlar el driver del eje Alpha
        '''
        toolwin = AMCIWidget(self.musician.alpha_drive, parent=self)
        toolwin.setWindowTitle('Alpha Driver Tool')
        toolwin.show()

    def pause(self):
        '''
        Mientras se está ejecutando una partitura es posible detenerla (hacer una pausa) con la posibilidad de después seguir ejecutándola desde donde se dejó.
        '''
        if self.musician.playing.is_set():
            self.pauseButton.setText('Play')
            self.musician.playing.clear()
            self.moveBox.enableButtons()
            while QApplication.hasPendingEvents():
                QApplication.processEvents()
            self.moveBox.set_values(self.state)

        else:
            self.pauseButton.setText('Pause')
            self.moveBox.disableButtons()
            self.musician.playing.set()
    
    def execute_score(self):
        '''
        Esta función comienza la ejecución de la partitura que haya sido ingresada desde la interfaz.
        '''
        self.pauseButton.show()
        self.stopButton.show()
        self.executeButton.hide()
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

        self.musician.initial_position = self.initial_position
        self.musician.phrase_instructions = self.phrase_actions
        self.musician.finger_instructions = self.finger_actions

        self.musician.playing.set()
        self.musician.performing.set()

        if self.initialPositionLayout.count():
            self.initialPositionLayout.itemAt(0).widget().paint_green()

    def stop(self):
        '''
        Esta función se usa para detener una partitura que se esté ejecutando
        '''
        #print('Stop clicked')
        self.musician.performing.clear()
        self.musician.playing.clear()
        self.pauseButton.hide()
        self.stopButton.hide()
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
        while QApplication.hasPendingEvents():
            QApplication.processEvents()
        while self.musician.moving():
            pass
        self.moveBox.set_values(self.state)

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
        self.setWindowTitle("Flutist Robot UC")
        self.file_saved = True

    def changes_to_save(self):
        '''
        Se llama esta función cuando la partitura tiene cambios respecto a la versión guardada.
        '''
        self.setWindowTitle("Flutist Robot UC*")
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
            elif retval == 1: # 
                pass
            elif retval == 2:
                self.save()
                self.clean_score()

    def save(self):
        '''
        Se usa esta función para guardar una partitura o los cambios realizados
        '''
        if self.scoreLayout.count() == 0 and self.fingersScoreLayout.count() == 0 and self.initialPositionLayout.count() == 0:
            return
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
        
    def save_as(self):
        '''
        Se usa esta función para guardar la partitura actual como un archivo nuevo.
        '''
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.base_path,"JSON files (*.json)")
        if fname[-5:] != '.json':
            fname += '.json'
        self.filename = fname
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
                        while QApplication.hasPendingEvents():
                            QApplication.processEvents()
                        self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
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
            self.musician.flow_controller.change_controlled_var(var_traduction[data[0]])
            DATA['FlowLoop'] = data[1]
            self.musician.flow_controller.change_control_loop(data[1])
            DATA['kp'] = data[2]
            self.musician.flow_controller.change_kp(data[2])
            DATA['ki'] = data[3]
            self.musician.flow_controller.change_ki(data[3])
            DATA['kd'] = data[4]
            self.musician.flow_controller.change_kd(data[4])
            save_variables()

    def plot_measure(self, measure, title):
        '''
        Se usa esta función para desplegar una ventana con el gráfico de alguna variable de interés
        '''
        plotwin = PlotWindow(self.app, measure, self.musician.recorder, parent=self)
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

    def autohome_routine(self):
        '''
        Esta función ejecuta la función de homing de los motores. Los ejes X y Z son automáticos, en cambio para alpha aparece una ventana de dialogo donde se le da instrucciones al motor para llevarlo a su posición de origen (donde la boca se encuentra horizontal)
        '''
        # while(not self.player.motors_control.started):
        #     pass
        self.musician.auto_home()
        data=[0]
        #self.alpha_driver.request_write_reset_errors()
        self.autohomeDlg = CalibrateAngleForm(parent=self, data=data)
        self.autohomeDlg.angleSpinBox.valueChanged.connect(self.change_motor_angle)
        self.autohomeDlg.setWindowTitle("Choose parameters")
        self.musician.motors_controller.home_alpha()
        if self.autohomeDlg.exec():
            pass
        #self.musician.motors_controller.homed()
        self.musician.motors_controller.homed()
        self.moveBox.set_values(self.state)
        #self.desired_state.change_state(self.state)
        self.musician.finish_autohome()
    
    def change_motor_angle(self, value):
        '''
        Esta función se usa para el homing del eje alpha, conecta los valores que se introducen en el spinbox con el driver del motor.
        '''
        self.musician.motors_controller.move_alpha(value)

    def stop_motors(self):
        self.musician.stop()
        sleep(0.4)
        self.moveBox.set_values(self.state)
     
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
            QApplication.processEvents()

    def updateIndexes(self):
        '''
        Actualiza los índices de las acciones de la frase musical (posición + flujo) cuando se introduce una en medio.
        '''
        #print('Count:', self.scoreLayout.count())
        for index in range(self.scoreLayout.count()):
            self.scoreLayout.itemAt(index).widget().index = index
            QApplication.processEvents()

    def add_initial_position_action(self, data=None, dialog=True):
        '''
        Con esta función se agrega una acción de posicionamiento inicial.
        '''
        if not data:
            data={'type': 0, 'r': self.state.r, 'theta': self.state.theta,'offset': self.state.o}
        if dialog:
            dlg = StartActionForm(parent=self, data=data)
            dlg.setWindowTitle("Choose parameters for initial position")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            startPosAction = ActionWidget(data, 'Move to initial position', width=2, parent=self, context=self.initialPositionLayout, index=0)
            self.initialPositionLayout.insertWidget(0, startPosAction)
            #self.phrase_actions.insert(0, {'type': 0, 'data': data})
            #self.finger_actions.insert(0, {'note': data['key']})
            self.initial_position = data
            while QApplication.hasPendingEvents():
                QApplication.processEvents()
            self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
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
            data={'type': 2, 'time': 1.0, 'note': note}
        if dialog:
            dlg = FingersActionForm(parent=self, data=data, index=pos)
            dlg.setWindowTitle("Choose parameters")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            newAction = ActionWidget(data, str(data['note']), width=data['time'], parent=self, context=self.fingersScoreLayout, index=pos)
            self.fingerActionsCount += 1
            self.fingersScoreLayout.insertWidget(pos, newAction)
            self.finger_actions.insert(pos, data)
            while QApplication.hasPendingEvents():
                QApplication.processEvents()
            self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
            self.changes_to_save()

        return rsp

    def add_action(self, a=0, pos=-1, data=None, dialog=True):
        '''
        Con esta función se agrega una acción de la frase musical (posición + flujo)
        '''
        if pos == -1:
            pos = self.actionsCount
        if not data:
            r, theta, o, f, v_a, v_f = self.get_previous_pos(pos)
            data={'type': 1, 'move': 0, 'time': 1.0, 'r': r, 'theta': theta, 'offset': o, 'acceleration': 0, 'deceleration': 0, 'jerk': 0, 'flow': f, 'deformation': 1, 'vibrato_amp': v_a, 'vibrato_freq': v_f}
        if dialog:
            dlg = MoveActionForm(parent=self, data=data, index=pos)
            dlg.setWindowTitle("Choose parameters")
            rsp = dlg.exec()
        else:
            rsp = True
        if rsp:
            while not self.validate_action(data):
                msg = QMessageBox()
                msg.setText("There was an error while submiting the action.")
                msg.setInformativeText("You need to change the parameters to fit the restrictions.")
                msg.setWindowTitle("Invalid Movement")
                msg.exec_()
                rsp = dlg.exec()
                if not rsp:
                    break

            newAction = ActionWidget(data, 'Changing State', width=data['time'], parent=self, context=self.scoreLayout, index=pos)
            self.actionsCount += 1
            self.scoreLayout.insertWidget(pos, newAction)
            self.phrase_actions.insert(pos, data)
            while QApplication.hasPendingEvents():
                QApplication.processEvents()
            self.scrollArea.horizontalScrollBar().setValue(self.scrollArea.horizontalScrollBar().maximum())
            self.changes_to_save()

        return rsp
        
    def get_previous_note(self, pos):
        '''
        Retorna la nota anterior al de la acción de dedos en la posición pos
        '''
        if pos != 0:
            return self.finger_actions[-1]['note']
        return 0

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
     
    def validate_action(self, actionData):
        '''
        Esta función se usa para validar que una acción de la frase musical sea posible de realizar.
        '''
        print("TO-DO: validate action")
        return True

if __name__ == "__main__":
    app = QApplication(sys.argv)

    win = Window(app)
    win.show()

    sys.exit(app.exec())