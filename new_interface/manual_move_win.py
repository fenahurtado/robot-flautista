from PyQt5 import QtGui, QtCore, QtWidgets
from manual_move import Ui_MainWindow as ManualWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
import json
from route import dict_notes, dict_notes_rev
from cinematica import State

with open('new_interface/look_up_table.json', 'r') as f:
    LOOK_UP_TABLE = json.load(f)

class ManualWindow(QMainWindow, ManualWindow):
    stop_playing = QtCore.pyqtSignal()
    def __init__(self, app, musician_pipe, data, parent=None, connected=False):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        self.musician_pipe = musician_pipe
        self.data = data
        self.noteComboBox.addItems(list(dict_notes.values()))

        self.tableWidget.setColumnCount(4)
        self.tableWidget.setRowCount(len(LOOK_UP_TABLE))

        labels = list(LOOK_UP_TABLE.keys())
        self.tableWidget.setVerticalHeaderLabels(labels)

        labels = ["r [mm]", "theta [°]", "offset [mm]", "flow [SLPM]"]
        self.tableWidget.setHorizontalHeaderLabels(labels)

        self.speed = 50

        row = 0
        for value in LOOK_UP_TABLE.values():
            item = QtWidgets.QTableWidgetItem(str(value['r']))
            self.tableWidget.setItem(row, 0, item)
            item = QtWidgets.QTableWidgetItem(str(value['theta']))
            self.tableWidget.setItem(row, 1, item)
            item = QtWidgets.QTableWidgetItem(str(value['offset']))
            self.tableWidget.setItem(row, 2, item)
            item = QtWidgets.QTableWidgetItem(str(value['flow']))
            self.tableWidget.setItem(row, 3, item)
            row += 1

        self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tableWidget.itemChanged.connect(self.item_changed)

        self.moving_with_notes = False
        self.changing_other = False
        self.desired_state = State(0,0,0,0)
        self.desired_state.x = self.data['x'][-1]
        self.desired_state.z = self.data['z'][-1]
        self.desired_state.alpha = self.data['alpha'][-1]
        self.moveToNoteCheckBox.stateChanged.connect(self.move_to_notes_change)

        self.xSpinBox.setValue(self.data['x'][-1])
        self.zSpinBox.setValue(self.data['z'][-1])
        self.alphaSpinBox.setValue(self.data['alpha'][-1])
        self.rSpinBox.setValue(self.data['radius'][-1])
        self.thetaSpinBox.setValue(self.data['theta'][-1])
        self.offsetSpinBox.setValue(self.data['offset'][-1])
        self.flowSpinBox.setValue(self.data['mass_flow'][-1])

        self.xSpinBox.valueChanged.connect(self.change_x)
        self.zSpinBox.valueChanged.connect(self.change_z)
        self.alphaSpinBox.valueChanged.connect(self.change_angle)
        self.rSpinBox.valueChanged.connect(self.change_r)
        self.thetaSpinBox.valueChanged.connect(self.change_theta)
        self.offsetSpinBox.valueChanged.connect(self.change_offset)
        self.flowSpinBox.valueChanged.connect(self.change_flow)
        self.freqSpinBox.valueChanged.connect(self.change_flow_vibrato)
        self.ampSpinBox.valueChanged.connect(self.change_flow_vibrato_amp)
        self.noteComboBox.currentIndexChanged.connect(self.change_note)
        self.speedSlider1.valueChanged.connect(self.change_speed)
    
    def change_speed(self, value):
        self.speed = value

    def change_note(self, value):
        #print(self.comboBoxNote.itemText(value))
        #self.parent.musician.execute_fingers_action(self.comboBoxNote.itemText(value), through_action=False)
        self.musician_pipe.send(['execute_fingers_action', self.noteComboBox.itemText(value), False])
        if self.moving_with_notes:
            self.changing_other = True
            pos = LOOK_UP_TABLE[self.noteComboBox.itemText(value)]
            self.desired_state.r = pos['r']
            self.desired_state.theta = pos['theta']
            self.desired_state.o = pos['offset']
            self.desired_state.flow = pos['flow']
            self.desired_state.vibrato_amp = 0
            self.desired_state.vibrato_freq = 0
            #self.musician.move_to(self.desired_state)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def move_to_notes_change(self, value):
        self.moving_with_notes = value
        self.rSpinBox.setEnabled(not value)
        self.thetaSpinBox.setEnabled(not value)
        self.offsetSpinBox.setEnabled(not value)
        self.xSpinBox.setEnabled(not value)
        self.zSpinBox.setEnabled(not value)
        self.alphaSpinBox.setEnabled(not value)
        self.flowSpinBox.setEnabled(not value)

    def set_values(self, state):
        self.changing_other = True
        self.rSpinBox.setValue(state.x)
        self.thetaSpinBox.setValue(state.z)
        self.offsetSpinBox.setValue(state.alpha)
        self.xSpinBox.setValue(state.r)
        self.zSpinBox.setValue(state.theta)
        self.alphaSpinBox.setValue(state.o)
        self.flowSpinBox.setValue(state.flow)
        self.freqSpinBox.setValue(state.vibrato_freq)
        self.ampSpinBox.setValue(state.vibrato_amp)
        self.desired_state.change_state(state)
        self.changing_other = False

    def update_values(self):
        #print(self.state)
        self.flowSpinBox.setValue(self.desired_state.flow)
        self.xSpinBox.setValue(self.desired_state.x)
        self.zSpinBox.setValue(self.desired_state.z)
        self.alphaSpinBox.setValue(self.desired_state.alpha)
        self.rSpinBox.setValue(self.desired_state.r)
        self.thetaSpinBox.setValue(self.desired_state.theta)
        self.offsetSpinBox.setValue(self.desired_state.o)
        
        self.freqSpinBox.setValue(self.desired_state.vibrato_freq)
        self.ampSpinBox.setValue(self.desired_state.vibrato_amp)
        #self.desired_state.change_state(self.state)

    def change_x(self, value):
        if not self.changing_other:
            #print("x", value)
            self.changing_other = True
            self.desired_state.x = value
            #self.musician.move_to(self.desired_state, only_x=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, True, False, False, False, self.speed])
            self.update_values()
            #flutist.moveTo(self.state, onlyCartesian=True)
            self.changing_other = False
    
    def change_z(self, value):
        if not self.changing_other:
            #print("z")
            self.changing_other = True
            self.desired_state.z = value
            #print(value, self.state.z)
            #self.musician.move_to(self.desired_state, only_z=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, True, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_angle(self, value):
        if not self.changing_other:
            #print("alpha", value)
            self.changing_other = True
            self.desired_state.alpha = value
            #self.musician.move_to(self.desired_state, only_alpha=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, True, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_r(self, value):
        if not self.changing_other:
            #print("r")
            self.changing_other = True
            self.desired_state.r = value
            #self.musician.move_to(self.desired_state)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_theta(self, value):
        if not self.changing_other:
            #print("theta")
            self.changing_other = True
            self.desired_state.theta = value
            #self.musician.move_to(self.desired_state)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_offset(self, value):
        if not self.changing_other:
            #print("o")
            self.changing_other = True
            self.desired_state.o = value
            #self.musician.move_to(self.desired_state)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False
    
    def change_flow(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.flow = value
            #self.musician.move_to(self.desired_state, only_flow=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed])
            self.changing_other = False

    def change_flow_vibrato(self, value):
        if not self.changing_other:
            #print("flow vibrato")
            self.changing_other = True
            self.desired_state.vibrato_freq = value
            #self.musician.move_to(self.desired_state, only_flow=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed])
            self.changing_other = False

    def change_flow_vibrato_amp(self, value):
        if not self.changing_other:
            #print("flow vibrato")
            self.changing_other = True
            self.desired_state.vibrato_amp = value
            #self.musician.move_to(self.desired_state, only_flow=True)
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed])
            self.changing_other = False

    def item_changed(self, item):
        # Get the new value of the item
        new_value = item.text()
        # Get the row and column of the item
        row = item.row()
        col = item.column()
        # Get the key corresponding to the row and col
        key = self.tableWidget.verticalHeaderItem(row).text()
        key2 = self.tableWidget.horizontalHeaderItem(col).text().split(" ")[0]

        # Check the input is a number
        try:
            new_value = float(new_value)
        except:
            item.setText(str(LOOK_UP_TABLE[key][key2]))
            return
        
        # Update the value in the dictionary
        LOOK_UP_TABLE[key][key2] = new_value

        with open('new_interface/look_up_table.json', 'w') as file:
            json.dump(LOOK_UP_TABLE, file, indent=4, sort_keys=False)
        


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ManualWindow(app, connected=False)
    win.show()

    sys.exit(app.exec())