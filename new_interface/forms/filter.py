# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'filter.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(403, 576)
        self.gridLayout_4 = QtWidgets.QGridLayout(Dialog)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.filter_choice = QtWidgets.QComboBox(Dialog)
        self.filter_choice.setObjectName("filter_choice")
        self.filter_choice.addItem("")
        self.filter_choice.addItem("")
        self.filter_choice.addItem("")
        self.filter_choice.addItem("")
        self.filter_choice.addItem("")
        self.gridLayout_4.addWidget(self.filter_choice, 2, 1, 1, 1)
        self.time_i = QtWidgets.QDoubleSpinBox(Dialog)
        self.time_i.setObjectName("time_i")
        self.gridLayout_4.addWidget(self.time_i, 0, 1, 1, 1)
        self.time_f = QtWidgets.QDoubleSpinBox(Dialog)
        self.time_f.setObjectName("time_f")
        self.gridLayout_4.addWidget(self.time_f, 1, 1, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout_4.addWidget(self.buttonBox, 5, 1, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_4.addItem(spacerItem, 3, 0, 1, 1)
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setObjectName("label")
        self.gridLayout_4.addWidget(self.label, 0, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 2, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        self.gridLayout_4.addWidget(self.label_2, 1, 0, 1, 1)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.windowGroup = QtWidgets.QGroupBox(Dialog)
        self.windowGroup.setObjectName("windowGroup")
        self.gridLayout = QtWidgets.QGridLayout(self.windowGroup)
        self.gridLayout.setObjectName("gridLayout")
        self.window_layout = QtWidgets.QGridLayout()
        self.window_layout.setObjectName("window_layout")
        self.label_6 = QtWidgets.QLabel(self.windowGroup)
        self.label_6.setObjectName("label_6")
        self.window_layout.addWidget(self.label_6, 2, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.windowGroup)
        self.label_5.setObjectName("label_5")
        self.window_layout.addWidget(self.label_5, 1, 0, 1, 1)
        self.window_choice = QtWidgets.QComboBox(self.windowGroup)
        self.window_choice.setObjectName("window_choice")
        self.window_choice.addItem("")
        self.window_choice.addItem("")
        self.window_choice.addItem("")
        self.window_choice.addItem("")
        self.window_choice.addItem("")
        self.window_layout.addWidget(self.window_choice, 0, 1, 1, 1)
        self.window_n = QtWidgets.QSpinBox(self.windowGroup)
        self.window_n.setMaximum(999)
        self.window_n.setObjectName("window_n")
        self.window_layout.addWidget(self.window_n, 1, 1, 1, 1)
        self.cutoff = QtWidgets.QDoubleSpinBox(self.windowGroup)
        self.cutoff.setObjectName("cutoff")
        self.window_layout.addWidget(self.cutoff, 2, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.windowGroup)
        self.label_4.setObjectName("label_4")
        self.window_layout.addWidget(self.label_4, 0, 0, 1, 1)
        self.gridLayout.addLayout(self.window_layout, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.windowGroup)
        self.OtherGroup = QtWidgets.QGroupBox(Dialog)
        self.OtherGroup.setObjectName("OtherGroup")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.OtherGroup)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.normal_layout = QtWidgets.QGridLayout()
        self.normal_layout.setObjectName("normal_layout")
        self.label_7 = QtWidgets.QLabel(self.OtherGroup)
        self.label_7.setObjectName("label_7")
        self.normal_layout.addWidget(self.label_7, 0, 0, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.OtherGroup)
        self.label_11.setObjectName("label_11")
        self.normal_layout.addWidget(self.label_11, 3, 0, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.OtherGroup)
        self.label_8.setObjectName("label_8")
        self.normal_layout.addWidget(self.label_8, 1, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.OtherGroup)
        self.label_10.setObjectName("label_10")
        self.normal_layout.addWidget(self.label_10, 2, 0, 1, 1)
        self.Ap = QtWidgets.QDoubleSpinBox(self.OtherGroup)
        self.Ap.setObjectName("Ap")
        self.normal_layout.addWidget(self.Ap, 0, 1, 1, 1)
        self.As = QtWidgets.QDoubleSpinBox(self.OtherGroup)
        self.As.setObjectName("As")
        self.normal_layout.addWidget(self.As, 1, 1, 1, 1)
        self.fp = QtWidgets.QDoubleSpinBox(self.OtherGroup)
        self.fp.setObjectName("fp")
        self.normal_layout.addWidget(self.fp, 2, 1, 1, 1)
        self.fs = QtWidgets.QDoubleSpinBox(self.OtherGroup)
        self.fs.setObjectName("fs")
        self.normal_layout.addWidget(self.fs, 3, 1, 1, 1)
        self.gridLayout_2.addLayout(self.normal_layout, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.OtherGroup)
        self.ChebGroup = QtWidgets.QGroupBox(Dialog)
        self.ChebGroup.setObjectName("ChebGroup")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.ChebGroup)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.cheb_layout = QtWidgets.QGridLayout()
        self.cheb_layout.setObjectName("cheb_layout")
        self.label_9 = QtWidgets.QLabel(self.ChebGroup)
        self.label_9.setObjectName("label_9")
        self.cheb_layout.addWidget(self.label_9, 1, 0, 1, 1)
        self.chebfs = QtWidgets.QDoubleSpinBox(self.ChebGroup)
        self.chebfs.setObjectName("chebfs")
        self.cheb_layout.addWidget(self.chebfs, 4, 1, 1, 1)
        self.chebAs = QtWidgets.QDoubleSpinBox(self.ChebGroup)
        self.chebAs.setObjectName("chebAs")
        self.cheb_layout.addWidget(self.chebAs, 2, 1, 1, 1)
        self.chebAp = QtWidgets.QDoubleSpinBox(self.ChebGroup)
        self.chebAp.setObjectName("chebAp")
        self.cheb_layout.addWidget(self.chebAp, 1, 1, 1, 1)
        self.chebfp = QtWidgets.QDoubleSpinBox(self.ChebGroup)
        self.chebfp.setObjectName("chebfp")
        self.cheb_layout.addWidget(self.chebfp, 3, 1, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.ChebGroup)
        self.label_12.setObjectName("label_12")
        self.cheb_layout.addWidget(self.label_12, 4, 0, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.ChebGroup)
        self.label_15.setObjectName("label_15")
        self.cheb_layout.addWidget(self.label_15, 5, 0, 1, 1)
        self.label_14 = QtWidgets.QLabel(self.ChebGroup)
        self.label_14.setObjectName("label_14")
        self.cheb_layout.addWidget(self.label_14, 3, 0, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.ChebGroup)
        self.label_13.setObjectName("label_13")
        self.cheb_layout.addWidget(self.label_13, 2, 0, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.ChebGroup)
        self.label_16.setObjectName("label_16")
        self.cheb_layout.addWidget(self.label_16, 0, 0, 1, 1)
        self.chebN = QtWidgets.QComboBox(self.ChebGroup)
        self.chebN.setObjectName("chebN")
        self.chebN.addItem("")
        self.chebN.addItem("")
        self.cheb_layout.addWidget(self.chebN, 0, 1, 1, 1)
        self.chebrp = QtWidgets.QDoubleSpinBox(self.ChebGroup)
        self.chebrp.setObjectName("chebrp")
        self.cheb_layout.addWidget(self.chebrp, 5, 1, 1, 1)
        self.gridLayout_3.addLayout(self.cheb_layout, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.ChebGroup)
        self.gridLayout_4.addLayout(self.verticalLayout, 4, 0, 1, 2)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.filter_choice.setItemText(0, _translate("Dialog", "firwin"))
        self.filter_choice.setItemText(1, _translate("Dialog", "remez"))
        self.filter_choice.setItemText(2, _translate("Dialog", "butter"))
        self.filter_choice.setItemText(3, _translate("Dialog", "chebyshev"))
        self.filter_choice.setItemText(4, _translate("Dialog", "elliptic"))
        self.label.setText(_translate("Dialog", "Starting at"))
        self.label_3.setText(_translate("Dialog", "Filter"))
        self.label_2.setText(_translate("Dialog", "Ending at"))
        self.windowGroup.setTitle(_translate("Dialog", "Filter parameters"))
        self.label_6.setText(_translate("Dialog", "Cutoff"))
        self.label_5.setText(_translate("Dialog", "N"))
        self.window_choice.setItemText(0, _translate("Dialog", "hamming"))
        self.window_choice.setItemText(1, _translate("Dialog", "hann"))
        self.window_choice.setItemText(2, _translate("Dialog", "blackman"))
        self.window_choice.setItemText(3, _translate("Dialog", "bartlett"))
        self.window_choice.setItemText(4, _translate("Dialog", "rect"))
        self.label_4.setText(_translate("Dialog", "Window"))
        self.OtherGroup.setTitle(_translate("Dialog", "Filter parameters"))
        self.label_7.setText(_translate("Dialog", "Ap"))
        self.label_11.setText(_translate("Dialog", "fs"))
        self.label_8.setText(_translate("Dialog", "As"))
        self.label_10.setText(_translate("Dialog", "fp"))
        self.ChebGroup.setTitle(_translate("Dialog", "Filter parameters"))
        self.label_9.setText(_translate("Dialog", "Ap"))
        self.label_12.setText(_translate("Dialog", "fs"))
        self.label_15.setText(_translate("Dialog", "rp"))
        self.label_14.setText(_translate("Dialog", "fp"))
        self.label_13.setText(_translate("Dialog", "As"))
        self.label_16.setText(_translate("Dialog", "N"))
        self.chebN.setItemText(0, _translate("Dialog", "1"))
        self.chebN.setItemText(1, _translate("Dialog", "2"))
