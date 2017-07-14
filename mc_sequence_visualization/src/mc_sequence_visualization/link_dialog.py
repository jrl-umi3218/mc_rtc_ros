# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'link_dialog.ui'
#
# Created: Wed May 27 17:46:29 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_LinkDialog(object):
    def setupUi(self, LinkDialog):
        LinkDialog.setObjectName("LinkDialog")
        LinkDialog.resize(749, 580)
        self.verticalLayout = QtGui.QVBoxLayout(LinkDialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.linkLabel = QtGui.QLabel(LinkDialog)
        self.linkLabel.setObjectName("linkLabel")
        self.horizontalLayout_2.addWidget(self.linkLabel)
        self.linkSelectBox = QtGui.QComboBox(LinkDialog)
        self.linkSelectBox.setObjectName("linkSelectBox")
        self.horizontalLayout_2.addWidget(self.linkSelectBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.taskTable = QtGui.QTableWidget(LinkDialog)
        self.taskTable.setEditTriggers(QtGui.QAbstractItemView.DoubleClicked)
        self.taskTable.setProperty("showDropIndicator", False)
        self.taskTable.setDragDropOverwriteMode(False)
        self.taskTable.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        self.taskTable.setObjectName("taskTable")
        self.taskTable.setColumnCount(0)
        self.taskTable.setRowCount(0)
        self.taskTable.horizontalHeader().setStretchLastSection(True)
        self.taskTable.verticalHeader().setVisible(False)
        self.verticalLayout.addWidget(self.taskTable)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.jointLabel = QtGui.QLabel(LinkDialog)
        self.jointLabel.setObjectName("jointLabel")
        self.horizontalLayout_3.addWidget(self.jointLabel)
        self.jointSelectBox = QtGui.QComboBox(LinkDialog)
        self.jointSelectBox.setObjectName("jointSelectBox")
        self.horizontalLayout_3.addWidget(self.jointSelectBox)
        self.jointLineEdit = QtGui.QLineEdit(LinkDialog)
        self.jointLineEdit.setMaximumSize(QtCore.QSize(608, 16777215))
        self.jointLineEdit.setObjectName("jointLineEdit")
        self.horizontalLayout_3.addWidget(self.jointLineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.qlLabel = QtGui.QLabel(LinkDialog)
        self.qlLabel.setObjectName("qlLabel")
        self.horizontalLayout_4.addWidget(self.qlLabel)
        self.jointSlider = QtGui.QSlider(LinkDialog)
        self.jointSlider.setOrientation(QtCore.Qt.Horizontal)
        self.jointSlider.setObjectName("jointSlider")
        self.horizontalLayout_4.addWidget(self.jointSlider)
        self.quLabel = QtGui.QLabel(LinkDialog)
        self.quLabel.setObjectName("quLabel")
        self.horizontalLayout_4.addWidget(self.quLabel)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.buttonBox = QtGui.QDialogButtonBox(LinkDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.horizontalLayout.addWidget(self.buttonBox)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(LinkDialog)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), LinkDialog.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), LinkDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(LinkDialog)

    def retranslateUi(self, LinkDialog):
        LinkDialog.setWindowTitle(QtGui.QApplication.translate("LinkDialog", "Task", None, QtGui.QApplication.UnicodeUTF8))
        self.linkLabel.setText(QtGui.QApplication.translate("LinkDialog", "Link", None, QtGui.QApplication.UnicodeUTF8))
        self.jointLabel.setText(QtGui.QApplication.translate("LinkDialog", "Joint", None, QtGui.QApplication.UnicodeUTF8))
        self.qlLabel.setText(QtGui.QApplication.translate("LinkDialog", "Min", None, QtGui.QApplication.UnicodeUTF8))
        self.quLabel.setText(QtGui.QApplication.translate("LinkDialog", "Max", None, QtGui.QApplication.UnicodeUTF8))

