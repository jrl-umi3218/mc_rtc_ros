# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'task_dialog.ui'
#
# Created: Tue Apr 30 17:15:51 2013
#      by: pyside-uic 0.2.13 running on PySide 1.1.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_TaskDialog(object):
    def setupUi(self, TaskDialog):
        TaskDialog.setObjectName("TaskDialog")
        TaskDialog.resize(341, 225)
        self.verticalLayout = QtGui.QVBoxLayout(TaskDialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.taskTable = QtGui.QTableWidget(TaskDialog)
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
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.buttonBox = QtGui.QDialogButtonBox(TaskDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.horizontalLayout.addWidget(self.buttonBox)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(TaskDialog)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), TaskDialog.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), TaskDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(TaskDialog)

    def retranslateUi(self, TaskDialog):
        TaskDialog.setWindowTitle(QtGui.QApplication.translate("TaskDialog", "Task", None, QtGui.QApplication.UnicodeUTF8))

