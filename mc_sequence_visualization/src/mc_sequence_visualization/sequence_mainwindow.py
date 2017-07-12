# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sequence_mainwindow.ui'
#
# Created: Tue Oct 29 14:07:59 2013
#      by: pyside-uic 0.2.13 running on PySide 1.1.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_Sequence(object):
    def setupUi(self, Sequence):
        Sequence.setObjectName("Sequence")
        Sequence.resize(359, 211)
        self.centralwidget = QtGui.QWidget(Sequence)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.stanceSlider = QtGui.QSlider(self.centralwidget)
        self.stanceSlider.setOrientation(QtCore.Qt.Horizontal)
        self.stanceSlider.setTickPosition(QtGui.QSlider.NoTicks)
        self.stanceSlider.setObjectName("stanceSlider")
        self.horizontalLayout.addWidget(self.stanceSlider)
        self.stanceSpinBox = QtGui.QSpinBox(self.centralwidget)
        self.stanceSpinBox.setObjectName("stanceSpinBox")
        self.horizontalLayout.addWidget(self.stanceSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.contactTable = QtGui.QTableWidget(self.centralwidget)
        self.contactTable.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.contactTable.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.contactTable.setProperty("showDropIndicator", False)
        self.contactTable.setDragDropOverwriteMode(False)
        self.contactTable.setAlternatingRowColors(True)
        self.contactTable.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        self.contactTable.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        self.contactTable.setWordWrap(True)
        self.contactTable.setColumnCount(0)
        self.contactTable.setObjectName("contactTable")
        self.contactTable.setColumnCount(0)
        self.contactTable.setRowCount(0)
        self.contactTable.horizontalHeader().setCascadingSectionResizes(False)
        self.contactTable.horizontalHeader().setHighlightSections(False)
        self.contactTable.horizontalHeader().setStretchLastSection(True)
        self.contactTable.verticalHeader().setHighlightSections(False)
        self.verticalLayout.addWidget(self.contactTable)
        Sequence.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(Sequence)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 359, 21))
        self.menubar.setObjectName("menubar")
        self.menu_Fichier = QtGui.QMenu(self.menubar)
        self.menu_Fichier.setObjectName("menu_Fichier")
        self.menu_diter = QtGui.QMenu(self.menubar)
        self.menu_diter.setObjectName("menu_diter")
        Sequence.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(Sequence)
        self.statusbar.setObjectName("statusbar")
        Sequence.setStatusBar(self.statusbar)
        self.undo = QtGui.QAction(Sequence)
        self.undo.setEnabled(False)
        self.undo.setObjectName("undo")
        self.redo = QtGui.QAction(Sequence)
        self.redo.setEnabled(False)
        self.redo.setObjectName("redo")
        self.jointState = QtGui.QAction(Sequence)
        self.jointState.setObjectName("jointState")
        self.menu_Fichier.addAction(self.jointState)
        self.menu_diter.addAction(self.undo)
        self.menu_diter.addAction(self.redo)
        self.menubar.addAction(self.menu_Fichier.menuAction())
        self.menubar.addAction(self.menu_diter.menuAction())

        self.retranslateUi(Sequence)
        QtCore.QMetaObject.connectSlotsByName(Sequence)

    def retranslateUi(self, Sequence):
        Sequence.setWindowTitle(QtGui.QApplication.translate("Sequence", "Sequences editor", None, QtGui.QApplication.UnicodeUTF8))
        self.menu_Fichier.setTitle(QtGui.QApplication.translate("Sequence", "&Fichier", None, QtGui.QApplication.UnicodeUTF8))
        self.menu_diter.setTitle(QtGui.QApplication.translate("Sequence", "&Edit", None, QtGui.QApplication.UnicodeUTF8))
        self.undo.setText(QtGui.QApplication.translate("Sequence", "&Undo", None, QtGui.QApplication.UnicodeUTF8))
        self.undo.setShortcut(QtGui.QApplication.translate("Sequence", "Ctrl+Z", None, QtGui.QApplication.UnicodeUTF8))
        self.redo.setText(QtGui.QApplication.translate("Sequence", "&Redo", None, QtGui.QApplication.UnicodeUTF8))
        self.redo.setShortcut(QtGui.QApplication.translate("Sequence", "Ctrl+Shift+Z", None, QtGui.QApplication.UnicodeUTF8))
        self.jointState.setText(QtGui.QApplication.translate("Sequence", "&JointStates…", None, QtGui.QApplication.UnicodeUTF8))

